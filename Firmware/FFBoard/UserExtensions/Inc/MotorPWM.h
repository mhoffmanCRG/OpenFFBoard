/*
 * MotorPWM.h
 *
 *  Created on: Mar 29, 2020
 *      Author: Yannick
 */

#ifndef MOTORPWM_H_
#define MOTORPWM_H_

#include <MotorDriver.h>
#include <array>
#include "cppmain.h"
#include "CommandHandler.h"
#include "TimerHandler.h"
#include "target_constants.h"
//#include "cpp_target_config.h"
#include "PersistentStorage.h"
#include "thread.hpp"

extern TIM_HandleTypeDef TIM_PWM;

void pwmInitTimer(TIM_HandleTypeDef* timer,uint32_t channel,uint32_t period,uint32_t prescaler);
void setPWM_HAL(uint32_t value,TIM_HandleTypeDef* timer,uint32_t channel,uint32_t period);

enum class ModePWM_DRV : uint8_t {RC_PWM=0,CENTERED_PWM=1,PWM_DIR=2,PWM_DUAL=3};
enum class SpeedPWM_DRV : uint8_t {LOW=0,MID=1,HIGH=2,VERYHIGH=3};

struct PWMConfig{
		uint32_t channel_1 = TIM_CHANNEL_1;
		uint32_t channel_2 = TIM_CHANNEL_2;
		uint32_t channel_3 = TIM_CHANNEL_3;
		uint32_t channel_4 = TIM_CHANNEL_4;

		uint8_t pwm_chan = 1;
		uint8_t	dir_chan = 3;
		uint8_t	dir_chan_n = 4;

		uint8_t centerpwm_chan = 1;

		uint8_t rcpwm_chan = 1;

		uint8_t dualpwm1 = 1;
		uint8_t dualpwm2 = 2;

		TIM_HandleTypeDef* timer = &TIM_PWM;
		uint32_t timerFreq = 168000000;

		/*
		 * Optional, used by multi instance targets (PWMDRIVER_MULTI):
		 * dirPort/dirPin: GPIO direction output for PWM/DIR mode instead of the dir_chan/dir_chan_n timer channels.
		 * pwmPort/pwmPin/pwmAf: PWM output pin, configured at runtime instead of HAL_TIM_MspPostInit.
		 * pwmComplementary: pwm_chan is output on the complementary CHxN pin (advanced timers).
		 */
		GPIO_TypeDef* dirPort = nullptr;
		uint16_t dirPin = 0;
		GPIO_TypeDef* pwmPort = nullptr;
		uint16_t pwmPin = 0;
		uint8_t pwmAf = 0;
		bool pwmComplementary = false;
	};

#ifdef PWMDRIVER_MULTI
#define PWMDRIVER_INSTANCES PWMDRIVER_MULTI
#else
#define PWMDRIVER_INSTANCES 1
#endif

/**
 * Contains motor drivers methods based on PWM generation.
 *
 * Can output different types of PWM signals based on a torque input.
 */

class MotorPWM: public MotorDriver,public CommandHandler,public PersistentStorage,public cpp_freertos::Thread{
	enum class MotorPWM_commands : uint32_t {
		mode,freq,dir,duty,torque,hz,active,minduty,autozero,calibrate,calstate,zero,calfail,calduty,caltravel,calstartv,calhigh,callow
	};
public:
	enum class AutoZeroState : uint8_t {NOT_RUN=0,RUNNING=1,OK=2,FAILED=3};
	enum class AutoZeroFail : uint8_t {NONE=0,NOT_PWMDIR=1,NO_ENCODER=2,STOPPED=3,NO_MOVEMENT=4,TRAVEL=5,NOT_STOPPING=6,NO_LOW_EDGE=7};
public:
	/**
	 * @param instance driver instance. 0 on single PWM driver targets. Multi instance targets (PWMDRIVER_MULTI)
	 * use one timer channel and DIR pin per instance from MotorPWM::multiConfigs and share the PWM frequency.
	 */
	MotorPWM(uint8_t instance = 0);
	virtual ~MotorPWM();

	static ClassIdentifier info;
	const ClassIdentifier getInfo();
	static bool isCreatable();
	static bool isInstanceCreatable(uint8_t instance);

	void turn(int16_t power);
	void stopMotor();
	void startMotor();
	bool motorReady() override;

	void Run(); // Auto zero thread

	void setPwmSpeed(SpeedPWM_DRV spd);
	SpeedPWM_DRV getPwmSpeed();

	void setMode(ModePWM_DRV mode);
	ModePWM_DRV getMode();

	void saveFlash(); 		// Write to flash here
	void restoreFlash();	// Load from flash

	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies);
	virtual std::string getHelpstring(){return "PWM output motor driver";}

	void setPWM(uint32_t value,uint8_t ccr);

	static std::array<bool,PWMDRIVER_INSTANCES> instanceInUse;

private:
	const uint8_t instance;
	const PWMConfig& cfg; // Timer and pins of this instance
	bool isMulti() const; // Shares the timer with other instances. Only touches its own channel
	void setDir(bool positive); // Direction output for PWM/DIR mode
	void initMultiOutput(); // Pins and channel of this instance (multi instance)
	void applySpeed(SpeedPWM_DRV spd,int32_t period,int32_t prescaler); // Takes over a new timer period
	void refreshOutput(); // Rewrites the output after a period change
	int16_t lastInput = 0; // Last torque passed to turn() before inversion
#ifdef PWMDRIVER_MULTI
	static std::array<MotorPWM*,PWMDRIVER_INSTANCES> instances; // Live instances. They share the PWM frequency
#endif

	float tFreq = 1; // Frequency scaling. Timer freq in MHz
	int32_t period = 20000;
	int32_t prescaler = 95;

	bool invertDir = false;


	SpeedPWM_DRV pwmspeed = SpeedPWM_DRV::LOW;
	ModePWM_DRV mode = ModePWM_DRV::CENTERED_PWM;
	bool active = false;
	int16_t lastPower = 0; // Last torque sent to the output (after direction inversion)

	static const uint16_t MINDUTY_DEFAULT = 10; // 1%
	static const uint16_t MINDUTY_MAX = 254; // 25.4%
	uint16_t minDuty = MINDUTY_DEFAULT; // PWM/DIR duty at zero torque in 0.1% steps
	void setMinDuty(uint16_t val);

	/*
	 * Auto zero (PWM/DIR mode): on the first motor start after power up, starting at minDuty:
	 * 1. If the wheel already turns (servo drive zero far above) ramp up fast until it stops.
	 * 2. Ramp up slowly until it starts turning forward (upper edge of the friction band).
	 * 3. Ramp down slowly until it stops and starts turning backwards (lower edge).
	 * The middle of both edges is used as zero torque point instead of minDuty.
	 * Finds the servo drives analog zero when its reference floats.
	 */
	bool autoZeroEnabled = true;
	volatile bool autoZeroRequested = false;
	volatile bool calibrating = false;
	volatile AutoZeroState autoZeroState = AutoZeroState::NOT_RUN;
	float autoZeroFrac = 0; // Found zero duty as fraction of the period. 0 = not calibrated, use minDuty
	// Diagnostics of the last auto zero run
	AutoZeroFail calFail = AutoZeroFail::NONE;
	uint32_t calEndDuty = 0; // Duty at the end in 0.01%
	int32_t calTravel = 0; // Encoder counts from start position at the end
	int32_t calStartV = 0; // Encoder counts moved in the velocity window before ramping (wheel already turning?)
	uint32_t calHigh = 0; // Duty where it started turning forward in 0.01%
	uint32_t calLow = 0; // Duty where it started turning backwards in 0.01%
	bool runAutoZero(float* zeroFrac);
	bool calSetPWM(uint32_t val); // Writes pulse only if still active. Returns false if aborted
	uint32_t zeroPulse() const {
		if(autoZeroFrac <= 0)
			return minDacPulse();
		return std::max<uint32_t>((uint32_t)(autoZeroFrac * period), (period + 4095) / 4096);
	}
	void setIdleOutput(); // Zero torque output in PWM/DIR mode without changing active state

	static const uint32_t AUTOZERO_STARTDELAY_MS = 2000; // Let the servo drive start
	static const uint32_t AUTOZERO_STEP_MS = 20;
	static const uint32_t AUTOZERO_FAST_STEP = 10; // 0.1% per step = 5%/s while the wheel still turns
	static const uint32_t AUTOZERO_SLOW_STEP = 1; // 0.01% per step = 0.5%/s to find the edges
	static const uint32_t AUTOZERO_MAXDUTY_PERCENT = 25; // Abort above this duty
	static const int32_t AUTOZERO_MAXTRAVEL_TURNS = 3; // Abort if the wheel turned further than this from the start
	static const uint32_t AUTOZERO_HIST = 5; // Velocity window = AUTOZERO_HIST * AUTOZERO_STEP_MS

	uint32_t getPWM(uint8_t ccr);
	uint32_t getDuty(); // Duty of the main output channel in 0.01%
	uint32_t getFrequency(); // Actual PWM frequency in Hz

	/**
	 * The timer channels are mapped in the cpp_target_config files
	 */
#ifdef PWMDRIVER_MULTI
public:
	static const std::array<PWMConfig,PWMDRIVER_MULTI> multiConfigs; // One entry per instance
private:
#else
	static const PWMConfig timerConfig;
#endif
	std::pair<uint16_t,uint16_t> freqToPeriodPsc(uint32_t freq);

	/**
	 * Pulse at zero torque in PWM/DIR mode (minDuty, default 1%). Torque is scaled from here to 100%.
	 * PWM-to-voltage DACs like the LTC2644/LTC2645 (e.g. on MMOS boards) enter an idle state
	 * (zero-scale or high impedance, depending on IDLSEL) if the input stays at a constant level for >60ms.
	 * A high impedance output floats and gives a random offset voltage. Keeping the input toggling
	 * makes the DAC actively drive a known voltage at zero torque that the servo drive offset can be zeroed to.
	 * Never less than 1/4096 of the period so the DAC can't idle even with minDuty 0.
	 */
	uint32_t minDacPulse() const {
		uint32_t p = ((uint64_t)period * minDuty) / 1000;
		return std::max<uint32_t>(p, (period + 4095) / 4096);
	}
};

#ifdef PWMDRIVER_MULTI
/**
 * PWM driver for further axes on multi instance targets. INST is the instance (1 = second axis)
 */
template<uint8_t INST>
class MotorPWM_N : public MotorPWM{
	static_assert(INST > 0 && INST < PWMDRIVER_MULTI, "PWM instance not defined by the target");
public:
	MotorPWM_N() : MotorPWM(INST){}
	static bool isCreatable(){return MotorPWM::isInstanceCreatable(INST);}
};
#endif


#endif /* MOTORPWM_H_ */
