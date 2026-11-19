/*
 * MotorPWM.cpp
 *
 *  Created on: Mar 29, 2020
 *      Author: Yannick
 */

#include <MotorPWM.h>
#ifdef PWMDRIVER

#include "cpp_target_config.h"

/*
 * Mapping of names for ModePWM_DRV
 */
const std::vector<std::string> RC_SpeedNames = {"20ms","15ms","10ms","5ms"};
const std::vector<std::string> PWM_SpeedNames = {"3khz","9khz","17khz","24khz"};
// Names of SpeedPWM_DRV
const std::vector<std::string> PwmModeNames = {"RC PPM","0%-50%-100% Centered","0-100% PWM/DIR","0-100% dual PWM"};
bool MotorPWM::pwmDriverInUse = false;

ClassIdentifier MotorPWM::info = {
		 .name = "PWM" ,
		 .id=CLSID_MOT_PWM,
 };
const ClassIdentifier MotorPWM::getInfo(){
	return info;
}

bool MotorPWM::isCreatable(){
	return !MotorPWM::pwmDriverInUse; // Creatable if not already in use for example by another axis
}


// Thread stack in words. Uses the FPU (larger context) and encoder calls. 128 overflowed and corrupted the heap
MotorPWM::MotorPWM() : CommandHandler("pwmdrv",CLSID_MOT_PWM), Thread("PWMZERO",512,20) {

	MotorPWM::pwmDriverInUse = true;
	restoreFlash();
	//HAL_TIM_Base_Start_IT(timer);
	setPwmSpeed(pwmspeed);

	CommandHandler::registerCommands();
	registerCommand("freq", MotorPWM_commands::freq, "PWM period selection",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("mode", MotorPWM_commands::mode, "PWM mode",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("dir", MotorPWM_commands::dir, "Invert direction",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("duty", MotorPWM_commands::duty, "Current PWM duty of output in 0.01%",CMDFLAG_GET);
	registerCommand("torque", MotorPWM_commands::torque, "Current torque output (-32767 to 32767)",CMDFLAG_GET);
	registerCommand("hz", MotorPWM_commands::hz, "Actual PWM frequency in Hz",CMDFLAG_GET);
	registerCommand("active", MotorPWM_commands::active, "Output enabled",CMDFLAG_GET);
	registerCommand("minduty", MotorPWM_commands::minduty, "PWM/DIR duty at zero torque in 0.1% (0-254)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("autozero", MotorPWM_commands::autozero, "Find zero torque duty at startup (PWM/DIR)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("calibrate", MotorPWM_commands::calibrate, "Run auto zero now (set 1)",CMDFLAG_SET);
	registerCommand("calstate", MotorPWM_commands::calstate, "Auto zero state: 0 not run, 1 running, 2 ok, 3 failed",CMDFLAG_GET);
	registerCommand("zero", MotorPWM_commands::zero, "Zero torque duty in use in 0.01%",CMDFLAG_GET);
	registerCommand("calfail", MotorPWM_commands::calfail, "Auto zero fail reason: 0 none, 1 not PWM/DIR, 2 no encoder, 3 motor stopped, 4 no forward movement up to max duty, 5 turned too far, 6 still turning at max duty, 7 no backward movement down to min duty",CMDFLAG_GET);
	registerCommand("calduty", MotorPWM_commands::calduty, "Auto zero: found zero (or duty when it failed) in 0.01%",CMDFLAG_GET);
	registerCommand("calhigh", MotorPWM_commands::calhigh, "Auto zero: duty where it started turning forward in 0.01%",CMDFLAG_GET);
	registerCommand("callow", MotorPWM_commands::callow, "Auto zero: duty where it started turning backwards in 0.01%",CMDFLAG_GET);
	registerCommand("caltravel", MotorPWM_commands::caltravel, "Auto zero encoder counts moved",CMDFLAG_GET);
	registerCommand("calstartv", MotorPWM_commands::calstartv, "Auto zero encoder counts moved in 100ms before ramping",CMDFLAG_GET);

	this->Start();
}

MotorPWM::~MotorPWM() {
	MotorPWM::pwmDriverInUse = false;
	HAL_TIM_PWM_Stop(timerConfig.timer, timerConfig.channel_1);
	HAL_TIM_PWM_Stop(timerConfig.timer, timerConfig.channel_2);
	HAL_TIM_PWM_Stop(timerConfig.timer, timerConfig.channel_3);
	HAL_TIM_PWM_Stop(timerConfig.timer, timerConfig.channel_4);
	HAL_TIM_Base_Stop_IT(timerConfig.timer);
}



void MotorPWM::turn(int16_t power){
	if(!active || calibrating)
		return;

	if(invertDir){
		power = -power;
	}
	lastPower = power;
	/*
	 * Generates a classic RC 20ms 1000-2000µs signal
	 * Centered at 1500µs for bidirectional RC ESCs and similiar stuff
	 */
	if(mode == ModePWM_DRV::RC_PWM){
		int32_t pval = power;

		float val = ((pval * 1000)/0x7fff)*tFreq;
		val = clip((1500*tFreq)-val,1000*tFreq, 2000*tFreq);
		setPWM(val,timerConfig.rcpwm_chan);

	/*
	 * Generates a 0-100% PWM signal
	 * and outputs complementary direction signals.
	 * Can be used with cheap halfbridge modules and DC motors
	 */
	}else if(mode == ModePWM_DRV::PWM_DIR){
		if(power < 0){
			setPWM(0,timerConfig.dir_chan);
			setPWM(0xffff,timerConfig.dir_chan_n);
		}else{
			setPWM(0,timerConfig.dir_chan_n);
			setPWM(0xffff,timerConfig.dir_chan);
		}
		// Zero torque = zeroPulse(), full torque = 100%. Never a constant low level. See minDacPulse()
		uint32_t minval = zeroPulse();
		uint32_t val = minval + (uint32_t)(((uint64_t)abs(power) * (period - minval))/0x7fff);
		setPWM(val,timerConfig.pwm_chan);

	}else if(mode == ModePWM_DRV::CENTERED_PWM){
		int32_t pval = 0x7fff+power;
		int32_t val = (pval * period)/0xffff;
		setPWM(val,timerConfig.centerpwm_chan);

	}else if(mode == ModePWM_DRV::PWM_DUAL){
		int32_t val = (uint32_t)((abs(power) * period)/0x7fff);
		if(power < 0){
			setPWM(0,timerConfig.dualpwm1);
			setPWM(val,timerConfig.dualpwm2);
		}else{
			setPWM(0,timerConfig.dualpwm2);
			setPWM(val,timerConfig.dualpwm1);
		}
	}
}

/**
 * Calculates period and prescaler for a timer based on frequency
 */
std::pair<uint16_t,uint16_t> MotorPWM::freqToPeriodPsc(uint32_t freq){
	uint32_t arr_raw = (timerConfig.timerFreq/freq);
	uint32_t prescaler = arr_raw / 0xffff;
	uint32_t arr = arr_raw / (prescaler + 1);
	return {arr,prescaler};
}

/**
 * Setup the timer for different frequency presets.
 */
void MotorPWM::setPwmSpeed(SpeedPWM_DRV spd){
	bool ok = true;
	switch(spd){

	case SpeedPWM_DRV::LOW:
		if(mode == ModePWM_DRV::RC_PWM){
			period =  40000;  //20ms (40000/Sysclock)
			prescaler = timerConfig.timerFreq/2000000;
		}else{
			std::tie(period,prescaler) = freqToPeriodPsc(3000);
		}

	break;
	case SpeedPWM_DRV::MID:
		if(mode == ModePWM_DRV::RC_PWM){
			period = 30000;//15ms(30000/47)
			prescaler = timerConfig.timerFreq/2000000;
		}else{
			std::tie(period,prescaler) = freqToPeriodPsc(9000);
		}

	break;
	case SpeedPWM_DRV::HIGH:
		if(mode == ModePWM_DRV::RC_PWM){
			period = 20000; //10ms (20000/47)
			prescaler = timerConfig.timerFreq/2000000;
		}else{
			std::tie(period,prescaler) = freqToPeriodPsc(17000);
		}
	break;
	case SpeedPWM_DRV::VERYHIGH:
		if(mode == ModePWM_DRV::RC_PWM){
			period = 10000; //5ms (20000/23)
			prescaler = timerConfig.timerFreq/2000000;
		}else{
			std::tie(period,prescaler) = freqToPeriodPsc(24000);
		}
	break;
	default:
		ok = false;
	}

	if(ok){
		this->pwmspeed = spd;
		tFreq = (float)(timerConfig.timerFreq/1000000)/(float)(prescaler+1);

		pwmInitTimer(timerConfig.timer, timerConfig.channel_1,period,prescaler);
		pwmInitTimer(timerConfig.timer, timerConfig.channel_2,period,prescaler);
		pwmInitTimer(timerConfig.timer, timerConfig.channel_3,period,prescaler);
		pwmInitTimer(timerConfig.timer, timerConfig.channel_4,period,prescaler);
		HAL_TIM_MspPostInit(timerConfig.timer);
//		setPWM_HAL(0, timer, channel_1, period);
//		pwmInitTimer(timer, channel_2,period,prescaler);
//		setPWM_HAL(0, timer, channel_2, period);
		// Also before the motor is started so a PWM DAC never idles at startup. See minDacPulse()
		setIdleOutput();
		turn(0);
	}
}

/**
 * Updates pwm pulse length
 */
void MotorPWM::setPWM(uint32_t value,uint8_t ccr){
	if(ccr == 1){
		timerConfig.timer->Instance->CCR1 = value; // Set next CCR for channel 1
	}else if(ccr == 2){
		timerConfig.timer->Instance->CCR2 = value; // Set next CCR for channel 2
	}else if(ccr == 3){
		timerConfig.timer->Instance->CCR3 = value; // Set next CCR for channel 3
	}else if(ccr == 4){
		timerConfig.timer->Instance->CCR4 = value; // Set next CCR for channel 4
	}

}

/**
 * Reads back the current pulse length of a channel
 */
uint32_t MotorPWM::getPWM(uint8_t ccr){
	switch(ccr){
	case 1: return timerConfig.timer->Instance->CCR1;
	case 2: return timerConfig.timer->Instance->CCR2;
	case 3: return timerConfig.timer->Instance->CCR3;
	case 4: return timerConfig.timer->Instance->CCR4;
	default: return 0;
	}
}

/**
 * Duty cycle of the channel that carries the torque signal in the current mode, in 0.01%
 */
uint32_t MotorPWM::getDuty(){
	uint32_t ccr = 0;
	switch(mode){
	case ModePWM_DRV::RC_PWM: ccr = getPWM(timerConfig.rcpwm_chan); break;
	case ModePWM_DRV::CENTERED_PWM: ccr = getPWM(timerConfig.centerpwm_chan); break;
	case ModePWM_DRV::PWM_DIR: ccr = getPWM(timerConfig.pwm_chan); break;
	case ModePWM_DRV::PWM_DUAL: ccr = std::max(getPWM(timerConfig.dualpwm1),getPWM(timerConfig.dualpwm2)); break;
	}
	uint32_t arr = timerConfig.timer->Instance->ARR + 1;
	return std::min<uint32_t>(((uint64_t)ccr * 10000) / arr, 10000);
}

uint32_t MotorPWM::getFrequency(){
	uint32_t ticks = (timerConfig.timer->Instance->PSC + 1) * (timerConfig.timer->Instance->ARR + 1);
	return ticks ? timerConfig.timerFreq / ticks : 0;
}

SpeedPWM_DRV MotorPWM::getPwmSpeed(){
	return this->pwmspeed;
}




void MotorPWM::saveFlash(){
	// 0-3: mode
	// 4-6: speed
	// 3: auto zero disabled (0 = enabled so it is on by default)
	// 7-14: min duty. 0 = default so older saved settings get the default. Else minDuty + 1
	uint16_t var = (uint8_t)this->mode & 0x7;
	var |= (autoZeroEnabled ? 0 : 1) << 3;
	var |= ((uint8_t)this->pwmspeed & 0x7) << 4;
	uint8_t md = (minDuty == MINDUTY_DEFAULT) ? 0 : minDuty + 1;
	var |= (md & 0xff) << 7;
	var |= (this->invertDir & 0x1) << 15; // Last bit inversion
	Flash_Write(ADR_PWM_MODE, var);
}
void MotorPWM::restoreFlash(){
	uint16_t var = 0;
	if(Flash_Read(ADR_PWM_MODE, &var)){
		uint8_t md = (var >> 7) & 0xff;
		this->minDuty = (md == 0) ? MINDUTY_DEFAULT : md - 1;
		this->autoZeroEnabled = ((var >> 3) & 0x1) == 0;

		uint8_t m = var & 0x7;
		this->setMode(ModePWM_DRV(m));

		uint8_t s = (var >> 4) & 0x7;
		this->setPwmSpeed(SpeedPWM_DRV(s));

		this->invertDir = (var >> 15) & 0x1;
	}
}

void MotorPWM::setMinDuty(uint16_t val){
	minDuty = std::min(val,MINDUTY_MAX);
	// When active the next turn() call (every axis update) applies it
	if(!active){
		setIdleOutput();
	}
}

/**
 * Sets the PWM/DIR outputs to zero torque like turn(0) but also when not active or calibrating
 */
void MotorPWM::setIdleOutput(){
	if(mode != ModePWM_DRV::PWM_DIR)
		return;
	setPWM(0,timerConfig.dir_chan_n);
	setPWM(0xffff,timerConfig.dir_chan);
	setPWM(zeroPulse(),timerConfig.pwm_chan);
}


void MotorPWM::startMotor(){
	active = true;
	if(autoZeroEnabled && mode == ModePWM_DRV::PWM_DIR && autoZeroState == AutoZeroState::NOT_RUN){
		autoZeroRequested = true; // Once after power up. Thread runs it
	}
	turn(0);
}

void MotorPWM::stopMotor(){
	turn(0);
	// May be called from an interrupt (e-stop). Mask interrupts instead of a RTOS critical section
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	active = false;
	if(calibrating){
		setIdleOutput(); // Cancel calibration output immediately. Thread aborts on next step
	}
	if(!primask) __enable_irq();
}

bool MotorPWM::motorReady(){
	// Block torque from the axis until auto zero is done
	return active && !calibrating && !autoZeroRequested;
}

/**
 * Writes the calibration pulse with direction positive. Only if the motor is still enabled
 * @return false if calibration must be aborted
 */
bool MotorPWM::calSetPWM(uint32_t val){
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	bool ok = active && mode == ModePWM_DRV::PWM_DIR;
	if(ok){
		setPWM(0,timerConfig.dir_chan_n);
		setPWM(0xffff,timerConfig.dir_chan);
		setPWM(val,timerConfig.pwm_chan);
	}
	if(!primask) __enable_irq();
	return ok;
}

void MotorPWM::Run(){
	while(true){
		Delay(50);
		if(!autoZeroRequested)
			continue;

		Delay(AUTOZERO_STARTDELAY_MS);
		if(!active){
			autoZeroRequested = false; // Stopped meanwhile. Try again at next start
			continue;
		}
		calibrating = true; // Set before clearing the request so motorReady() stays false
		autoZeroRequested = false;
		autoZeroState = AutoZeroState::RUNNING;

		float frac = 0;
		bool ok = runAutoZero(&frac);

		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		if(ok){
			autoZeroFrac = frac;
		}
		autoZeroState = ok ? AutoZeroState::OK : AutoZeroState::FAILED;
		calibrating = false;
		setIdleOutput();
		if(!primask) __enable_irq();
	}
}

/**
 * Finds the duty where the servo drive outputs no torque. Starts at minDuty.
 * 1. If the wheel already turns, ramp up fast until it stops (zero point was above minDuty).
 * 2. Ramp up slowly until it starts turning forward: upper edge of the friction band.
 * 3. Ramp down slowly until it stopped and starts turning backwards: lower edge.
 * The zero point is the middle of both edges.
 * The wheel must be free. Aborts if the motor is stopped (e-stop), the duty leaves its limits
 * or the wheel turns more than AUTOZERO_MAXTRAVEL_TURNS.
 * @param zeroFrac found zero torque duty as fraction of the period
 * @return true if a zero point was found
 */
bool MotorPWM::runAutoZero(float* zeroFrac){
	calFail = AutoZeroFail::NONE;
	calEndDuty = 0;
	calTravel = 0;
	calStartV = 0;
	calHigh = 0;
	calLow = 0;
	Encoder* enc = getEncoder();
	if(mode != ModePWM_DRV::PWM_DIR){
		calFail = AutoZeroFail::NOT_PWMDIR;
		return false;
	}
	if(enc == nullptr || enc->getCpr() == 0){
		calFail = AutoZeroFail::NO_ENCODER;
		return false;
	}
	const int32_t cpr = enc->getCpr();
	const int32_t moveThres = std::max<int32_t>(cpr / 360, 4); // 1°
	const int32_t stopThres = std::max<int32_t>(cpr / 1200, 3); // 0.3° per velocity window (100ms)
	const int32_t maxTravel = cpr * AUTOZERO_MAXTRAVEL_TURNS;
	const uint32_t floorPulse = (period + 4095) / 4096; // Keep the DAC input toggling
	const uint32_t fastStep = std::max<uint32_t>(((uint32_t)period * AUTOZERO_FAST_STEP) / 10000, 1);
	const uint32_t slowStep = std::max<uint32_t>(((uint32_t)period * AUTOZERO_SLOW_STEP) / 10000, 1);
	const uint32_t maxPulse = ((uint32_t)period * AUTOZERO_MAXDUTY_PERCENT) / 100;
	auto toDuty = [&](uint32_t p){ // pulse to 0.01%
		return (uint32_t)(((uint64_t)p * 10000) / (period + 1));
	};

	// Position history for the velocity over the last AUTOZERO_HIST steps
	int32_t hist[AUTOZERO_HIST];
	uint32_t histIdx = 0;
	auto velocity = [&](int32_t p){
		int32_t vel = p - hist[histIdx];
		hist[histIdx] = p;
		histIdx = (histIdx + 1) % AUTOZERO_HIST;
		return vel;
	};

	uint32_t pulse = std::min(std::max(minDacPulse(), floorPulse), maxPulse);
	int32_t origin = enc->getPos();
	int32_t pos = origin;
	int32_t v = 0; // Encoder counts moved in the last velocity window
	AutoZeroFail err = AutoZeroFail::NONE;

	// Records why and where it stopped for the diagnostics commands
	auto fail = [&](AutoZeroFail reason){
		calFail = reason;
		calEndDuty = toDuty(pulse);
		calTravel = enc->getPos() - origin;
		return false;
	};
	// Sets a pulse, waits one step and measures. False if aborted (err is set)
	auto step = [&](uint32_t p){
		pulse = p;
		if(!calSetPWM(pulse)){
			err = AutoZeroFail::STOPPED;
			return false;
		}
		Delay(AUTOZERO_STEP_MS);
		pos = enc->getPos();
		v = velocity(pos);
		if(abs(pos - origin) > maxTravel){
			err = AutoZeroFail::TRAVEL;
			return false;
		}
		return true;
	};

	if(!calSetPWM(pulse)) return fail(AutoZeroFail::STOPPED);
	Delay(300);
	origin = enc->getPos();
	for(uint32_t i = 0; i < AUTOZERO_HIST; i++){
		hist[i] = origin;
	}
	for(uint32_t i = 0; i < AUTOZERO_HIST; i++){ // Fill the history so a wheel already turning is detected
		if(!step(pulse)) return fail(err);
	}
	calStartV = v;

	int32_t fwd = 0; // Encoder direction of increasing duty. 0 = not known yet

	// 1. Wheel already turns: the zero point is above. Ramp up fast until it stops
	if(abs(v) >= stopThres){
		const int32_t s0 = v > 0 ? 1 : -1;
		fwd = -s0;
		while(v * s0 >= stopThres){
			if(pulse + fastStep > maxPulse) return fail(AutoZeroFail::NOT_STOPPING);
			if(!step(pulse + fastStep)) return fail(err);
		}
	}

	// 2. Slowly up until it turns forward: upper edge
	int32_t startPos = pos;
	while(true){
		int32_t d = pos - startPos;
		if(fwd == 0){
			if(abs(d) > moveThres){
				fwd = d > 0 ? 1 : -1;
				break;
			}
		}else{
			if(d * fwd > moveThres){
				break;
			}
			if(d * fwd < -moveThres){
				startPos = pos; // Still creeping backwards. Below the band
			}
		}
		if(pulse + slowStep > maxPulse) return fail(AutoZeroFail::NO_MOVEMENT);
		if(!step(pulse + slowStep)) return fail(err);
	}
	const uint32_t highPulse = pulse;
	calHigh = toDuty(highPulse);

	// 3. Slowly down until it stopped and turns backwards: lower edge
	bool stopped = false;
	startPos = pos;
	while(true){
		if(pulse < floorPulse + slowStep) return fail(AutoZeroFail::NO_LOW_EDGE);
		if(!step(pulse - slowStep)) return fail(err);
		if(!stopped){
			if(abs(v) < stopThres){
				stopped = true;
				startPos = pos;
			}
		}else if((pos - startPos) * fwd < -moveThres){
			break;
		}
	}
	const uint32_t lowPulse = pulse;
	calLow = toDuty(lowPulse);

	const uint32_t zero = (lowPulse + highPulse) / 2;
	*zeroFrac = (float)zero / (float)period;
	calEndDuty = toDuty(zero);
	calTravel = enc->getPos() - origin;
	return true;
}


void MotorPWM::setMode(ModePWM_DRV mode){
	this->mode = mode;
	setPwmSpeed(pwmspeed); // Reinit timer
}

ModePWM_DRV MotorPWM::getMode(){
	return this->mode;
}



CommandStatus MotorPWM::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){

	switch(static_cast<MotorPWM_commands>(cmd.cmdId)){

	case MotorPWM_commands::freq:
	{
		if(cmd.type == CMDtype::set){
			this->setPwmSpeed((SpeedPWM_DRV)cmd.val);
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back((uint8_t)this->getPwmSpeed());
		}else if(cmd.type == CMDtype::info){
			std::vector<std::string> names = PWM_SpeedNames;
			if(this->mode == ModePWM_DRV::RC_PWM){
				names = RC_SpeedNames;
			}else{
				names = PWM_SpeedNames;
			}
			for(uint8_t i = 0; i<names.size();i++){
				replies.emplace_back(names[i]  + ":" + std::to_string(i)+"\n");
			}
		}
		break;
	}
	case MotorPWM_commands::mode:
	{
		if(cmd.type == CMDtype::set){
			this->setMode((ModePWM_DRV)cmd.val);
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back((uint8_t)this->getMode());
		}else if(cmd.type == CMDtype::info){
			for(uint8_t i = 0; i<PwmModeNames.size();i++){
				replies.emplace_back(PwmModeNames[i]  + ":" + std::to_string(i)+"\n");
			}
		}
		break;
	}
	case MotorPWM_commands::dir:
		return handleGetSet(cmd, replies, this->invertDir);
	case MotorPWM_commands::duty:
		replies.emplace_back(getDuty());
		break;
	case MotorPWM_commands::torque:
		replies.emplace_back(active ? lastPower : 0);
		break;
	case MotorPWM_commands::hz:
		replies.emplace_back(getFrequency());
		break;
	case MotorPWM_commands::active:
		replies.emplace_back(active ? 1 : 0);
		break;
	case MotorPWM_commands::minduty:
		if(cmd.type == CMDtype::set){
			setMinDuty(cmd.val);
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back(minDuty);
		}
		break;
	case MotorPWM_commands::autozero:
		return handleGetSet(cmd, replies, this->autoZeroEnabled);
	case MotorPWM_commands::calibrate:
		if(cmd.type == CMDtype::set && cmd.val != 0){
			if(!active || mode != ModePWM_DRV::PWM_DIR || calibrating){
				return CommandStatus::ERR;
			}
			autoZeroRequested = true;
		}
		break;
	case MotorPWM_commands::calstate:
		replies.emplace_back((uint8_t)autoZeroState);
		break;
	case MotorPWM_commands::calfail:
		replies.emplace_back((uint8_t)calFail);
		break;
	case MotorPWM_commands::calduty:
		replies.emplace_back(calEndDuty);
		break;
	case MotorPWM_commands::caltravel:
		replies.emplace_back(calTravel);
		break;
	case MotorPWM_commands::calstartv:
		replies.emplace_back(calStartV);
		break;
	case MotorPWM_commands::calhigh:
		replies.emplace_back(calHigh);
		break;
	case MotorPWM_commands::callow:
		replies.emplace_back(calLow);
		break;
	case MotorPWM_commands::zero:
		replies.emplace_back(((uint64_t)zeroPulse() * 10000) / (timerConfig.timer->Instance->ARR + 1));
		break;
	default:
		return CommandStatus::NOT_FOUND;
	}

	return CommandStatus::OK;

}



cpp_freertos::MutexStandard pwmTimMutex;
void pwmInitTimer(TIM_HandleTypeDef* timer,uint32_t channel,uint32_t period,uint32_t prescaler){
	timer->Instance->ARR = period;
	timer->Instance->PSC = prescaler;
	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	pwmTimMutex.Lock();
	HAL_TIM_PWM_Stop(timer, channel);
	HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, channel);
	//setPWM_HAL(0,timer,channel,period);
	HAL_TIM_PWM_Start(timer, channel);
	pwmTimMutex.Unlock();

}


/**
 * Changes the pwm value of the timer via HAL
 */
void setPWM_HAL(uint32_t value,TIM_HandleTypeDef* timer,uint32_t channel,uint32_t period){
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, channel);
    HAL_TIM_PWM_Start(timer, channel);
}
#endif
