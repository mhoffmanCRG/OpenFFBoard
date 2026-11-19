/*
 * HX711.h
 *
 *  Bit-banged driver for HX711 24 bit load cell ADCs and an analog source using them.
 *
 *  Multiple chips are supported. Each chip has its own DOUT pin and a SCK pin.
 *  Chips that share a SCK pin form a group: they are clocked together and all data lines
 *  are sampled on every clock, so a shared clock costs no extra time per chip.
 *  Pins are defined by the target in HX711_PINS (see target_constants.h).
 */

#ifndef USEREXTENSIONS_INC_HX711_H_
#define USEREXTENSIONS_INC_HX711_H_

#include "constants.h"
#include "target_constants.h"

#ifdef HX711ANALOG
#include <array>
#include <vector>
#include "cppmain.h"
#include "AnalogSource.h"
#include "CommandHandler.h"
#include "PersistentStorage.h"
#include "AnalogAxisProcessing.h"
#include "thread.hpp"

struct HX711PinConfig{
	GPIO_TypeDef* sckPort;
	uint16_t sckPin;
	GPIO_TypeDef* doutPort;
	uint16_t doutPin;
};

/**
 * Input channel and gain for the next conversion.
 * Selected by the number of extra clock pulses after the 24 data bits.
 */
enum class HX711_Gain : uint8_t {
	A128 = 0, // Channel A, gain 128 (25 pulses)
	A64 = 1,  // Channel A, gain 64  (27 pulses)
	B32 = 2   // Channel B, gain 32  (26 pulses)
};

/**
 * Low level driver for one or more HX711 chips
 */
class HX711 {
public:
	static constexpr uint8_t MAX_CHIPS = 4; // Limited by the reserved flash addresses
	static constexpr uint32_t TIMEOUT_MS = 500; // A chip is considered disconnected if not ready for this long (10 SPS = 100 ms)

	HX711(const HX711PinConfig* pins, uint8_t pinCount);
	virtual ~HX711();

	void setChips(uint8_t count);
	uint8_t getChips() const {return chips;}
	uint8_t getMaxChips() const {return maxChips;}

	void setGainMode(HX711_Gain gain);
	HX711_Gain getGainMode() const {return gain;}

	bool isReady(uint8_t chip) const;
	uint8_t getAliveMask() const;

	uint8_t poll(int32_t* out);

protected:
	void configurePins();
	void releasePins(uint8_t fromChip = 0);
	void readGroup(uint8_t group, int32_t* out);

	struct Group{
		GPIO_TypeDef* sckPort = nullptr;
		uint16_t sckPin = 0;
		uint8_t memberCount = 0;
		std::array<uint8_t,MAX_CHIPS> members = {0};
	};

	const HX711PinConfig* pins;
	const uint8_t maxChips;
	uint8_t chips = 0;
	uint8_t groupCount = 0;
	std::array<Group,MAX_CHIPS> groups;
	std::array<uint32_t,MAX_CHIPS> lastReady = {0};
	volatile HX711_Gain gain = HX711_Gain::A128;
};


/**
 * Analog source reading up to MAX_CHIPS HX711 chips. One axis per chip.
 *
 * Output per axis: ((raw - tare offset) >> shift), optionally inverted, clipped to 16 bit
 * and then passed through the usual analog processing (filter, autorange or manual min/max).
 * Bipolar mode: no load = 50% (0), negative load below that.
 * Pedal mode (default): no load = 0% (-32767), negative load clipped to 0%, 65534 shifted counts = 100%.
 */
class HX711_AnalogSource : public HX711, public AnalogSource, public CommandHandler, public AnalogAxisProcessing, public cpp_freertos::Thread {
	enum class HX711_AnalogSource_commands : uint32_t {
		chips,maxchips,gain,shift,invert,autotare,tare,offset,counts,status,pedal
	};

public:
	HX711_AnalogSource();
	~HX711_AnalogSource();

	const ClassIdentifier getInfo();
	static ClassIdentifier info;
	static bool isCreatable() {return true;};
	std::string getHelpstring(){return "HX711 load cell ADC analog source";}
	const ClassType getClassType() override {return ClassType::Analogsource;};

	void saveFlash();
	void restoreFlash();

	void Run();

	std::vector<int32_t>* getAxes();
	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies);

	void tare(uint8_t mask);

	static constexpr uint8_t TARE_SAMPLES = 8;
	static constexpr uint32_t AUTOTARE_DELAY_MS = 1000; // Settling time after startup before the automatic tare

private:
	int32_t getTaredCounts(uint8_t chip) const;

	volatile uint8_t requestedChips = 1; // Applied by the sampling thread
	uint8_t shift = 8; // 24 bit -> 16 bit by default
	uint8_t invertMask = 0;
	uint8_t pedalMask = (1 << MAX_CHIPS) - 1; // Pedal mode: no load = 0% (-32767), counts>>shift = 65534 -> 100%
	bool autotare = true;

	std::array<volatile int32_t,MAX_CHIPS> rawCounts = {0};
	std::array<volatile int32_t,MAX_CHIPS> offsets = {0};

	volatile uint8_t tareRequest = 0; // Set by commands, consumed by the thread
	volatile uint8_t tareBusy = 0; // Chips currently being tared
	std::array<int32_t,MAX_CHIPS> tareSum = {0};
	std::array<uint8_t,MAX_CHIPS> tareCount = {0};
};

#endif /* HX711ANALOG */
#endif /* USEREXTENSIONS_INC_HX711_H_ */
