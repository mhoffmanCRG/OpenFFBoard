/*
 * HX711.cpp
 *
 *  Bit-banged driver for HX711 24 bit load cell ADCs and an analog source using them.
 *
 *  Protocol: DOUT goes low when a conversion is ready. 24 clock pulses shift out the
 *  sample MSB first (two's complement), followed by 1-3 pulses selecting channel/gain
 *  of the next conversion. SCK high for more than 60 us powers the chip down, so each
 *  high phase runs with interrupts disabled (about 1 us).
 */

#include "HX711.h"

#ifdef HX711ANALOG
#include "eeprom_addresses.h"
#include "flash_helpers.h"

/**
 * Busy wait of roughly 1 us (at least 0.2 us is required for the SCK high and low phases)
 */
static inline void hx711Delay(){
	const uint32_t loops = SystemCoreClock / 4000000U;
	for(volatile uint32_t i = 0; i < loops; i++){}
}

static void enableGpioClock(GPIO_TypeDef* port){
#ifdef RCC_AHB1ENR_GPIOAEN
	// STM32F4: GPIO ports are spaced 0x400 apart and their clock enable bits are consecutive starting at GPIOA
	RCC->AHB1ENR |= 1U << (((uint32_t)port - GPIOA_BASE) / 0x400U);
	(void)RCC->AHB1ENR;
#else
	(void)port;
#endif
}

HX711::HX711(const HX711PinConfig* pins, uint8_t pinCount) : pins(pins), maxChips(std::min<uint8_t>(pinCount, MAX_CHIPS)) {

}

HX711::~HX711() {
	releasePins(0);
}

/**
 * Changes the number of used chips (entries of the pin table) and configures their pins
 */
void HX711::setChips(uint8_t count){
	count = clip<uint8_t,uint8_t>(count, 1, maxChips);
	releasePins(count);
	chips = count;
	configurePins();
}

/**
 * Sets the input channel and gain for the following conversions of all chips.
 * The first sample after a change still uses the previous setting.
 */
void HX711::setGainMode(HX711_Gain gain){
	if((uint8_t)gain > (uint8_t)HX711_Gain::B32){
		gain = HX711_Gain::A128;
	}
	this->gain = gain;
}

/**
 * Configures SCK as output (low = chip running) and DOUT as input and builds the SCK groups
 */
void HX711::configurePins(){
	groupCount = 0;
	const uint32_t now = HAL_GetTick();
	for(uint8_t i = 0; i < chips; i++){
		const HX711PinConfig& p = pins[i];
		enableGpioClock(p.sckPort);
		enableGpioClock(p.doutPort);

		HAL_GPIO_WritePin(p.sckPort, p.sckPin, GPIO_PIN_RESET);
		GPIO_InitTypeDef init = {0};
		init.Pin = p.sckPin;
		init.Mode = GPIO_MODE_OUTPUT_PP;
		init.Pull = GPIO_NOPULL;
		init.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(p.sckPort, &init);

		init.Pin = p.doutPin;
		init.Mode = GPIO_MODE_INPUT;
		init.Pull = GPIO_PULLUP; // A missing chip reads as "not ready"
		HAL_GPIO_Init(p.doutPort, &init);

		// Find or create the group for this SCK pin
		uint8_t g = 0;
		for(; g < groupCount; g++){
			if(groups[g].sckPort == p.sckPort && groups[g].sckPin == p.sckPin){
				break;
			}
		}
		if(g == groupCount){
			groups[g].sckPort = p.sckPort;
			groups[g].sckPin = p.sckPin;
			groups[g].memberCount = 0;
			groupCount++;
		}
		groups[g].members[groups[g].memberCount++] = i;
		lastReady[i] = now; // Assume connected until the timeout expires
	}
}

/**
 * Releases SCK pins of chips >= fromChip that are not shared with a remaining chip
 */
void HX711::releasePins(uint8_t fromChip){
	for(uint8_t i = fromChip; i < chips; i++){
		bool shared = false;
		for(uint8_t j = 0; j < fromChip; j++){
			if(pins[j].sckPort == pins[i].sckPort && pins[j].sckPin == pins[i].sckPin){
				shared = true;
				break;
			}
		}
		if(!shared){
			GPIO_InitTypeDef init = {0};
			init.Pin = pins[i].sckPin;
			init.Mode = GPIO_MODE_INPUT;
			init.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(pins[i].sckPort, &init);
		}
	}
}

bool HX711::isReady(uint8_t chip) const{
	return (pins[chip].doutPort->IDR & pins[chip].doutPin) == 0;
}

/**
 * Bitmask of chips that produced a sample within TIMEOUT_MS
 */
uint8_t HX711::getAliveMask() const{
	const uint32_t now = HAL_GetTick();
	uint8_t mask = 0;
	for(uint8_t i = 0; i < chips; i++){
		if(now - lastReady[i] < TIMEOUT_MS){
			mask |= 1 << i;
		}
	}
	return mask;
}

/**
 * Reads every group whose connected chips are all ready.
 * Chips sharing a clock must be read together, so a group waits for its slowest connected chip.
 * Disconnected chips (not ready for TIMEOUT_MS) do not block their group.
 * @param[out] out sample per chip (indexed by chip number). Only chips set in the returned mask are written.
 * @return bitmask of chips with a new sample
 */
uint8_t HX711::poll(int32_t* out){
	uint8_t newSamples = 0;
	const uint8_t alive = getAliveMask();
	for(uint8_t g = 0; g < groupCount; g++){
		const Group& group = groups[g];
		uint8_t readyMask = 0;
		uint8_t groupMask = 0;
		for(uint8_t m = 0; m < group.memberCount; m++){
			const uint8_t chip = group.members[m];
			groupMask |= 1 << chip;
			if(isReady(chip)){
				readyMask |= 1 << chip;
			}
		}
		const uint8_t waitMask = alive & groupMask;
		if(readyMask == 0 || (readyMask & waitMask) != waitMask){
			continue; // Nothing ready or waiting for another connected chip on this clock
		}
		readGroup(g, out);
		const uint32_t now = HAL_GetTick();
		for(uint8_t m = 0; m < group.memberCount; m++){
			const uint8_t chip = group.members[m];
			if(readyMask & (1 << chip)){
				lastReady[chip] = now;
			}
		}
		newSamples |= readyMask;
	}
	return newSamples;
}

/**
 * Clocks out one sample of every chip in a group (all data lines sampled per clock)
 */
void HX711::readGroup(uint8_t groupIdx, int32_t* out){
	const Group& group = groups[groupIdx];
	std::array<uint32_t,MAX_CHIPS> vals = {0};
	const uint8_t extraPulses = gain == HX711_Gain::A128 ? 1 : (gain == HX711_Gain::B32 ? 2 : 3);
	const uint32_t sckSet = group.sckPin;
	const uint32_t sckReset = (uint32_t)group.sckPin << 16;

	for(uint8_t bit = 0; bit < 24 + extraPulses; bit++){
		const uint32_t primask = __get_PRIMASK();
		__disable_irq(); // Keep the high phase short. >60 us would power down the chip
		group.sckPort->BSRR = sckSet;
		hx711Delay();
		if(bit < 24){
			for(uint8_t m = 0; m < group.memberCount; m++){
				const uint8_t chip = group.members[m];
				vals[chip] = (vals[chip] << 1) | ((pins[chip].doutPort->IDR & pins[chip].doutPin) ? 1 : 0);
			}
		}
		group.sckPort->BSRR = sckReset;
		__set_PRIMASK(primask);
		hx711Delay();
	}

	for(uint8_t m = 0; m < group.memberCount; m++){
		const uint8_t chip = group.members[m];
		out[chip] = ((int32_t)(vals[chip] << 8)) >> 8; // Sign extend 24 bit two's complement
	}
}



// ---------------------------------------------------------------------- //

ClassIdentifier HX711_AnalogSource::info = {
		 .name = "HX711 load cell" ,
		 .id = CLSID_ANALOG_HX711,
};

const ClassIdentifier HX711_AnalogSource::getInfo(){
	return info;
}

static const HX711PinConfig hx711Pins[] = HX711_PINS;

static const std::array<std::pair<uint16_t,uint16_t>,HX711::MAX_CHIPS> minMaxValAddr = {
	std::pair<uint16_t,uint16_t>(ADR_HX711_MIN_0,ADR_HX711_MAX_0),
	std::pair<uint16_t,uint16_t>(ADR_HX711_MIN_1,ADR_HX711_MAX_1),
	std::pair<uint16_t,uint16_t>(ADR_HX711_MIN_2,ADR_HX711_MAX_2),
	std::pair<uint16_t,uint16_t>(ADR_HX711_MIN_3,ADR_HX711_MAX_3),
};

// Tare offsets (low 16b, high 16b)
static const std::array<std::pair<uint16_t,uint16_t>,HX711::MAX_CHIPS> offsetAddr = {
	std::pair<uint16_t,uint16_t>(ADR_HX711_OFS_L_0,ADR_HX711_OFS_H_0),
	std::pair<uint16_t,uint16_t>(ADR_HX711_OFS_L_1,ADR_HX711_OFS_H_1),
	std::pair<uint16_t,uint16_t>(ADR_HX711_OFS_L_2,ADR_HX711_OFS_H_2),
	std::pair<uint16_t,uint16_t>(ADR_HX711_OFS_L_3,ADR_HX711_OFS_H_3),
};

HX711_AnalogSource::HX711_AnalogSource() :
		HX711(hx711Pins, sizeof(hx711Pins) / sizeof(hx711Pins[0])),
		CommandHandler("hx711", CLSID_ANALOG_HX711, 0),
		AnalogAxisProcessing(HX711::MAX_CHIPS, this, this, true, true, true, true),
		Thread("hx711", 128, 20)
{
	CommandHandler::registerCommands();
	registerCommand("chips", HX711_AnalogSource_commands::chips, "Number of HX711 chips used",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("maxchips", HX711_AnalogSource_commands::maxchips, "Chips defined by the target pin table",CMDFLAG_GET);
	registerCommand("gain", HX711_AnalogSource_commands::gain, "Channel and gain (0=A x128, 1=A x64, 2=B x32)",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("shift", HX711_AnalogSource_commands::shift, "Right shift from 24b counts to 16b axis (0-15)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("invert", HX711_AnalogSource_commands::invert, "Invert axis (bitmask, bit=chip)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("pedal", HX711_AnalogSource_commands::pedal, "Pedal mode: no load = 0% (bitmask, bit=chip)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("autotare", HX711_AnalogSource_commands::autotare, "Tare all chips after startup",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("tare", HX711_AnalogSource_commands::tare, "Tare now (set=all, setadr=chip). Get: chips still taring",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_SETADR);
	registerCommand("offset", HX711_AnalogSource_commands::offset, "Tare offset in counts (adr=chip)",CMDFLAG_GETADR | CMDFLAG_SETADR);
	registerCommand("counts", HX711_AnalogSource_commands::counts, "Tared 24b counts per chip",CMDFLAG_GET);
	registerCommand("status", HX711_AnalogSource_commands::status, "Responding chips (bitmask)",CMDFLAG_GET);

	restoreFlash();
	HX711::setChips(requestedChips);
	this->buf.resize(getChips(), 0);
	this->Start();
}

HX711_AnalogSource::~HX711_AnalogSource(){

}

void HX711_AnalogSource::saveFlash(){
	uint16_t conf = requestedChips & 0x7;
	conf |= ((uint8_t)getGainMode() & 0x3) << 3;
	conf |= (shift & 0xF) << 5;
	conf |= (autotare ? 0 : 1) << 9;
	conf |= (AnalogAxisProcessing::encodeAnalogProcessingConfToInt(AnalogAxisProcessing::getAnalogProcessingConfig()) & 0x3) << 10;
	conf |= (invertMask & 0xF) << 12;
	Flash_Write(ADR_HX711_CONF1, conf);
	Flash_Write(ADR_HX711_CONF2, pedalMask & 0xF);

	AnalogAxisProcessing::saveMinMaxValues(minMaxValAddr);

	for(uint8_t i = 0; i < MAX_CHIPS; i++){
		const uint32_t ofs = (uint32_t)offsets[i];
		Flash_Write(offsetAddr[i].first, ofs & 0xffff);
		Flash_Write(offsetAddr[i].second, (ofs >> 16) & 0xffff);
	}
}

void HX711_AnalogSource::restoreFlash(){
	uint16_t conf;
	if(Flash_Read(ADR_HX711_CONF1, &conf)){
		const uint8_t n = conf & 0x7;
		requestedChips = clip<uint8_t,uint8_t>(n == 0 ? 1 : n, 1, getMaxChips());
		setGainMode((HX711_Gain)((conf >> 3) & 0x3));
		shift = (conf >> 5) & 0xF;
		autotare = ((conf >> 9) & 0x1) == 0;
		AnalogAxisProcessing::setAnalogProcessingConfig(AnalogAxisProcessing::decodeAnalogProcessingConfFromInt((conf >> 10) & 0x3));
		invertMask = (conf >> 12) & 0xF;
	}
	uint16_t conf2;
	if(Flash_Read(ADR_HX711_CONF2, &conf2)){
		pedalMask = conf2 & 0xF;
	}

	AnalogAxisProcessing::restoreMinMaxValues(minMaxValAddr);

	for(uint8_t i = 0; i < MAX_CHIPS; i++){
		uint16_t lo, hi;
		// Do not treat 0xffff as empty. It is a valid half of a negative offset
		if(Flash_Read(offsetAddr[i].first, &lo, false) && Flash_Read(offsetAddr[i].second, &hi, false)){
			offsets[i] = (int32_t)(((uint32_t)hi << 16) | lo);
		}
	}
}

/**
 * Requests a tare (zero) of the chips in mask. Averages TARE_SAMPLES samples.
 */
void HX711_AnalogSource::tare(uint8_t mask){
	__atomic_fetch_or(&tareRequest, mask, __ATOMIC_RELAXED);
}

int32_t HX711_AnalogSource::getTaredCounts(uint8_t chip) const{
	return rawCounts[chip] - offsets[chip];
}

/**
 * Sampling thread. Polls the chips every ms and reads finished conversions (10 or 80 SPS depending on the RATE pin).
 */
void HX711_AnalogSource::Run(){
	const uint32_t startTick = HAL_GetTick();
	bool bootTareDone = !autotare;
	std::array<int32_t,MAX_CHIPS> samples = {0};

	while(true){
		if(requestedChips != getChips()){
			HX711::setChips(requestedChips);
		}
		const uint8_t chipMask = (1 << getChips()) - 1;

		// Start requested tares
		const uint8_t newTares = __atomic_exchange_n(&tareRequest, 0, __ATOMIC_RELAXED) & chipMask;
		for(uint8_t i = 0; i < getChips(); i++){
			if(newTares & (1 << i)){
				tareSum[i] = 0;
				tareCount[i] = 0;
			}
		}
		tareBusy = (tareBusy | newTares) & chipMask & getAliveMask(); // Drop disconnected chips

		const uint8_t newSamples = HX711::poll(samples.data());
		for(uint8_t i = 0; i < getChips(); i++){
			if(!(newSamples & (1 << i))){
				continue;
			}
			rawCounts[i] = samples[i];
			if(tareBusy & (1 << i)){
				tareSum[i] += samples[i];
				if(++tareCount[i] >= TARE_SAMPLES){
					offsets[i] = tareSum[i] / TARE_SAMPLES;
					tareBusy &= ~(1 << i);
				}
			}
		}

		if(!bootTareDone && HAL_GetTick() - startTick > AUTOTARE_DELAY_MS){
			bootTareDone = true;
			tare(chipMask);
		}

		Delay(1);
	}
}

std::vector<int32_t>* HX711_AnalogSource::getAxes(){
	const uint8_t n = getChips();
	const uint8_t alive = getAliveMask();
	this->buf.resize(n);
	for(uint8_t i = 0; i < n; i++){
		int32_t val = 0; // Disconnected chips read as 0 (released)
		if(alive & (1 << i)){
			val = getTaredCounts(i) >> shift;
			if(invertMask & (1 << i)){
				val = -val;
			}
		}
		if(pedalMask & (1 << i)){
			// Unipolar: no load (or below tare) = 0% of the axis
			this->buf[i] = clip<int32_t,int32_t>(val, 0, 0xfffe) - 0x7fff;
		}else{
			this->buf[i] = clip<int32_t,int32_t>(val, -0x7fff, 0x7fff);
		}
	}
	AnalogAxisProcessing::processAxes(this->buf);
	return &this->buf;
}

CommandStatus HX711_AnalogSource::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){

	switch(static_cast<HX711_AnalogSource_commands>(cmd.cmdId)){

	case HX711_AnalogSource_commands::chips:
		if(cmd.type == CMDtype::set){
			requestedChips = clip<uint8_t,uint8_t>(cmd.val, 1, getMaxChips()); // Applied by the thread
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back(requestedChips);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case HX711_AnalogSource_commands::maxchips:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(getMaxChips());
		}else{
			return CommandStatus::ERR;
		}
		break;

	case HX711_AnalogSource_commands::gain:
		if(cmd.type == CMDtype::set){
			setGainMode((HX711_Gain)clip<int64_t,int64_t>(cmd.val, 0, 2));
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back((uint8_t)getGainMode());
		}else if(cmd.type == CMDtype::info){
			replies.emplace_back("A x128 (20mV):0,A x64 (40mV):1,B x32 (80mV):2");
		}else{
			return CommandStatus::ERR;
		}
		break;

	case HX711_AnalogSource_commands::shift:
		if(cmd.type == CMDtype::set){
			shift = clip<int64_t,int64_t>(cmd.val, 0, 15);
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back(shift);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case HX711_AnalogSource_commands::invert:
		if(cmd.type == CMDtype::set){
			invertMask = cmd.val & ((1 << MAX_CHIPS) - 1);
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back(invertMask);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case HX711_AnalogSource_commands::pedal:
		if(cmd.type == CMDtype::set){
			pedalMask = cmd.val & ((1 << MAX_CHIPS) - 1);
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back(pedalMask);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case HX711_AnalogSource_commands::autotare:
		return handleGetSet(cmd, replies, autotare);

	case HX711_AnalogSource_commands::tare:
		if(cmd.type == CMDtype::set){
			tare((1 << getChips()) - 1);
		}else if(cmd.type == CMDtype::setat){
			if(cmd.adr < 0 || cmd.adr >= getChips()){
				return CommandStatus::ERR;
			}
			tare(1 << cmd.adr);
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back(tareBusy | tareRequest);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case HX711_AnalogSource_commands::offset:
		if(cmd.adr < 0 || cmd.adr >= MAX_CHIPS){
			return CommandStatus::ERR;
		}
		if(cmd.type == CMDtype::setat){
			offsets[cmd.adr] = clip<int64_t,int64_t>(cmd.val, -0x800000, 0x7fffff);
		}else if(cmd.type == CMDtype::getat){
			replies.emplace_back(offsets[cmd.adr]);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case HX711_AnalogSource_commands::counts:
		if(cmd.type == CMDtype::get){
			for(uint8_t i = 0; i < getChips(); i++){
				replies.emplace_back(getTaredCounts(i));
			}
		}else{
			return CommandStatus::ERR;
		}
		break;

	case HX711_AnalogSource_commands::status:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(getAliveMask());
		}else{
			return CommandStatus::ERR;
		}
		break;

	default:
		return AnalogAxisProcessing::command(cmd, replies); // filter, autocal, values, rawval, min, max
	}
	return CommandStatus::OK;
}

#endif /* HX711ANALOG */
