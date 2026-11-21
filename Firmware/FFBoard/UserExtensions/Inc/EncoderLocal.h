/*
 * EncoderLocal.h
 *
 *  Created on: 02.02.2020
 *      Author: Yannick
 */

#ifndef ENCODERLOCAL_H_
#define ENCODERLOCAL_H_

#include "cppmain.h"
#include <Encoder.h>
#include "constants.h"
#include "ExtiHandler.h"
#include "TimerHandler.h"
#include "CommandHandler.h"
#include "PersistentStorage.h"

#ifdef LOCALENCODER
#include <array>
extern TIM_HandleTypeDef TIM_ENC;

#ifdef LOCALENCODER_MULTI
#define LOCALENCODER_INSTANCES LOCALENCODER_MULTI
/**
 * Timer and pins of one local encoder instance (multi instance targets).
 * The timer is configured in encoder mode at runtime. htim is only used to dispatch the overflow interrupt:
 * its IRQ handler must call HAL_TIM_IRQHandler(htim).
 */
struct EncoderLocalConfig{
	TIM_TypeDef* tim;
	TIM_HandleTypeDef* htim;
	IRQn_Type irq;
	GPIO_TypeDef* aPort;
	uint16_t aPin;
	GPIO_TypeDef* bPort;
	uint16_t bPin;
	uint8_t af;
	GPIO_TypeDef* zPort = nullptr; // Index pin (EXTI). nullptr = no index
	uint16_t zPin = 0;
	IRQn_Type zIrq = (IRQn_Type)0;
	uint32_t zMode = GPIO_MODE_IT_RISING; // Same edge as the CubeMX config of ENCODER_Z
	uint8_t filter = 5; // Input filter (0-15) for A and B
};
#else
#define LOCALENCODER_INSTANCES 1
#endif



class EncoderLocal: public Encoder,public ExtiHandler,TimerHandler,public CommandHandler, public PersistentStorage{
	enum class EncoderLocal_commands : uint32_t{
		cpr,useindex
	};
public:
	static std::array<bool,LOCALENCODER_INSTANCES> inUse;
	static ClassIdentifier info;
	const ClassIdentifier getInfo();
	static bool isCreatable() {return isInstanceCreatable(0);};
	static bool isInstanceCreatable(uint8_t instance) {return instance < LOCALENCODER_INSTANCES && !inUse[instance];}

	EncoderLocal(uint8_t instance = 0);
	virtual ~EncoderLocal();

	EncoderType getEncoderType();

	void registerCommands();

	int32_t getPos();
	void setPos(int32_t pos);
	//void setOffset(int32_t offset);
	void setPeriod(uint32_t period);
	void overflowCallback();
	void exti(uint16_t GPIO_Pin) override;
	void timerElapsed(TIM_HandleTypeDef* htim);

	int32_t getTimerCount();

	void setCpr(uint32_t cpr);	// Encoder counts per rotation

	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies);
	std::string getHelpstring(){return "Local ABN encoder\n";}
	void saveFlash(); 		// Write to flash here
	void restoreFlash();	// Load from flash

	//const ClassType getClassType() override {return ClassType::Encoder;};
#ifdef LOCALENCODER_MULTI
	static const std::array<EncoderLocalConfig,LOCALENCODER_MULTI> configs; // One entry per instance. Defined by the target
#endif
private:
	const uint8_t instance;
	TIM_HandleTypeDef* htim;
	bool isIndexPin(uint16_t GPIO_Pin);
#ifdef LOCALENCODER_MULTI
	void initMultiTimer();
#endif
	int16_t offset = 0;
	int32_t pos = 0; // Extra position counter for overflows
	uint32_t indexpos = 0;

	bool indexHit = false;
	bool useIndex = false;
};
#ifdef LOCALENCODER_MULTI
/**
 * Local encoder for further axes on multi instance targets. INST is the instance (1 = second axis)
 */
template<uint8_t INST>
class EncoderLocal_N : public EncoderLocal{
	static_assert(INST > 0 && INST < LOCALENCODER_MULTI, "Local encoder instance not defined by the target");
public:
	EncoderLocal_N() : EncoderLocal(INST){}
	static bool isCreatable(){return EncoderLocal::isInstanceCreatable(INST);}
};
#endif
#endif
#endif /* ENCODERLOCAL_H_ */
