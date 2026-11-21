/*
 * EncoderLocal.cpp
 *
 *  Created on: 02.02.2020
 *      Author: Yannick
 */

#include "EncoderLocal.h"
#include "flash_helpers.h"
#ifdef LOCALENCODER
std::array<bool,LOCALENCODER_INSTANCES> EncoderLocal::inUse = {false};
ClassIdentifier EncoderLocal::info = {
		 .name = "Local ABN" ,
		 .id=CLSID_ENCODER_LOCAL,
 };
const ClassIdentifier EncoderLocal::getInfo(){
	return info;
}

// Settings addresses per instance: cpr, offset
static const std::array<std::pair<uint16_t,uint16_t>,4> encLocalAddr = {
	std::pair<uint16_t,uint16_t>(ADR_ENCLOCAL_CPR,ADR_ENCLOCAL_OFS),
	std::pair<uint16_t,uint16_t>(ADR_ENCLOCAL_CPR_2,ADR_ENCLOCAL_OFS_2),
	std::pair<uint16_t,uint16_t>(ADR_ENCLOCAL_CPR_3,ADR_ENCLOCAL_OFS_3),
	std::pair<uint16_t,uint16_t>(ADR_ENCLOCAL_CPR_4,ADR_ENCLOCAL_OFS_4),
};
static_assert(LOCALENCODER_INSTANCES <= 4, "Only 4 local encoder flash addresses defined");

EncoderLocal::EncoderLocal(uint8_t instance) : CommandHandler("localenc",CLSID_ENCODER_LOCAL,instance),
		instance(std::min<uint8_t>(instance, LOCALENCODER_INSTANCES - 1)) {
	EncoderLocal::inUse[this->instance] = true;
	this->restoreFlash();
#ifdef LOCALENCODER_MULTI
	initMultiTimer();
#else
	this->htim = &TIM_ENC;
	HAL_TIM_Base_Start_IT(htim); // May immediately call overflow. Initialize count again
	this->htim->Instance->CNT = 0x7fff;
	pos = 0;
	this->htim->Instance->CR1 = 1;
#endif

	if(!useIndex)
		setPos(0);

	registerCommands();
}

#ifdef LOCALENCODER_MULTI
static void encEnableGpioClock(GPIO_TypeDef* port){
	// STM32F4: GPIO ports are 0x400 apart and their clock enable bits are consecutive from GPIOA
	RCC->AHB1ENR |= 1U << (((uint32_t)port - GPIOA_BASE) / 0x400U);
	(void)RCC->AHB1ENR;
}

static void encEnableTimerClock(TIM_TypeDef* tim){
	if(tim == TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
	else if(tim == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
	else if(tim == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
	else if(tim == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
	else if(tim == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
	else if(tim == TIM8) __HAL_RCC_TIM8_CLK_ENABLE();
}

/**
 * Configures the pins and the timer of this instance in encoder mode (count both edges of A and B).
 * Done with registers so the CubeMX MSP callbacks can't reassign pins of other instances
 */
void EncoderLocal::initMultiTimer(){
	const EncoderLocalConfig& c = configs[instance];
	this->htim = c.htim;
	c.htim->Instance = c.tim; // Used by HAL_TIM_IRQHandler and the overflow callback

	encEnableTimerClock(c.tim);
	encEnableGpioClock(c.aPort);
	encEnableGpioClock(c.bPort);
	GPIO_InitTypeDef init = {0};
	init.Mode = GPIO_MODE_AF_PP;
	init.Pull = GPIO_PULLUP;
	init.Speed = GPIO_SPEED_FREQ_LOW;
	init.Alternate = c.af;
	init.Pin = c.aPin;
	HAL_GPIO_Init(c.aPort, &init);
	init.Pin = c.bPin;
	HAL_GPIO_Init(c.bPort, &init);

	TIM_TypeDef* t = c.tim;
	t->CR1 = 0;
	t->DIER = 0;
	t->PSC = 0;
	t->ARR = 0xffff; // 16 bit counting on all timers. Overflows are added to pos
	t->CCER = 0; // Inputs not inverted
	const uint32_t f = c.filter & 0xF;
	t->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | (f << TIM_CCMR1_IC1F_Pos) | (f << TIM_CCMR1_IC2F_Pos); // TI1, TI2 inputs
	t->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // Encoder mode 3: count on TI1 and TI2 edges
	t->EGR = TIM_EGR_UG;
	t->SR = 0;
	t->CNT = 0x7fff;
	pos = 0;
	t->DIER = TIM_DIER_UIE;
	if(!NVIC_GetEnableIRQ(c.irq)){ // Keep the priority if the vector is already used (shared vectors)
		HAL_NVIC_SetPriority(c.irq, 7, 0);
	}
	HAL_NVIC_EnableIRQ(c.irq);
	t->CR1 = TIM_CR1_CEN;

	if(c.zPort){
		encEnableGpioClock(c.zPort);
		init.Pin = c.zPin;
		init.Mode = c.zMode;
		init.Pull = GPIO_PULLUP;
		init.Alternate = 0;
		HAL_GPIO_Init(c.zPort, &init);
		if(!NVIC_GetEnableIRQ(c.zIrq)){ // EXTI9_5 is shared with the e-stop. Keep its priority
			HAL_NVIC_SetPriority(c.zIrq, 6, 0);
		}
		HAL_NVIC_EnableIRQ(c.zIrq);
	}
}
#endif

/**
 * True if the EXTI pin number belongs to this instance's index pin
 */
bool EncoderLocal::isIndexPin(uint16_t GPIO_Pin){
#ifdef LOCALENCODER_MULTI
	const EncoderLocalConfig& c = configs[instance];
	if(!c.zPort || GPIO_Pin != c.zPin){
		return false;
	}
	// Several ports share an EXTI line number. Check the line is mapped to our port
	const uint32_t line = POSITION_VAL(GPIO_Pin);
	const uint32_t port = ((SYSCFG->EXTICR[line >> 2]) >> (4 * (line & 0x3))) & 0xF;
	return port == ((uint32_t)c.zPort - GPIOA_BASE) / 0x400U;
#else
	return GPIO_Pin == ENCODER_Z_Pin;
#endif
}

void EncoderLocal::registerCommands(){
	CommandHandler::registerCommands();
	registerCommand("cpr", EncoderLocal_commands::cpr, "CPR of encoder",CMDFLAG_GET|CMDFLAG_SET);
	registerCommand("index", EncoderLocal_commands::useindex, "Enable index pin",CMDFLAG_GET|CMDFLAG_SET);
}

EncoderLocal::~EncoderLocal() {
	EncoderLocal::inUse[instance] = false;
	this->htim->Instance->CR1 = 0;
#ifdef LOCALENCODER_MULTI
	this->htim->Instance->DIER = 0;
#endif
}


void EncoderLocal::saveFlash(){
	Flash_Write(encLocalAddr[instance].first, this->getCpr());
	Flash_Write(encLocalAddr[instance].second, useIndex ? (int16_t)this->offset : 0); // 0 offset indicates index off

}

void EncoderLocal::restoreFlash(){
	uint16_t cpr = 0;
	if (Flash_Read(encLocalAddr[instance].first, &cpr)){
		this->setCpr(cpr);
	}
	uint16_t ofs = 0;
	if(Flash_Read(encLocalAddr[instance].second, &ofs)){
		this->offset = (int16_t)ofs;
		useIndex = ofs != 0;
	}

}

int32_t EncoderLocal::getTimerCount(){
	return (int32_t)htim->Instance->CNT - (int32_t)0x7fff;
}

EncoderType EncoderLocal::getEncoderType(){
	return EncoderType::incremental;
}


int32_t EncoderLocal::getPos(){
	return getTimerCount() + pos;
}

void EncoderLocal::setPos(int32_t pos){
	int32_t cnt = getTimerCount();

	this->pos = pos - cnt;
	if(useIndex && indexHit){
		int32_t idxpos = ((int32_t)indexpos - (int32_t)0x7fff);
		offset = ( ( idxpos-cnt-pos ) % this->cpr); // offset is distance to index position (0 = 0x7fff index)
	}
	//this->pos = pos - cnt;
	//htim->Instance->CNT = 0x7fff; // Reset //pos+0x7fff;


}


void EncoderLocal::setPeriod(uint32_t period){
	this->htim->Instance->ARR = period-1;
}

void EncoderLocal::exti(uint16_t GPIO_Pin){
	if(isIndexPin(GPIO_Pin)){
#ifdef LOCALENCODER_MULTI
		const GPIO_PinState zState = HAL_GPIO_ReadPin(configs[instance].zPort, configs[instance].zPin);
#else
		const GPIO_PinState zState = HAL_GPIO_ReadPin(ENCODER_Z_GPIO_Port, ENCODER_Z_Pin);
#endif
		if(zState == GPIO_PIN_RESET){
		// Encoder Z pin activated

			if(useIndex && !indexHit){
				pulseSysLed();
				htim->Instance->CNT = 0x7fff; // Center encoder
				this->pos = offset;
			}
			indexpos = htim->Instance->CNT; // Store current encoder count
			indexHit = true;
		}

	}
}

void EncoderLocal::timerElapsed(TIM_HandleTypeDef* htim){
	if(htim == this->htim){
		overflowCallback();
	}
}

void EncoderLocal::overflowCallback(){
	if(htim->Instance->CNT > this->htim->Instance->ARR/2){
		pos -= htim->Instance->ARR+1;
	}else{
		pos += htim->Instance->ARR+1;
	}
}

void EncoderLocal::setCpr(uint32_t cpr){
	this->cpr = cpr;
}

CommandStatus EncoderLocal::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){
	switch(static_cast<EncoderLocal_commands>(cmd.cmdId)){
	case EncoderLocal_commands::cpr:
		handleGetFuncSetFunc(cmd, replies, &EncoderLocal::getCpr,&EncoderLocal::setCpr,this);
	break;
	case EncoderLocal_commands::useindex:
		return handleGetSet(cmd, replies, useIndex);
	break;
	default:
		return CommandStatus::NOT_FOUND;
	}

	return CommandStatus::OK;
}
#endif
