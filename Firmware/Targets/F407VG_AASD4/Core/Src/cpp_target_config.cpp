#include "cpp_target_config.h"
#include <array>
#include "MotorPWM.h"
#include "EncoderLocal.h"

extern SPI_HandleTypeDef hspi2;

static const std::vector<OutputPin> external_spi_cspins{OutputPin(*SPI2_NSS_GPIO_Port, SPI2_NSS_Pin), OutputPin(*SPI2_SS2_GPIO_Port, SPI2_SS2_Pin),OutputPin(*SPI2_SS3_GPIO_Port, SPI2_SS3_Pin)};
SPIPort external_spi{hspi2,external_spi_cspins,42000000,true};


//static const std::vector<OutputPin> motor_spi_cspins{OutputPin(*SPI1_SS1_GPIO_Port, SPI1_SS1_Pin), OutputPin(*SPI1_SS2_GPIO_Port, SPI1_SS2_Pin),OutputPin(*SPI1_SS3_GPIO_Port, SPI1_SS3_Pin)};
//extern SPI_HandleTypeDef hspi1;
//SPIPort motor_spi{hspi1,motor_spi_cspins,84000000,false};

#ifdef EXT3_SPI_PORT
static const std::vector<OutputPin> ext3_spi_cspins{OutputPin(*SPI3_SS1_GPIO_Port, SPI3_SS1_Pin), OutputPin(*SPI3_SS2_GPIO_Port, SPI3_SS2_Pin),OutputPin(*SPI3_SS3_GPIO_Port, SPI3_SS3_Pin)};
extern SPI_HandleTypeDef EXT3_SPI_PORT;
SPIPort ext3_spi{hspi3,ext3_spi_cspins,42000000,true};
#endif

#ifdef UART_PORT_MOTOR
extern UART_HandleTypeDef UART_PORT_MOTOR;
UARTPort motor_uart{UART_PORT_MOTOR};
#endif

#ifdef UART_PORT_EXT
extern UART_HandleTypeDef UART_PORT_EXT;
UARTPort external_uart{UART_PORT_EXT};
#endif

#ifdef I2C_PORT
const I2C_InitTypeDef i2cPreset_10000 = {100000,I2C_DUTYCYCLE_2,0,I2C_ADDRESSINGMODE_7BIT,I2C_DUALADDRESS_DISABLE,0,I2C_GENERALCALL_DISABLE,I2C_NOSTRETCH_DISABLE};
const I2C_InitTypeDef i2cPreset_40000 = {400000,I2C_DUTYCYCLE_2,0,I2C_ADDRESSINGMODE_7BIT,I2C_DUALADDRESS_DISABLE,0,I2C_GENERALCALL_DISABLE,I2C_NOSTRETCH_DISABLE};
const auto i2cpresets = std::to_array<I2CPortHardwareConfig::I2CPortHardwareConfig_preset>({{i2cPreset_10000,"100kb/s"},{i2cPreset_40000,"400kb/s"}});
const I2CPortHardwareConfig i2cHwConf = I2CPortHardwareConfig(true, i2cpresets);

extern I2C_HandleTypeDef I2C_PORT;
I2CPort i2cport{I2C_PORT,i2cHwConf};
#endif

#ifdef GPIO_MOTOR
const OutputPin gpMotor{*DRV_GP1_GPIO_Port,DRV_GP1_Pin};
#endif


#ifdef CANBUS
/*
 * Can BTR register for different speed configs
 * 50, 100, 125, 250, 500, 1000 kbit
 */
#include "CANPort2B.h"
const auto canpresetentries = std::to_array<CANPortHardwareConfig::PresetEntry>({{0x001b0037,50000,"50k"},{0x001b001b,100000,"100k"},{0x001c0014,125000,"125k"},{0x001a000b,250000,"250k"},{0x001a0005,500000,"500k"},{0x001a0002,1000000,"1000k"}});
CANPortHardwareConfig canpresets = CANPortHardwareConfig(true,canpresetentries);
const OutputPin canSilentPin = OutputPin(*CAN_S_GPIO_Port, CAN_S_Pin);
CANPort_2B canport_base{CANPORT,canpresets,&canSilentPin};
CANPort& canport = static_cast<CANPort&>(canport_base);
#endif

const OutputPin debugpin = OutputPin(*GP1_GPIO_Port, GP1_Pin);

#ifdef PWMDRIVER
/*
 * One PWM/DIR output per axis. All on TIM1 (168MHz), so they share the PWM frequency.
 * channel_1 = HAL channel of the instance, pwm_chan/centerpwm_chan = its CCR number.
 * The DIR pins are GPIOs. Axis 1 keeps PE9/PE11 like the DISCO target (PE11 was TIM1_CH2 there, same levels).
 * Axis 4 uses the complementary output TIM1_CH2N on PE10 (CH2 itself has no pin because PE11 is a GPIO).
 */
#define PWM_AXIS_CONFIG(CHAN, CCR, PWMPORT, PWMPIN, COMPL, DIRPORT, DIRPIN) \
{ \
	.channel_1 = CHAN, .channel_2 = CHAN, .channel_3 = CHAN, .channel_4 = CHAN, \
	.pwm_chan = CCR, .dir_chan = 0, .dir_chan_n = 0, \
	.centerpwm_chan = CCR, .rcpwm_chan = CCR, .dualpwm1 = CCR, .dualpwm2 = CCR, \
	.timer = &TIM_PWM, .timerFreq = 168000000, \
	.dirPort = DIRPORT, .dirPin = DIRPIN, \
	.pwmPort = PWMPORT, .pwmPin = PWMPIN, .pwmAf = GPIO_AF1_TIM1, .pwmComplementary = COMPL \
}

const std::array<PWMConfig,PWMDRIVER_MULTI> MotorPWM::multiConfigs = {{
	PWM_AXIS_CONFIG(TIM_CHANNEL_1, 1, GPIOE, GPIO_PIN_9,  false, GPIOE, GPIO_PIN_11), // Axis 1: PWM PE9,  DIR PE11
	PWM_AXIS_CONFIG(TIM_CHANNEL_3, 3, GPIOE, GPIO_PIN_13, false, GPIOE, GPIO_PIN_15), // Axis 2: PWM PE13, DIR PE15
	PWM_AXIS_CONFIG(TIM_CHANNEL_4, 4, GPIOE, GPIO_PIN_14, false, GPIOD, GPIO_PIN_10), // Axis 3: PWM PE14, DIR PD10
	PWM_AXIS_CONFIG(TIM_CHANNEL_2, 2, GPIOE, GPIO_PIN_10, true,  GPIOD, GPIO_PIN_11), // Axis 4: PWM PE10 (CH2N), DIR PD11
}};
#endif

#ifdef LOCALENCODER
/*
 * One quadrature encoder per axis, configured at runtime.
 * Axis 1 keeps PA0/PA1/PA2 but is counted by TIM5 (PA0/PA1 have TIM2 and TIM5) so TIM2 is free for axis 4.
 * The IRQ handlers of these timers call HAL_TIM_IRQHandler with the handles below (stm32f4xx_it.c).
 */
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

const std::array<EncoderLocalConfig,LOCALENCODER_MULTI> EncoderLocal::configs = {{
	// Axis 1: A PA0, B PA1, index PA2
	{.tim = TIM5, .htim = &htim5, .irq = TIM5_IRQn, .aPort = GPIOA, .aPin = GPIO_PIN_0, .bPort = GPIOA, .bPin = GPIO_PIN_1, .af = GPIO_AF2_TIM5,
	 .zPort = GPIOA, .zPin = GPIO_PIN_2, .zIrq = EXTI2_IRQn},
	// Axis 2: A PB4, B PB5, index PC0
	{.tim = TIM3, .htim = &htim3, .irq = TIM3_IRQn, .aPort = GPIOB, .aPin = GPIO_PIN_4, .bPort = GPIOB, .bPin = GPIO_PIN_5, .af = GPIO_AF2_TIM3,
	 .zPort = GPIOC, .zPin = GPIO_PIN_0, .zIrq = EXTI0_IRQn},
	// Axis 3: A PC6, B PC7, index PC3
	{.tim = TIM8, .htim = &htim8, .irq = TIM8_UP_TIM13_IRQn, .aPort = GPIOC, .aPin = GPIO_PIN_6, .bPort = GPIOC, .bPin = GPIO_PIN_7, .af = GPIO_AF3_TIM8,
	 .zPort = GPIOC, .zPin = GPIO_PIN_3, .zIrq = EXTI3_IRQn},
	// Axis 4: A PA5, B PB3, index PD7
	{.tim = TIM2, .htim = &htim2, .irq = TIM2_IRQn, .aPort = GPIOA, .aPin = GPIO_PIN_5, .bPort = GPIOB, .bPin = GPIO_PIN_3, .af = GPIO_AF1_TIM2,
	 .zPort = GPIOD, .zPin = GPIO_PIN_7, .zIrq = EXTI9_5_IRQn},
}};
#endif
