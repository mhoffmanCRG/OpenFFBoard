# F407VG_DISCO firmware on the TDD Micro AASD-SERVO MMOS v2.0: pin map

This lists every MCU pin the F407VG_DISCO firmware uses, where it sits in the DIY-MORE STM32F407VGT6 module socket, and what it connects to on the MMOS v2.0 interface board.

Sources:
- Firmware: [OpenFFBoard_F407VG_DISCO.ioc](../../Firmware/Targets/F407VG_DISCO/OpenFFBoard_F407VG_DISCO.ioc), `main.h`, and `HX711_PINS` in `target_constants.h`.
- Board: the reconstructed schematic and the 1:1 layout in this folder ([layout_1to1/README.md](layout_1to1/README.md)). The board connections are reverse-engineered, so check them before relying on them.

**Module pin** is the pin's position in the module socket on the MMOS board:
- **L1–L36** is the left 2×18 header, **R1–R32** the right 2×16 header, and **B1–B24** the bottom 2×12 header.
- Odd numbers are the outer row and even numbers the inner row.
- This numbering comes from the reconstructed layout (footprint J1). It is not printed on either board.

## Motor and servo (DB25 to the AASD)
| MCU | Module pin | Firmware | MMOS board |
|---|---|---|---|
| PE9 | L29 | PWM1 (TIM1 CH1) = torque PWM | DAC U1 IN_B via right "B" jumper → DB25-25 Vref |
| PE11 | B2 | PWM2 / DIR (PWM+DIR mode) | DB25-7 (SigIn2 / CINV) + right "A" jumper → DAC IN_A |
| PA0 | L11 | ENCODER_A (TIM2 CH1) | DB25-18 (silkscreen "ENC A") |
| PA1 | L14 | ENCODER_B (TIM2 CH2) | DB25-20 (copper-verified) |
| PA2 | L13 | ENCODER_Z (interrupt) | DB25-15 (index) |
| PD5 | R14 | E_STOP (pull-up, active low) | ⚠ Not wired. The board's E-STOP terminal goes to DB25-6 (SON), not this pin |
| PE7 | L27 | DRV_ENABLE | No connection found on the board |
| PE10 | L32 | DRV_BRAKE | No connection found on the board |
| PE8 | L30 | FLAG (interrupt) | No connection found on the board |
| PE12 | B1 | DRV_GP1 | No connection found on the board |
| PE13 / PE14 | B4 / B3 | PWM3 / PWM4 | Not used in PWM+DIR mode |
| PA4 | L18 | SPI1_SS1 | Also the "PA4" torque-select jumper → DB25-25 |
| PA5 | L17 | SPI1_SCK | Also the "PA5" torque-select jumper → DB25-25 |

## Pedals, shifter and analog inputs
| MCU | Module pin | Firmware | MMOS board |
|---|---|---|---|
| PB0 | L24 | AIN0 (ADC ch 8) | Pedal DB9 pin 3, BRAKE |
| PB1 | L23 | AIN1 (ADC ch 9) | Pedal DB9 pin 2, GAS |
| PC4 | L22 | AIN2 (ADC ch 14) | Pedal DB9 pin 4, CLUTCH |
| PC5 | L21 | ADC_VINT (internal voltage sense) | ⚠ The board uses it for the Z header (J5) |
| PA3 | L15 | ADC_VEXT (external voltage sense) | Not used by the board |
| PC2 | L12 | Not configured | Shifter DB9 pin 4, X (⚠ no ADC set up in this build) |
| PC1 | L9 | Not configured | Shifter DB9 pin 8, Y (⚠ no ADC set up in this build) |
| PA5 / PA6 / PA4 | L17 / L20 / L18 | SPI1 SCK / MISO / CS | Shifter DB9 pins 1 / 2 / 3 |
| PA7 | L19 | SPI1_MOSI | Not routed to the shifter |

## HX711 load cells
The firmware sets these pins when it starts, from `HX711_PINS` in `target_constants.h`. All three chips share one clock pin, so they must use the same channel/gain.

| MCU | Module pin | Firmware | MMOS board |
|---|---|---|---|
| PC7 | B23 | HX711 SCK (all chips) | Button header J7 **B8** |
| PC6 | B22 | HX711 chip 0 DOUT | Button header J7 **B7** |
| PC8 | B24 | HX711 chip 1 DOUT | Button header J7 **B9** |
| PC9 | R29 | HX711 chip 2 DOUT | Button header J7 **B10** |

## Buttons
The firmware's 8 local buttons (DIN0–7) only partly line up with the board's 14-button header (J7, labelled B1–B14).

| MCU | Module pin | Firmware | MMOS J7 |
|---|---|---|---|
| PE2 | L2 | DIN7 (button 8) | B3 ✓ |
| PE3 | L1 | DIN6 (button 7) | B4 ✓ |
| PE4 | L4 | DIN5 (button 6) | B5 ✓ |
| PE5 | L3 | DIN4 (button 5) | B6 ✓ |
| PE6 | L6 | DIN3 (button 4) | Not on J7 |
| PC13 | L5 | DIN2 (button 3) | B14 ✓ |
| PC14 / PC15 | Not on the module headers | DIN1 / DIN0 | Unreachable |
| PE0 / PE1 | R6 / L28 | Not configured | B1 / B2 ✗ |
| PC6–PC9 | See the HX711 table | HX711 | B7–B10 (taken by the load cells) |
| PC10–PC12 | R22 / R19 / R20 | SPI3 SCK / MISO / MOSI | B11–B13 ✗ |
| PB2 | L31 | BUTTON_A | Not on J7. BOOT1 on the module, which holds it high |

## LEDs
| MCU | Module pin | Firmware | MMOS board |
|---|---|---|---|
| PD12 | B18 | LED_PWR | D1 green |
| PD13 | B19 | LED_SYS | No LED on the board |
| PD14 | B20 | LED_ERR | D3 red |
| PD15 | B21 | LED_CLIP | D2 blue |

## Communication buses and system pins
| MCU | Module pin | Firmware | MMOS board |
|---|---|---|---|
| PA11 / PA12 | R25 / R26 | USB D− / D+ | USB, via the module |
| PA9 | R27 | USB VBUS (sensing disabled in this build) | Wired to 5V on this unit |
| PA13 / PA14 | R23 / R24 | SWDIO / SWCLK | J6 SWD header |
| PB3 | R9 | SWO / GP4 | — |
| PH0 / PH1 | — | HSE 8 MHz crystal | Crystal on the module |
| PB12 | B10 | SPI2_NSS | Bottom-side "SPI2" pads |
| PB13 / PB14 / PB15 | B11 / B12 / B13 | SPI2 SCK / MISO / MOSI | Bottom-side "SPI2" pads |
| PD8 / PD9 | B14 / B15 | SPI2_SS2 / SS3 | — |
| PA15 | R21 | SPI3_SS1 | — |
| PD2 / PD3 | R15 / R16 | SPI3_SS2 / SS3 | — |
| PC10 / PC11 / PC12 | R22 / R19 / R20 | SPI3 SCK / MISO / MOSI | J7 B11–B13 |
| PD0 / PD1 | R17 / R18 | CAN1 RX / TX | — |
| PD4 | R13 | CAN_S | — |
| PB10 / PB11 | B5 / B8 | USART3 TX / RX (GP7 / GP8) | — |
| PB6 / PB7 | R8 / R5 | USART1 TX / RX | — |
| PB8 / PB9 | L26 / L25 | I2C1 SCL / SDA | — |
| PD6 | R11 | GP1 input | — |
| PB4 | R10 | GP2 input | — |
| PB5 | R7 | GP3 input | — |

## Not used by the firmware
These pins are on the module headers but not set up by the firmware: PC0 (L10), PC3 (L16), PA8 (R30), PA10 (R28), PD7 (R12), PD10 (B16), PD11 (B17), PE15 (B6), PE0 (R6), PE1 (L28), PC1 (L9) and PC2 (L12).

## Mismatches between the firmware and the board
- Only 5 of the 14 button-header inputs work as buttons: B3–B6 and B14.
- The shifter's X/Y inputs (PC2/PC1) aren't read, because this build sets up no ADC channel for them.
- The board's E-STOP terminal switches the servo's SON input (DB25-6) directly. The firmware's E-stop pin (PD5) isn't connected.
- The Z header pin (PC5) is used by the firmware as a voltage sense (ADC_VINT).
