## F407VG_AASD4 target: FFB wheel + 3 command axes on 4 AASD servos

This target is for a custom base board with a DIY-MORE (or any) STM32F407VGT6 module, 8 MHz HSE, driving
**4 AASD servo drives** in analog torque mode. It is based on the F407VG_DISCO target.

Build: `make MCU_TARGET=F407VG_AASD4 BUILD_DIR=build_aasd4`

### What it does
- Main class **"FFB Wheel + command axes"** (id 6, default). Axis 1 is a normal FFB wheel: games drive it
  through the standard USB force feedback descriptor (the same as the FFB Wheel main class).
- Axes 2-4 are **command axes**. Game effects do not reach them. They use their own axis effects
  (`axis.N.idlespring`, `axisdamper`, `axisfriction`, `axisinertia`, endstop) plus a torque that PC software
  sends with `axis.N.hosttorque` (-32767 to 32767). The host torque is dropped after `axis.N.hosttimeout`
  ms (default 100, 0 = never) without a new value, so a crashed program can't leave torque applied.
  Send it at least every 50 ms.
- Positions of all 4 axes are reported as HID axes X, Y, Z and Rx. Analog inputs use the remaining
  axes (Ry, Rz, Dial, Slider).
- `hosttorque` also works on axis 1 and is added to the game effects there.

### Pins
| Axis | PWM (torque)    | DIR  | Encoder A / B     | Index |
|------|-----------------|------|-------------------|-------|
| 1    | PE9  TIM1_CH1   | PE11 | PA0 / PA1 (TIM5)  | PA2   |
| 2    | PE13 TIM1_CH3   | PE15 | PB4 / PB5 (TIM3)  | PC0   |
| 3    | PE14 TIM1_CH4   | PD10 | PC6 / PC7 (TIM8)  | PC3   |
| 4    | PE10 TIM1_CH2N  | PD11 | PA5 / PB3 (TIM2)  | PD7   |

- Axis 1 uses the same pins as the F407VG_DISCO / MMOS v2.0 setup.
- All PWM outputs are on TIM1 and **share one frequency**. Set it with `pwmdrv.0.freq`. Use 3 kHz
  (`freq=0`) if axis 1 goes through an LTC2644 PWM-to-voltage DAC (max 6.25 kHz input). With RC filters
  on every axis a higher frequency gives less ripple.
- Each axis uses the PWM driver in **PWM/DIR mode**: duty = |torque| from the zero-torque duty
  (`pwmdrv.N.minduty`, default 1%, or the auto zero result) to 100%, DIR high = positive.
  Centered PWM is also available. RC PPM and dual PWM are not, because they would need their own timer.
- Settings per axis: `pwmdrv.N.*` (mode, invert, minduty, autozero) and `localenc.N.*` (cpr, index),
  where N = axis - 1.
- HX711 load cells moved to SCK PE0, DOUT PE1 / PC8 / PC9 because PC6/PC7 are the axis 3 encoder.
- Not available on this target: SPI1 (PA5 is an encoder input) and SWO trace (PB3).
- E-stop (PD5) stops all axes.

### Analog torque input of the AASD drives
The AASD analog torque input (Vref, DB25 pin 25 on the AASD-30A) needs a DC voltage, so each PWM output needs
a filter or a PWM-to-voltage DAC. A 2 stage RC low pass with its corner around 300 Hz is a good start. Zero the
drive offset (Pn190) at the zero-torque duty and set the torque gain (Pn189) for the 0-3.3 V range.
Wire each DIR output to the drive's direction/CINV input (SigIn2, DB25 pin 7, Pn053=18).

### Notes for developers
- PWM and encoder pins are configured at runtime from `MotorPWM::multiConfigs` and `EncoderLocal::configs`
  in `Core/Src/cpp_target_config.cpp` (`PWMDRIVER_MULTI`, `LOCALENCODER_MULTI` in `target_constants.h`).
  The CubeMX project (`OpenFFBoard_F407VG_AASD4.ioc`) is still the DISCO one. CubeMX-generated init for those
  pins runs first and is then overridden.
- Encoder overflow interrupts: TIM3_IRQHandler, the TIM8 part of TIM8_UP_TIM13_IRQHandler, EXTI0/EXTI3 handlers
  and EXTI line 7 in EXTI9_5 were added in the USER CODE sections of `Core/Src/stm32f4xx_it.c`.
- Settings use their own flash addresses: AXIS4 block 0x421-0x42C, PWM instances 0x221-0x223,
  local encoder instances 0x212-0x217.
