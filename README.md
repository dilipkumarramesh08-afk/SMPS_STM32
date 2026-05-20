# STM32F103C6T6 H-Bridge PWM Firmware

This firmware is a normal bipolar H-bridge transformer driver. It is not PSFB.

Target MCU: STM32F103C6T6 at 72 MHz from an 8 MHz external HSE.

```text
8 MHz HSE * PLL 9 = 72 MHz SYSCLK
APB1 = 36 MHz
APB2 = 72 MHz
TIM1 = 72 MHz
ADC clock = PCLK2 / 6 = 12 MHz
```

## Hardware

- Q1 left high-side: PA8 / TIM1_CH1
- Q2 left low-side: PB13 / TIM1_CH1N
- Q3 right high-side: PA9 / TIM1_CH2
- Q4 right low-side: PB14 / TIM1_CH2N
- Output feedback: PA0 / ADC1_IN0
- Fault LED: PC13
- No input-voltage sensing
- No current sensing
- No hardware fault/BKIN input
- No UART menu

Do not run high power until current sensing and hardware shutdown are added.

## Waveform

TIM1 generates one center-aligned bipolar switching period in hardware:

```text
Q1 + Q4 ON -> +VIN across transformer primary
Q2 + Q4 ON -> zero/freewheel state
Q2 + Q3 ON -> -VIN across transformer primary
Q2 + Q4 ON -> zero/freewheel state
repeat
```

Positive and negative pulse widths are always equal. Q1/Q2 and Q3/Q4 are never intentionally enabled together. TIM1 complementary outputs provide same-leg dead time in hardware.

## Active Defaults

```c
#define TARGET_VOUT_MV 26000U
#define FSW_HZ 20000UL
#define CONTROL_LOOP_HZ 8000UL
#define NORMAL_DEADTIME_NS 500UL
#define SOFTSTART_TIME_MS 3000UL
#define DUTY_MAX_PERMILLE 850U
```

The duty command is total bipolar active duty. At the 850 permille limit, each transformer polarity pulse can reach about 42.5% of the switching period.

## Feedback

PA0 uses a 100k/10k divider:

```text
VOUT = ADC * 3300 / 4095 * 11
```

Expected PA0 levels:

```text
12 V output -> about 1.09 V
14 V output -> about 1.27 V
22 V output -> about 2.00 V
24 V output -> about 2.18 V
26 V output -> about 2.36 V
28 V output -> about 2.55 V
maximum measurable output -> about 36.3 V
```

## Control

The only active operating mode is closed-loop voltage control from PA0.

- soft-start from 0 V to target over 3 seconds
- 8 kHz PI control loop from SysTick
- fractional soft-start ramp accumulator
- duty slew below 70% target: up 300 permille/sec, down 1000 permille/sec
- duty slew above 70% target: up 3000 permille/sec, down 8000 permille/sec
- fractional duty-slew accumulator keeps those rates correct at 8 kHz
- 25 mV voltage deadband
- PI anti-windup
- ADC1 continuous sampling
- DMA1_Channel1 circular ADC buffer averaging plus IIR filtering
- TIM1 center-aligned PWM generates bridge timing in hardware
- SysTick control-loop interrupt runs below TIM1 priority
- soft-start step and OVP limit are cached after target setup

TIM1 PWM usage:

```text
CH1 PWM1  -> Q1 near counter zero
CH1N      -> Q2 complementary with dead time
CH2 PWM2  -> Q3 near counter ARR
CH2N      -> Q4 complementary with dead time
```

PI terms:

```text
P = error_mv >> 5
I update = error_mv >> 9
```

Useful ST-LINK watch fields:

```c
g_hbridge.vout_mv
g_hbridge.target_ramped_mv
g_hbridge.error_mv
g_hbridge.duty_target_permille
g_hbridge.duty_actual_permille
g_hbridge.fault
```

## Protection

Fault behavior:

- clear TIM1_BDTR.MOE
- set duty to zero
- latch fault
- require reset

Fault blink codes on PC13:

```text
1 blink  = invalid target voltage
2 blinks = output over-voltage
3 blinks = ADC near full scale
4 blinks = feedback low or disconnected
5 blinks = software limit
```

OVP:

```text
OVP = TARGET_VOUT_MV * 1.5
```

For the 26 V target, OVP is 39 V. OVP must be detected on 24 consecutive 8 kHz control-loop readings before shutdown. ADC near-full-scale protection trips above raw ADC 3900.

Low-feedback protection is blanked for 4000 ms after startup. After blanking, PA0 must remain almost zero for 3000 ms before the feedback-low fault latches.

## Build

```ini
[env:genericSTM32F103C6]
board = genericSTM32F103C6
framework = cmsis
board_build.ldscript = STM32F103C6T6_FLASH.ld
```

Linker memory:

```text
FLASH = 32K
RAM   = 10K
```
