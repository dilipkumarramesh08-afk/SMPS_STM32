# STM32F103C6T6 PSFB Firmware

This firmware is PSFB-only. The old variable-duty H-bridge fallback has been removed.

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

## PSFB Waveform

Both bridge legs are fixed 50% duty. Output power is controlled only by right-leg phase shift:

```text
Q1 + Q3 ON -> zero state
Q1 + Q4 ON -> +VIN across transformer primary
Q2 + Q4 ON -> zero state
Q2 + Q3 ON -> -VIN across transformer primary
repeat
```

Positive and negative transfer intervals are always equal. Q1/Q2 and Q3/Q4 are never intentionally enabled together. TIM1 complementary outputs provide same-leg dead time in hardware.

This is hard-switched PSFB. It does not implement ZVS detection, adaptive dead-time, or current-mode control.

## Secondary Rectifier

The secondary rectifier is diode-only in this build. There is no synchronous-rectifier GPIO, timer, or DMA drive.

## Timing Config Note

Keep all coupled timing values together in [psfb_control.h](</d:/PROJECTS/DC_SMPS_STM32/DC_SMPS_STM32/include/psfb_control.h:24>) under `Power-stage timing configuration`.

When changing switching frequency or dead time, review this whole block:

```c
#define FSW_HZ
#define CONTROL_LOOP_FSW_DIVIDER
#define NORMAL_DEADTIME_NS
#define PSFB_PHASE_MAX_PERMILLE
```

`CONTROL_LOOP_HZ` is derived as `FSW_HZ / CONTROL_LOOP_FSW_DIVIDER`. The current ratio is `40 kHz / 25 = 1.6 kHz`. Do not tune the loop frequency separately unless the divider is changed intentionally.

These affect TIM1 primary timing, TIM3 ADC trigger timing, PI/slew response, and oscilloscope validation.

## Active Defaults

```c
#define TARGET_VOUT_MV 28000U
#define FSW_HZ 40000UL
#define CONTROL_LOOP_FSW_DIVIDER 25UL
#define CONTROL_LOOP_HZ 1600UL
#define NORMAL_DEADTIME_NS 800UL
#define TIM1_DEADTIME_ACTUAL_NS 806UL
#define SOFTSTART_TIME_MS 3000UL
#define PSFB_PHASE_MAX_PERMILLE 900U
#define PSFB_LAGGING_LEG_RIGHT 0U
#define ADC_DMA_SAMPLES 16U
```

`PSFB_PHASE_MAX_PERMILLE` allows up to 90% phase shift for this test build. The bridge legs remain fixed at 50% duty in PSFB mode. There is still no current sense or hardware fault input, so use current-limited input during bring-up.

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
30 V output -> about 2.73 V
maximum measurable output -> about 36.3 V
```

## Control

- closed-loop voltage control from PA0
- soft-start from 0 V to target over 3 seconds
- 1.6 kHz PI control loop from SysTick, derived from switching frequency
- PI output commands `phase_permille`
- PI gains are integer shift based: proportional `error_mv >> 4`, integral accumulator update `error_mv >> 8`
- phase slew below 70% target: up 300 permille/sec, down 1000 permille/sec
- phase slew above 70% target: up 3000 permille/sec, down 8000 permille/sec
- fractional soft-start and phase-slew accumulators
- 25 mV voltage deadband
- PI anti-windup
- ADC1 sampling triggered from TIM3_TRGO once per full switching period
- TIM3 is synchronized by TIM1 full-period TRGO, so ADC sampling remains locked to PSFB timing
- DMA1_Channel1 circular ADC buffer with half/full-transfer interrupt averaging
- ADC DMA average rejects the highest and lowest sample from each 8-sample half-buffer
- ADC trigger is placed in the longer quiet interval between PSFB switching edges and guarded away from edge transitions
- adaptive ADC IIR filter: fast shift 1 when raw delta is 8 counts or more, slow shift 3 when steady
- TIM1 output compare toggle mode generates fixed-50% PSFB legs in hardware
- TIM1 full-period TRGO synchronizes TIM3 through ITR0
- SysTick control-loop interrupt runs below TIM1 hardware timing
- soft-start step and OVP limit are cached after target setup

TIM1 PSFB usage:

```text
CH1 toggle  -> left leg Q1/Q2 fixed 50%
CH1N        -> left complementary output with dead time
CH2 toggle  -> right leg Q3/Q4 fixed 50%
CH2N        -> right complementary output with dead time
TIM3_TRGO   -> ADC trigger in zero/freewheel interval; TIM3 is internal only
```

PI terms:

```text
P = error_mv >> 5
I update = error_mv >> 9
```

Useful ST-LINK watch fields:

```c
g_psfb.vout_mv
g_psfb.target_ramped_mv
g_psfb.error_mv
g_psfb.phase_target_permille
g_psfb.phase_actual_permille
g_psfb.phase_limit_permille
g_psfb.fault
```

## Protection

Fault behavior:

- clear TIM1_BDTR.MOE
- set phase shift to zero
- latch fault
- require reset

Fault blink codes on PC13:

```text
1 blink  = invalid target voltage
2 blinks = output over-voltage
3 blinks = ADC near full scale
4 blinks = feedback low or disconnected
```

OVP:

```text
OVP = min(TARGET_VOUT_MV + 4000 mV, 34000 mV)
```

For the 28 V target, OVP is 32 V. OVP and ADC near-full-scale protection use about 3 ms of consecutive fault confirmation, derived from the current control-loop rate.

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
