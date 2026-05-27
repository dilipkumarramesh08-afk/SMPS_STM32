# STM32F103C6T6 PSFB Firmware

Minimal PSFB firmware for STM32F103C6T6 with isolated TL431 + PC817 feedback on PA0.

## Hardware

- Q1 left high-side: PA8 / TIM1_CH1
- Q2 left low-side: PB13 / TIM1_CH1N
- Q3 right high-side: PA9 / TIM1_CH2
- Q4 right low-side: PB14 / TIM1_CH2N
- Feedback: PA0 / ADC1_IN0, pulled up on primary side
- Fault LED: PC13

No input-voltage sensing, current sensing, BKIN fault input, UART, TIM2 control timer, or synchronous rectifier control is used in this build.

## Active Settings

```c
#define FSW_HZ 40000UL
#define CONTROL_LOOP_HZ (FSW_HZ / 25UL)
#define NORMAL_DEADTIME_NS 1000UL
#define PSFB_PHASE_MAX_PERMILLE 900U
#define OPTO_TARGET_FEEDBACK_RAW 2048U
#define OPTO_OUTPUT_HIGH_PULLS_LOW 1U
```

## Control

TIM1 generates true PSFB drive:

- both legs fixed near 50% duty
- Q1/Q2 complementary with hardware dead time
- Q3/Q4 complementary with hardware dead time
- output power controlled only by phase shift

Feedback polarity:

```text
output low  -> TL431/PC817 off or weak -> PA0 high -> increase phase
output high -> TL431/PC817 on          -> PA0 low  -> decrease phase
```

The code normalizes PA0 before control:

```c
feedback_normalized = 4095 - PA0_ADC;
error = OPTO_TARGET_FEEDBACK_RAW - feedback_normalized;
```

So the PI loop always sees:

```text
positive error -> increase phase
negative error -> decrease phase
```

PA0 ADC uses DMA and a trimmed average. The added capacitor on PA0 handles the short optocoupler pulses in hardware, so the firmware only needs the filtered analog feedback value.

Timing:

- TIM1 generates PSFB gate timing.
- TIM1 update interrupt runs the control loop every 25 switching periods.
- TIM3 triggers ADC sampling from TIM1-synchronized timing.
- ADC1 + DMA samples PA0 silently.
- Main loop only drives the fault LED state.

## Build

```powershell
C:\Users\pm\.platformio\penv\Scripts\platformio.exe run
```
