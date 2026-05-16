# STM32F103 PSFB Firmware

Register-level CMSIS firmware for STM32F103 full-bridge / PSFB testing with TIM1 complementary outputs and hardware dead-time. The PlatformIO target is `genericSTM32F103C6`.

## Current Defaults

```c
#define CONTROL_MODE CONTROL_MODE_OPEN_LOOP_DUTY
#define FSW_HZ 18000u
#define DEADTIME_NS 500u
#define TEST_UNLOCK_HIGH_DUTY 1
#define DUTY_MAX_INITIAL_PERCENT 20.0f
```

Feedback is disabled by default. PA0 may be left disconnected in open-loop mode.

Startup settings are at the top of [src/main.c](src/main.c):

```c
#define USER_ENABLE_FEEDBACK_CONTROL      0u
#define USER_TARGET_OUTPUT_VOLTAGE        24.0f
#define USER_OPEN_LOOP_COMMAND_PERCENT    85.0f
#define USER_EST_INPUT_VOLTAGE            90.0f
#define USER_EST_OUTPUT_VOLTAGE           12.0f
```

Keep `USER_ENABLE_FEEDBACK_CONTROL = 0u` to keep the current open-loop/manual behavior.

Only PSFB phase-shift modulation is present in this build. The old basic balanced PWM mode has been removed.

## Pin Mapping

| Function | Pin | Peripheral |
|---|---:|---|
| Q1 high-side left leg | PA8 | TIM1_CH1 |
| Q2 low-side left leg | PB13 | TIM1_CH1N |
| Q3 high-side right leg | PA9 | TIM1_CH2 |
| Q4 low-side right leg | PB14 | TIM1_CH2N |
| VOUT feedback | PA0 | ADC1_IN0 |
| Future current fault | PB12 | TIM1_BKIN |

Every gate-driver input must have an external pull-down resistor.

## Feedback Divider

The feedback divider is 100 kOhm from VOUT+ to PA0 sense node and 10 kOhm from PA0 sense node to feedback reference.

```text
ADC voltage = VOUT * 10k / (100k + 10k)
ADC voltage = VOUT / 11
VOUT = ADC_raw_filtered * 3.3 / 4095 * 11
```

Code macros:

```c
#define ADC_REF_VOLTAGE              3.3f
#define ADC_FULL_SCALE               4095.0f
#define VOUT_DIVIDER_TOP_OHM         100000.0f
#define VOUT_DIVIDER_BOTTOM_OHM      10000.0f
#define VOUT_FEEDBACK_SCALE          ((VOUT_DIVIDER_TOP_OHM + VOUT_DIVIDER_BOTTOM_OHM) / VOUT_DIVIDER_BOTTOM_OHM)
#define ADC_OVERSAMPLE_COUNT         16u
#define ADC_IIR_ALPHA                0.03125f
```

Maximum measurable output voltage is about `3.3 V * 11 = 36.3 V`. Software latches `FAULT_ADC_NEAR_FULL_SCALE` if `adc_raw > 3900`.

ADC filtering averages 16 conversions per control update, then applies an IIR low-pass filter. The filter initializes from the first averaged sample so closed-loop feedback does not start from an artificial zero reading.

Important: feedback must be isolated if the STM32 control ground is on the primary side. Do not directly connect a secondary-side output divider to a primary-side MCU unless isolation and grounding are intentionally handled.

## Control Modes

```text
1 = CONTROL_MODE_OPEN_LOOP_DUTY
2 = CONTROL_MODE_OPEN_LOOP_EST_VOLT
3 = CONTROL_MODE_CLOSED_LOOP_FEEDBACK
```

Open-loop duty/phase mode:
- Feedback disabled.
- PA0 is not required.
- `USER_OPEN_LOOP_COMMAND_PERCENT` sets manual duty/phase percent.
- Initial hard limit is 20% unless `TEST_UNLOCK_HIGH_DUTY = 1`.

Open-loop estimated voltage mode:
- Feedback disabled.
- Uses estimated input voltage, transformer ratio, and diode drop.
- This is only estimation, not regulation.

Closed-loop feedback mode:
- Enabled by `USER_ENABLE_FEEDBACK_CONTROL = 1u`.
- Uses PA0 ADC through the 100k/10k divider.
- `USER_TARGET_OUTPUT_VOLTAGE` is clamped to 12 V through 28 V.
- PI loop runs at 1 kHz.
- Not limited by the 20% initial open-loop limit; feedback can command up to `DUTY_MAX_ABSOLUTE_PERCENT`.
- Includes soft-start, slew limit, anti-windup, collapse handling, ADC high fault, and OVP.

## Closed-Loop Protection

```c
#define TARGET_VOUT_DEFAULT          24.0f
#define TARGET_VOUT_MIN              12.0f
#define TARGET_VOUT_MAX              28.0f
#define CONTROL_LOOP_HZ              500u
#define SOFTSTART_TIME_MS            3000u
#define DUTY_SLEW_UP_PERCENT_PER_MS  0.5f
#define OVP_MULTIPLIER               1.25f
#define VOUT_COLLAPSE_RATIO          0.70f
#define DUTY_SAFE_LIMIT_PERCENT      15.0f
#define FEEDBACK_KP                  0.03f
#define FEEDBACK_KI                  0.004f
#define FEEDBACK_LOW_BLANKING_MS     1500u
#define ADC_PREFILL_SAMPLES          64u
```

The feedback loop is intentionally slower and conservative because there is no output LC filter yet. Feedback-low/disconnected detection is blanked for the first 1.5 seconds after feedback starts, and ADC filtering is prefilled before PWM starts.

For a 24 V target:

```text
OVP = 24 * 1.25 = 30 V
```

If OVP occurs, TIM1 `MOE` is cleared immediately and the fault is latched. Reset the MCU to recover.

Feedback is not a substitute for primary current sensing. Hardware current protection through TIM1_BKIN should be added before serious high-power testing.

## Timer And Dead-Time

For 18 kHz with 72 MHz TIM1 clock:

```text
ARR = 72000000 / (2 * 18000) - 1 = 1999
PSC = 0
```

Dead-time:

```text
Timer tick = 1 / 72 MHz = 13.888 ns
500 ns / 13.888 ns ~= 36 ticks
```

Verify actual dead-time at the MOSFET gate-source pins with an oscilloscope.

## Safe Bring-Up

1. Start with feedback disabled:
   ```c
   #define USER_ENABLE_FEEDBACK_CONTROL      0u
   #define USER_OPEN_LOOP_COMMAND_PERCENT    5.0f
   ```
2. Keep `TEST_UNLOCK_HIGH_DUTY = 0`.
3. Use low DC input with current limit.
4. Verify PA8/PB13 and PA9/PB14 complementary switching with dead-time.
5. Verify transformer primary has equal positive and negative volt-seconds.
6. Increase manual command slowly up to the 20% initial limit.
7. Only enable feedback after divider scaling and isolation are verified:
   ```c
   #define USER_ENABLE_FEEDBACK_CONTROL      1u
   #define USER_TARGET_OUTPUT_VOLTAGE        24.0f
   ```
8. Increase target voltage gradually and monitor VDS ringing, transformer current, and temperatures.

Do not use feedback control as the only safety mechanism. There is no primary current sensing yet.
