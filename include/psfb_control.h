#ifndef PSFB_CONTROL_H
#define PSFB_CONTROL_H

#include <stdint.h>

#define CONTROL_MODE_OPEN_LOOP_DUTY       1
#define CONTROL_MODE_OPEN_LOOP_EST_VOLT   2
#define CONTROL_MODE_CLOSED_LOOP_FEEDBACK 3
#define CONTROL_MODE_CLOSED_LOOP_VOUT     CONTROL_MODE_CLOSED_LOOP_FEEDBACK

#define CLOCK_SOURCE_HSE_8MHZ_PLL_72MHZ   1
#define CLOCK_SOURCE_HSI_PLL_64MHZ        2

#ifndef CONTROL_MODE
#define CONTROL_MODE CONTROL_MODE_OPEN_LOOP_DUTY
#endif

#ifndef FSW_HZ
#define FSW_HZ 100000u
#endif

#ifndef CLOCK_SOURCE
#define CLOCK_SOURCE CLOCK_SOURCE_HSE_8MHZ_PLL_72MHZ
#endif

#if (CLOCK_SOURCE != CLOCK_SOURCE_HSE_8MHZ_PLL_72MHZ) && \
    (CLOCK_SOURCE != CLOCK_SOURCE_HSI_PLL_64MHZ)
#error "Invalid CLOCK_SOURCE"
#endif

#ifndef SYSCLK_HZ
#if CLOCK_SOURCE == CLOCK_SOURCE_HSE_8MHZ_PLL_72MHZ
#define SYSCLK_HZ 72000000u
#else
#define SYSCLK_HZ 64000000u
#endif
#endif

#define TIM1_ARR_VALUE ((uint16_t)((SYSCLK_HZ / (2u * FSW_HZ)) - 1u))
#define TIM1_PERIOD_TICKS ((uint16_t)(TIM1_ARR_VALUE + 1u))

#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3f
#endif

#ifndef ADC_FULL_SCALE
#define ADC_FULL_SCALE 4095.0f
#endif

#ifndef VOUT_DIVIDER_TOP_OHM
#define VOUT_DIVIDER_TOP_OHM 100000.0f
#endif

#ifndef VOUT_DIVIDER_BOTTOM_OHM
#define VOUT_DIVIDER_BOTTOM_OHM 10000.0f
#endif

#ifndef VOUT_FEEDBACK_SCALE
#define VOUT_FEEDBACK_SCALE \
    ((VOUT_DIVIDER_TOP_OHM + VOUT_DIVIDER_BOTTOM_OHM) / VOUT_DIVIDER_BOTTOM_OHM)
#endif

#ifndef TARGET_VOUT_DEFAULT
#define TARGET_VOUT_DEFAULT 24.0f
#endif

#ifndef TARGET_VOUT_MIN
#define TARGET_VOUT_MIN 12.0f
#endif

#ifndef TARGET_VOUT_MAX
#define TARGET_VOUT_MAX 28.0f
#endif

#ifndef CONTROL_LOOP_HZ
#define CONTROL_LOOP_HZ 500u
#endif

#define CONTROL_LOOP_PERIOD_MS (1000u / CONTROL_LOOP_HZ)

#if (CONTROL_LOOP_HZ == 0u) || ((1000u % CONTROL_LOOP_HZ) != 0u)
#error "CONTROL_LOOP_HZ must divide 1000 exactly"
#endif

#ifndef SOFTSTART_TIME_MS
#define SOFTSTART_TIME_MS 3000u
#endif

#ifndef DUTY_MAX_INITIAL_PERCENT
#define DUTY_MAX_INITIAL_PERCENT 20.0f
#endif

#ifndef DUTY_SAFE_LIMIT_PERCENT
#define DUTY_SAFE_LIMIT_PERCENT 15.0f
#endif

#ifndef DUTY_MAX_ABSOLUTE_PERCENT
#define DUTY_MAX_ABSOLUTE_PERCENT 85.0f
#endif

#ifndef TEST_UNLOCK_HIGH_DUTY
#define TEST_UNLOCK_HIGH_DUTY 1
#endif

#ifndef DUTY_SLEW_UP_PERCENT_PER_MS
#define DUTY_SLEW_UP_PERCENT_PER_MS 0.5f
#endif

#ifndef OVP_MULTIPLIER
#define OVP_MULTIPLIER 1.25f
#endif

#ifndef VOUT_COLLAPSE_RATIO
#define VOUT_COLLAPSE_RATIO 0.70f
#endif

#ifndef ADC_NEAR_FULL_SCALE_LIMIT
#define ADC_NEAR_FULL_SCALE_LIMIT 3900u
#endif

#ifndef ADC_OVERSAMPLE_COUNT
#define ADC_OVERSAMPLE_COUNT 16u
#endif

#if ADC_OVERSAMPLE_COUNT == 0u
#error "ADC_OVERSAMPLE_COUNT must be greater than zero"
#endif

#ifndef ADC_IIR_ALPHA
#define ADC_IIR_ALPHA 0.03125f
#endif

#ifndef FEEDBACK_KP
#define FEEDBACK_KP 0.03f
#endif

#ifndef FEEDBACK_KI
#define FEEDBACK_KI 0.004f
#endif

#ifndef FEEDBACK_LOW_BLANKING_MS
#define FEEDBACK_LOW_BLANKING_MS 1500u
#endif

#ifndef ADC_PREFILL_SAMPLES
#define ADC_PREFILL_SAMPLES 64u
#endif

#ifndef MANUAL_EST_OUTPUT_VOLTAGE
#define MANUAL_EST_OUTPUT_VOLTAGE 12.0f
#endif

#ifndef MANUAL_EST_INPUT_VOLTAGE
#define MANUAL_EST_INPUT_VOLTAGE 90.0f
#endif

#ifndef PRIMARY_TURNS
#define PRIMARY_TURNS 40.0f
#endif

#ifndef SECONDARY_TURNS_TOTAL
#define SECONDARY_TURNS_TOTAL 12.0f
#endif

#ifndef RECTIFIER_DIODE_DROP_TOTAL
#define RECTIFIER_DIODE_DROP_TOTAL 1.4f
#endif

#ifndef DEADTIME_NS
#define DEADTIME_NS 500u
#endif

#ifndef ENABLE_TIM1_BKIN
#define ENABLE_TIM1_BKIN 0
#endif

#ifndef ENABLE_IWDG
#define ENABLE_IWDG 0
#endif

#ifndef INVERT_TIM1_OUTPUT_POLARITY
#define INVERT_TIM1_OUTPUT_POLARITY 0
#endif

#ifndef INVERT_TIM1_N_OUTPUT_POLARITY
#define INVERT_TIM1_N_OUTPUT_POLARITY 0
#endif

typedef enum {
    STATE_IDLE = 0,
    STATE_OPEN_LOOP_RAMP,
    STATE_SOFTSTART,
    STATE_RUN,
    STATE_FAULT
} psfb_state_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_OVERVOLTAGE,
    FAULT_ADC_NEAR_FULL_SCALE,
    FAULT_FEEDBACK_LOW_OR_DISCONNECTED,
    FAULT_INVALID_COMMAND,
    FAULT_SOFTWARE_LIMIT,
    FAULT_EXTERNAL_BREAK,
    FAULT_WATCHDOG_RESET
} psfb_fault_t;

typedef struct {
    uint16_t raw;
    float filtered;
    float adc_voltage;
    float vout;
} psfb_adc_sample_t;

typedef struct {
    uint8_t control_mode;
    uint8_t feedback_enabled;
    float target_vout;
    float manual_command_percent;
    float manual_est_output_voltage;
    float manual_est_input_voltage;
} psfb_config_t;

typedef struct {
    float command_percent;
    float applied_percent;
    float ramped_target_vout;
} psfb_runtime_status_t;

extern volatile uint16_t g_phase_ticks;
extern volatile psfb_state_t g_psfb_state;
extern volatile psfb_fault_t g_psfb_fault;
extern psfb_config_t g_psfb_config;
extern psfb_runtime_status_t g_psfb_status;

void psfb_init(void);
void psfb_start(void);
void psfb_control_1khz(const psfb_adc_sample_t *adc);
void psfb_disable_outputs(psfb_fault_t fault);
void psfb_clear_fault(void);
void psfb_set_phase_ticks(uint16_t ticks);
uint16_t psfb_percent_to_phase_ticks(float phase_percent);
uint8_t psfb_deadtime_ns_to_dtg(uint32_t deadtime_ns);
float psfb_get_command_percent(void);

#endif /* PSFB_CONTROL_H */
