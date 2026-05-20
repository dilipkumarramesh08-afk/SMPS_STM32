#ifndef HBRIDGE_CONTROL_H
#define HBRIDGE_CONTROL_H

#include <stdint.h>

#define ADC_REF_MV                    3300U
#define ADC_FULL_SCALE                4095U
#define VOUT_DIVIDER_GAIN_NUM         11U
#define VOUT_DIVIDER_GAIN_DEN         1U

#define TARGET_VOUT_MIN_MV            12000U
#define TARGET_VOUT_MAX_MV            28000U

#define LEFT_GATE_DRIVER_ACTIVE_HIGH  1
#define RIGHT_GATE_DRIVER_ACTIVE_HIGH 1

#define SYSCLK_HZ                     72000000UL
#define HSE_CLK_HZ                    8000000UL
#define APB1_PRESCALER                2UL
#define APB2_PRESCALER                1UL
#define APB2_CLK_HZ                   (SYSCLK_HZ / APB2_PRESCALER)
#define TIM1_CLK_HZ                   ((APB2_PRESCALER == 1UL) ? APB2_CLK_HZ : (2UL * APB2_CLK_HZ))

#define FSW_HZ                        20000UL
#define CONTROL_LOOP_HZ               8000UL
#define NORMAL_DEADTIME_NS            500UL

#define SOFTSTART_TIME_MS             3000UL
#define DUTY_MAX_PERMILLE             850U
#define DUTY_SLEW_UP_LOW_PERMILLE_PER_SEC 300U
#define DUTY_SLEW_DOWN_LOW_PERMILLE_PER_SEC 1000U
#define DUTY_SLEW_UP_HIGH_PERMILLE_PER_SEC 3000U
#define DUTY_SLEW_DOWN_HIGH_PERMILLE_PER_SEC 8000U
#define DUTY_START_PERMILLE           0U
#define VOUT_CONTROL_DEADBAND_MV      25U
#define FAST_CONTROL_THRESHOLD_NUM    7U
#define FAST_CONTROL_THRESHOLD_DEN    10U
#define PI_KP_SHIFT                   5U
#define PI_KI_SHIFT                   9U
#define SOFTSTART_RAMP_Q_SHIFT        8U
#define DUTY_SLEW_Q_SHIFT             8U

#define OVP_MULTIPLIER_NUM            3U
#define OVP_MULTIPLIER_DEN            2U
#define OVP_CONFIRM_COUNT             24U
#define ADC_NEAR_FULL_SCALE_LIMIT     3900U
#define ADC_FILTER_SHIFT              2U
#define ADC_DMA_SAMPLES               16U
#define ADC_USE_TIM1_TRIGGER          0

#define FEEDBACK_PROTECTION_BLANKING_MS 4000U
#define FEEDBACK_LOW_TIMEOUT_MS       3000U

#define IRQ_PRIORITY_TIM1             0U
#define IRQ_PRIORITY_SYSTICK          2U

/*
 * Center-aligned TIM1 PWM.
 * Full bipolar PWM period = 2 * (ARR + 1) / TIM1_CLK_HZ.
 */
#define TIM1_HALF_PERIOD_TICKS        (TIM1_CLK_HZ / (2UL * FSW_HZ))
#define TIM1_ARR_VALUE                (TIM1_HALF_PERIOD_TICKS - 1UL)

#if (LEFT_GATE_DRIVER_ACTIVE_HIGH != 0) && (LEFT_GATE_DRIVER_ACTIVE_HIGH != 1)
#error "LEFT_GATE_DRIVER_ACTIVE_HIGH must be 0 or 1"
#endif

#if (RIGHT_GATE_DRIVER_ACTIVE_HIGH != 0) && (RIGHT_GATE_DRIVER_ACTIVE_HIGH != 1)
#error "RIGHT_GATE_DRIVER_ACTIVE_HIGH must be 0 or 1"
#endif

typedef enum {
    FAULT_NONE = 0,
    FAULT_INVALID_TARGET,
    FAULT_OVERVOLTAGE,
    FAULT_ADC_NEAR_FULL_SCALE,
    FAULT_FEEDBACK_LOW_OR_DISCONNECTED,
    FAULT_SOFTWARE_LIMIT
} fault_t;

typedef struct {
    uint16_t adc_raw;
    uint16_t adc_filtered;
    uint32_t vout_mv;
    uint32_t target_ramped_mv;
    int32_t error_mv;
    uint16_t duty_cmd_permille;
    uint16_t duty_target_permille;
    uint16_t duty_actual_permille;
    fault_t fault;
} hbridge_status_t;

extern volatile hbridge_status_t g_hbridge;

void hbridge_init_timer(void);
void hbridge_start_outputs(void);
void hbridge_control_step(uint32_t target_vout_mv, uint16_t adc_raw);
void hbridge_latch_fault(fault_t fault);
void hbridge_set_duty_permille(uint16_t duty_permille);
uint32_t hbridge_adc_raw_to_vout_mv(uint16_t adc_raw);
uint32_t hbridge_ovp_limit_mv(uint32_t target_vout_mv);
uint8_t hbridge_deadtime_dtg(void);

#endif
