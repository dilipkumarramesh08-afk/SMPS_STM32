#ifndef PSFB_CONTROL_H
#define PSFB_CONTROL_H

#include <stdint.h>

#define ADC_FULL_SCALE_RAW            4095U
#define OPTO_TARGET_FEEDBACK_RAW      2048U
#define OPTO_CONTROL_DEADBAND_RAW     8U
#define OPTO_FAST_ERROR_RAW           300U
#define OPTO_FULL_POWER_ERROR_RAW     1800U
#define OPTO_OUTPUT_HIGH_PULLS_LOW    1U

#define SYSCLK_HZ                     72000000UL
#define HSE_CLK_HZ                    8000000UL
#define APB1_PRESCALER                2UL
#define APB2_PRESCALER                1UL
#define APB1_CLK_HZ                   (SYSCLK_HZ / APB1_PRESCALER)
#define APB2_CLK_HZ                   (SYSCLK_HZ / APB2_PRESCALER)
#define TIM_APB1_CLK_HZ               ((APB1_PRESCALER == 1UL) ? APB1_CLK_HZ : (2UL * APB1_CLK_HZ))
#define TIM1_CLK_HZ                   ((APB2_PRESCALER == 1UL) ? APB2_CLK_HZ : (2UL * APB2_CLK_HZ))

/*
 * Power-stage timing configuration.
 *
 * When changing FSW_HZ or NORMAL_DEADTIME_NS, review every value in this block.
 * The voltage-control loop is derived from FSW_HZ so the loop bandwidth scales
 * with switching frequency. Default ratio: FSW_HZ / 25.
 * TIM1 primary PWM, TIM3 ADC trigger timing, PI/slew response, and oscilloscope
 * validation are coupled.
 */
#define FSW_HZ                        40000UL
#define CONTROL_LOOP_FSW_DIVIDER      25UL
#define CONTROL_LOOP_HZ               (FSW_HZ / CONTROL_LOOP_FSW_DIVIDER)
#define NORMAL_DEADTIME_NS            1000UL
#define PSFB_PHASE_MAX_PERMILLE       900U
#define PSFB_PHASE_START_PERMILLE     0U

/*
 * 0: Q3/Q4 right leg is leading, Q1/Q2 left leg is lagging.
 * 1: Q1/Q2 left leg is leading, Q3/Q4 right leg is lagging.
 */
#define PSFB_LAGGING_LEG_RIGHT        0U

#define LEFT_GATE_DRIVER_ACTIVE_HIGH  1
#define RIGHT_GATE_DRIVER_ACTIVE_HIGH 1

#define PI_KP_SHIFT                   4U
#define PI_KI_SHIFT                   8U
#define PHASE_SLEW_Q_SHIFT            8U
#define PHASE_SLEW_UP_LOW_PERMILLE_PER_SEC 2500U
#define PHASE_SLEW_DOWN_LOW_PERMILLE_PER_SEC 8000U
#define PHASE_SLEW_UP_HIGH_PERMILLE_PER_SEC 3000U
#define PHASE_SLEW_DOWN_HIGH_PERMILLE_PER_SEC 12000U

#define ADC_FILTER_FAST_SHIFT         1U
#define ADC_FILTER_SLOW_SHIFT         3U
#define ADC_FILTER_FAST_DELTA_RAW     8U
#define ADC_DMA_SAMPLES               16U
#define ADC_DMA_HALF_SAMPLES          (ADC_DMA_SAMPLES / 2U)
#define ADC_TRIGGER_EDGE_GUARD_TICKS  (TIM1_DEADTIME_RAW_TICKS + 8UL)
#define ADC_SMPR2_CH0_55_5_CYCLES     (ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_0)

#define IRQ_PRIORITY_CONTROL          2U

/*
 * TIM1 uses output-compare toggle mode for true PSFB timing.
 * Full bipolar switching period = 2 * (ARR + 1) / TIM1_CLK_HZ.
 */
#define TIM1_HALF_PERIOD_TICKS        (TIM1_CLK_HZ / (2UL * FSW_HZ))
#define TIM1_ARR_VALUE                (TIM1_HALF_PERIOD_TICKS - 1UL)
#define TIM1_DEADTIME_RAW_TICKS       (((NORMAL_DEADTIME_NS * (TIM1_CLK_HZ / 1000000UL)) + 999UL) / 1000UL)
#define TIM1_DEADTIME_ACTUAL_NS       (((TIM1_DEADTIME_RAW_TICKS * 1000UL) + ((TIM1_CLK_HZ / 1000000UL) / 2UL)) / (TIM1_CLK_HZ / 1000000UL))

#if TIM1_HALF_PERIOD_TICKS < 100UL
#error "FSW_HZ is too high for safe TIM1 PSFB timing resolution"
#endif

#if (CONTROL_LOOP_FSW_DIVIDER == 0UL) || ((FSW_HZ % CONTROL_LOOP_FSW_DIVIDER) != 0UL)
#error "CONTROL_LOOP_FSW_DIVIDER must divide FSW_HZ exactly"
#endif

#if (CONTROL_LOOP_HZ < 1000UL) || (CONTROL_LOOP_HZ > 10000UL)
#error "CONTROL_LOOP_HZ derived from FSW_HZ is outside the intended 1 kHz..10 kHz range"
#endif

#if (TIM1_DEADTIME_RAW_TICKS == 0UL) || (TIM1_DEADTIME_RAW_TICKS > 1008UL)
#error "NORMAL_DEADTIME_NS is outside TIM1 BDTR.DTG supported range"
#endif

#if (PSFB_PHASE_MAX_PERMILLE == 0U) || (PSFB_PHASE_MAX_PERMILLE > 950U)
#error "PSFB_PHASE_MAX_PERMILLE must be 1..950"
#endif

#if (PSFB_LAGGING_LEG_RIGHT != 0U) && (PSFB_LAGGING_LEG_RIGHT != 1U)
#error "PSFB_LAGGING_LEG_RIGHT must be 0 or 1"
#endif

#if (ADC_DMA_SAMPLES < 4U) || ((ADC_DMA_SAMPLES % 2U) != 0U)
#error "ADC_DMA_SAMPLES must be an even value of at least 4"
#endif

#if (ADC_DMA_HALF_SAMPLES < 4U)
#error "ADC_DMA_HALF_SAMPLES must be at least 4"
#endif

#if (OPTO_OUTPUT_HIGH_PULLS_LOW != 0U) && (OPTO_OUTPUT_HIGH_PULLS_LOW != 1U)
#error "OPTO_OUTPUT_HIGH_PULLS_LOW must be 0 or 1"
#endif

#if (OPTO_TARGET_FEEDBACK_RAW < 200U) || (OPTO_TARGET_FEEDBACK_RAW > 3800U)
#error "OPTO_TARGET_FEEDBACK_RAW must stay away from ADC rails"
#endif

#if (OPTO_FULL_POWER_ERROR_RAW == 0U)
#error "OPTO_FULL_POWER_ERROR_RAW must be non-zero"
#endif

#if (LEFT_GATE_DRIVER_ACTIVE_HIGH != 0) && (LEFT_GATE_DRIVER_ACTIVE_HIGH != 1)
#error "LEFT_GATE_DRIVER_ACTIVE_HIGH must be 0 or 1"
#endif

#if (RIGHT_GATE_DRIVER_ACTIVE_HIGH != 0) && (RIGHT_GATE_DRIVER_ACTIVE_HIGH != 1)
#error "RIGHT_GATE_DRIVER_ACTIVE_HIGH must be 0 or 1"
#endif

typedef enum {
    FAULT_NONE = 0,
    FAULT_INVALID_CONFIG
} fault_t;

typedef struct {
    uint16_t adc_raw;
    uint16_t adc_filtered;
    uint16_t feedback_normalized_raw;
    int32_t feedback_error_raw;
    uint16_t phase_actual_permille;
    uint16_t phase_limit_permille;
    fault_t fault;
} psfb_status_t;

extern volatile psfb_status_t g_psfb;

void psfb_init_timer(void);
void psfb_start_outputs(void);
void psfb_control_step(uint16_t adc_raw);
void psfb_latch_fault(fault_t fault);
void psfb_set_phase_permille(uint16_t phase_permille);

#endif
