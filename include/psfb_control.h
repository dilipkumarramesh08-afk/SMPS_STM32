#ifndef PSFB_CONTROL_H
#define PSFB_CONTROL_H

#include <stdint.h>

#define ADC_REF_MV                    3300U
#define ADC_FULL_SCALE                4095U
#define VOUT_DIVIDER_GAIN_NUM         11U
#define VOUT_DIVIDER_GAIN_DEN         1U

#define TARGET_VOUT_MIN_MV            12000U
#define TARGET_VOUT_MAX_MV            30000U

#define LEFT_GATE_DRIVER_ACTIVE_HIGH  1
#define RIGHT_GATE_DRIVER_ACTIVE_HIGH 1

#define SYSCLK_HZ                     72000000UL
#define HSE_CLK_HZ                    8000000UL
#define APB1_PRESCALER                2UL
#define APB2_PRESCALER                1UL
#define APB2_CLK_HZ                   (SYSCLK_HZ / APB2_PRESCALER)
#define TIM1_CLK_HZ                   ((APB2_PRESCALER == 1UL) ? APB2_CLK_HZ : (2UL * APB2_CLK_HZ))

#define FSW_HZ                        100000UL
#define CONTROL_LOOP_HZ               8000UL
#define NORMAL_DEADTIME_NS            100UL
#define SOFTSTART_TIME_MS             3000UL

#define PSFB_PHASE_MAX_PERMILLE       900U
#define PSFB_PHASE_START_PERMILLE     0U

#define VOUT_CONTROL_DEADBAND_MV      25U
#define FAST_CONTROL_THRESHOLD_NUM    7U
#define FAST_CONTROL_THRESHOLD_DEN    10U
#define PI_KP_SHIFT                   5U
#define PI_KI_SHIFT                   9U
#define SOFTSTART_RAMP_Q_SHIFT        8U
#define PHASE_SLEW_Q_SHIFT            8U
#define PHASE_SLEW_UP_LOW_PERMILLE_PER_SEC 300U
#define PHASE_SLEW_DOWN_LOW_PERMILLE_PER_SEC 1000U
#define PHASE_SLEW_UP_HIGH_PERMILLE_PER_SEC 3000U
#define PHASE_SLEW_DOWN_HIGH_PERMILLE_PER_SEC 8000U

#define OVP_MARGIN_MV                 4000U
#define OVP_MAX_MV                    34000U
#define OVP_CONFIRM_COUNT             24U
#define ADC_NEAR_FULL_SCALE_LIMIT     3900U
#define ADC_NEAR_FULL_CONFIRM_COUNT   24U
#define ADC_FILTER_FAST_SHIFT         1U
#define ADC_FILTER_SLOW_SHIFT         3U
#define ADC_FILTER_FAST_DELTA_RAW     8U
#define ADC_DMA_SAMPLES               16U
#define ADC_SMPR2_CH0_55_5_CYCLES     (ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_0)

#define FEEDBACK_PROTECTION_BLANKING_MS 4000U
#define FEEDBACK_LOW_TIMEOUT_MS       3000U

#define IRQ_PRIORITY_DMA              1U
#define IRQ_PRIORITY_SYSTICK          2U

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

#if (TIM1_DEADTIME_RAW_TICKS == 0UL) || (TIM1_DEADTIME_RAW_TICKS > 1008UL)
#error "NORMAL_DEADTIME_NS is outside TIM1 BDTR.DTG supported range"
#endif

#if (PSFB_PHASE_MAX_PERMILLE == 0U) || (PSFB_PHASE_MAX_PERMILLE > 950U)
#error "PSFB_PHASE_MAX_PERMILLE must be 1..950"
#endif

#if (ADC_DMA_SAMPLES < 4U) || ((ADC_DMA_SAMPLES % 2U) != 0U)
#error "ADC_DMA_SAMPLES must be an even value of at least 4"
#endif

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
    FAULT_FEEDBACK_LOW_OR_DISCONNECTED
} fault_t;

typedef struct {
    uint16_t adc_raw;
    uint16_t adc_filtered;
    uint32_t vout_mv;
    uint32_t target_ramped_mv;
    int32_t error_mv;
    uint16_t phase_cmd_permille;
    uint16_t phase_target_permille;
    uint16_t phase_actual_permille;
    uint16_t phase_limit_permille;
    fault_t fault;
} psfb_status_t;

extern volatile psfb_status_t g_psfb;

void psfb_init_timer(void);
void psfb_start_outputs(void);
void psfb_control_step(uint32_t target_vout_mv, uint16_t adc_raw);
void psfb_latch_fault(fault_t fault);
void psfb_set_phase_permille(uint16_t phase_permille);
uint32_t psfb_adc_raw_to_vout_mv(uint16_t adc_raw);
uint32_t psfb_ovp_limit_mv(uint32_t target_vout_mv);

#endif
