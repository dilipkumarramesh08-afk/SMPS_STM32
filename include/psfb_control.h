#ifndef PSFB_CONTROL_H
#define PSFB_CONTROL_H

#include <stdint.h>

#define ADC_REF_MV                  3300U
#define ADC_FULL_SCALE              4095U
#define VOUT_DIVIDER_TOP_OHM        100000U
#define VOUT_DIVIDER_BOTTOM_OHM     10000U
#define VOUT_FEEDBACK_SCALE_NUM     (VOUT_DIVIDER_TOP_OHM + VOUT_DIVIDER_BOTTOM_OHM)
#define VOUT_FEEDBACK_SCALE_DEN     VOUT_DIVIDER_BOTTOM_OHM

#define TARGET_VOUT_MIN_MV          24000U
#define TARGET_VOUT_MAX_MV          28000U

#define FSW_HZ                      100000UL
#define TIM1_CLK_HZ                 72000000UL
#define CONTROL_LOOP_HZ             1000UL
#define DEADTIME_NS                 500UL

#define SOFTSTART_TIME_MS           3000UL
#define PHASE_SHIFT_MAX_INITIAL_PERMILLE  200U
#define PHASE_SHIFT_MAX_ABSOLUTE_PERMILLE 850U
#define TEST_UNLOCK_HIGH_PHASE      0
#define PHASE_SHIFT_SLEW_UP_PERMILLE_PER_MS 5U
#define SAFE_LOW_PHASE_PERMILLE     50U

#define OVP_MULTIPLIER_NUM          5U
#define OVP_MULTIPLIER_DEN          4U
#define VOUT_COLLAPSE_RATIO_NUM     7U
#define VOUT_COLLAPSE_RATIO_DEN     10U
#define ADC_NEAR_FULL_SCALE_LIMIT   3900U
#define ADC_FILTER_SHIFT            4U
#define ADC_DMA_SAMPLES             8U
#define FEEDBACK_LOW_TIMEOUT_MS     800U

#define TIM1_ARR_VALUE              ((TIM1_CLK_HZ / (2UL * FSW_HZ)) - 1UL)

#if TEST_UNLOCK_HIGH_PHASE
#define PHASE_SHIFT_MAX_ALLOWED_PERMILLE PHASE_SHIFT_MAX_ABSOLUTE_PERMILLE
#else
#define PHASE_SHIFT_MAX_ALLOWED_PERMILLE PHASE_SHIFT_MAX_INITIAL_PERMILLE
#endif

typedef enum {
    FAULT_NONE = 0,
    FAULT_INVALID_TARGET,
    FAULT_OVERVOLTAGE,
    FAULT_ADC_NEAR_FULL_SCALE,
    FAULT_FEEDBACK_LOW_OR_DISCONNECTED,
    FAULT_SOFTWARE_LIMIT,
    FAULT_EXTERNAL_BREAK
} fault_t;

typedef struct {
    uint16_t adc_raw;
    uint16_t adc_filtered;
    uint32_t vout_mv;
    uint32_t target_ramped_mv;
    uint16_t phase_cmd_permille;
    uint16_t phase_actual_permille;
    fault_t fault;
} psfb_status_t;

extern volatile psfb_status_t g_psfb;

void psfb_init_timer(void);
void psfb_start_outputs(void);
void psfb_control_1khz(uint32_t target_vout_mv, uint16_t adc_raw);
void psfb_latch_fault(fault_t fault);
uint8_t psfb_deadtime_dtg(void);

#endif
