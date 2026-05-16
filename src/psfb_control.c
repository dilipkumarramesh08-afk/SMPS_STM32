#include "psfb_control.h"

#include "stm32f1xx.h"

volatile psfb_status_t g_psfb = {0};

static volatile uint16_t s_pending_phase_ticks = 0;
static int32_t s_integrator = 0;
static uint16_t s_feedback_low_ms = 0;
static uint8_t s_outputs_enabled = 0;

static inline void psfb_emergency_shutdown(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    s_pending_phase_ticks = 0;
    g_psfb.phase_actual_permille = 0;
    g_psfb.phase_cmd_permille = 0;
    TIM1->CCR2 = 1U;
    s_outputs_enabled = 0;
}

uint8_t psfb_deadtime_dtg(void)
{
    uint32_t ticks = ((DEADTIME_NS * (TIM1_CLK_HZ / 1000000UL)) + 999UL) / 1000UL;

    // cppcheck-suppress knownConditionTrueFalse
    if (ticks > 127UL) {
        ticks = 127UL;
    }
    return (uint8_t)ticks;
}

static uint16_t phase_permille_to_ticks(uint16_t permille)
{
    uint32_t ticks;

    if (permille > PHASE_SHIFT_MAX_ABSOLUTE_PERMILLE) {
        permille = PHASE_SHIFT_MAX_ABSOLUTE_PERMILLE;
    }

    ticks = ((uint32_t)permille * TIM1_ARR_VALUE) / 1000UL;
    if (ticks > TIM1_ARR_VALUE) {
        ticks = TIM1_ARR_VALUE;
    }
    return (uint16_t)ticks;
}

static void schedule_phase_update(uint16_t phase_ticks)
{
    if (phase_ticks > phase_permille_to_ticks(PHASE_SHIFT_MAX_ABSOLUTE_PERMILLE)) {
        phase_ticks = phase_permille_to_ticks(PHASE_SHIFT_MAX_ABSOLUTE_PERMILLE);
    }
    s_pending_phase_ticks = phase_ticks;
}

// cppcheck-suppress unusedFunction
void TIM1_UP_IRQHandler(void)
{
    if ((TIM1->SR & TIM_SR_UIF) != 0U) {
        TIM1->SR = (uint16_t)~TIM_SR_UIF;
        TIM1->CCR2 = (uint16_t)(1U + s_pending_phase_ticks);
    }
}

void psfb_latch_fault(fault_t fault)
{
    if (g_psfb.fault == FAULT_NONE) {
        g_psfb.fault = fault;
    }
    psfb_emergency_shutdown();
}

// cppcheck-suppress unusedFunction
void psfb_init_timer(void)
{
    uint32_t ccmr1;
    uint32_t bdtr;

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->BDTR = 0U;                 /* MOE off during all setup. */
    TIM1->CR1 = 0U;
    TIM1->CR2 = 0U;
    TIM1->CCER = 0U;
    TIM1->PSC = 0U;
    TIM1->ARR = (uint16_t)TIM1_ARR_VALUE;
    TIM1->RCR = 0U;

    /*
     * True PSFB timing:
     * CH1/CH1N are the fixed 50% left leg.
     * CH2/CH2N are the fixed 50% right leg.
     * OC toggle mode toggles once per timer update; one bridge-leg cycle has
     * one ON half-cycle and one OFF half-cycle. Phase is controlled only by CCR2.
     */
    TIM1->CR1 = TIM_CR1_ARPE;
    ccmr1 = (3U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
            (3U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR1 = ccmr1;
    TIM1->CCR1 = 1U;
    TIM1->CCR2 = 1U;

    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;

    bdtr = TIM_BDTR_OSSI | TIM_BDTR_OSSR | psfb_deadtime_dtg();
    /* PB12/TIM1_BKIN is reserved for future current-trip hardware. */
    TIM1->BDTR = bdtr;

    TIM1->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;
    TIM1->CR1 |= TIM_CR1_CEN;        /* Counter runs with outputs disabled. */
}

// cppcheck-suppress unusedFunction
void psfb_start_outputs(void)
{
    if (g_psfb.fault == FAULT_NONE) {
        TIM1->BDTR |= TIM_BDTR_MOE;
        s_outputs_enabled = 1;
    }
}

static uint32_t adc_to_vout_mv(uint16_t adc_raw)
{
    uint32_t adc_mv = ((uint32_t)adc_raw * ADC_REF_MV) / ADC_FULL_SCALE;
    return (adc_mv * VOUT_FEEDBACK_SCALE_NUM) / VOUT_FEEDBACK_SCALE_DEN;
}

static uint16_t pi_controller(int32_t error_mv, uint16_t max_permille)
{
    enum {
        KP_NUM = 1,
        KP_DEN_SHIFT = 9,
        KI_NUM = 1,
        KI_DEN_SHIFT = 12,
        INTEGRATOR_MIN = -120000,
        INTEGRATOR_MAX = 120000
    };
    int32_t p;
    int32_t candidate_integrator;
    int32_t out;
    uint16_t saturated;

    p = (error_mv * KP_NUM) >> KP_DEN_SHIFT;
    candidate_integrator = s_integrator + ((error_mv * KI_NUM) >> KI_DEN_SHIFT);

    if (candidate_integrator > INTEGRATOR_MAX) {
        candidate_integrator = INTEGRATOR_MAX;
    } else if (candidate_integrator < INTEGRATOR_MIN) {
        candidate_integrator = INTEGRATOR_MIN;
    }

    out = p + candidate_integrator;
    if (out < 0) {
        saturated = 0;
    } else if (out > max_permille) {
        saturated = max_permille;
    } else {
        saturated = (uint16_t)out;
    }

    if (((saturated > 0U) && (saturated < max_permille)) ||
        ((saturated == 0U) && (error_mv > 0)) ||
        ((saturated == max_permille) && (error_mv < 0))) {
        s_integrator = candidate_integrator;
    }

    return saturated;
}

// cppcheck-suppress unusedFunction
void psfb_control_1khz(uint32_t target_vout_mv, uint16_t adc_raw)
{
    uint32_t ovp_limit_mv;
    uint32_t collapse_limit_mv;
    uint16_t max_permille = PHASE_SHIFT_MAX_ALLOWED_PERMILLE;
    uint16_t phase_cmd;

    if (g_psfb.fault != FAULT_NONE) {
        psfb_emergency_shutdown();
        return;
    }

    if ((TIM1->SR & TIM_SR_BIF) != 0U) {
        psfb_latch_fault(FAULT_EXTERNAL_BREAK);
        return;
    }

    if ((target_vout_mv < TARGET_VOUT_MIN_MV) || (target_vout_mv > TARGET_VOUT_MAX_MV)) {
        psfb_latch_fault(FAULT_INVALID_TARGET);
        return;
    }

    g_psfb.adc_raw = adc_raw;
    if (g_psfb.adc_filtered == 0U) {
        g_psfb.adc_filtered = adc_raw;
    } else {
        int32_t delta = (int32_t)adc_raw - (int32_t)g_psfb.adc_filtered;
        g_psfb.adc_filtered = (uint16_t)((int32_t)g_psfb.adc_filtered +
                                         (delta >> ADC_FILTER_SHIFT));
    }

    g_psfb.vout_mv = adc_to_vout_mv(g_psfb.adc_filtered);

    if (g_psfb.adc_filtered > ADC_NEAR_FULL_SCALE_LIMIT) {
        psfb_latch_fault(FAULT_ADC_NEAR_FULL_SCALE);
        return;
    }

    ovp_limit_mv = (target_vout_mv * OVP_MULTIPLIER_NUM) / OVP_MULTIPLIER_DEN;
    if (g_psfb.vout_mv > ovp_limit_mv) {
        psfb_latch_fault(FAULT_OVERVOLTAGE);
        return;
    }

    if (g_psfb.target_ramped_mv < target_vout_mv) {
        uint32_t step = target_vout_mv / SOFTSTART_TIME_MS;
        if (step == 0UL) {
            step = 1UL;
        }
        g_psfb.target_ramped_mv += step;
        if (g_psfb.target_ramped_mv > target_vout_mv) {
            g_psfb.target_ramped_mv = target_vout_mv;
        }
    }

    collapse_limit_mv = (g_psfb.target_ramped_mv * VOUT_COLLAPSE_RATIO_NUM) /
                        VOUT_COLLAPSE_RATIO_DEN;

    if ((g_psfb.target_ramped_mv > 4000UL) && (g_psfb.vout_mv < collapse_limit_mv)) {
        s_integrator = 0;
        phase_cmd = SAFE_LOW_PHASE_PERMILLE;
        if (phase_cmd > max_permille) {
            phase_cmd = max_permille;
        }
    } else {
        int32_t error_mv;
        error_mv = (int32_t)g_psfb.target_ramped_mv - (int32_t)g_psfb.vout_mv;
        phase_cmd = pi_controller(error_mv, max_permille);
    }

    if ((s_outputs_enabled != 0U) &&
        (g_psfb.target_ramped_mv > 5000UL) &&
        (g_psfb.phase_actual_permille > 50U) &&
        (g_psfb.adc_filtered < 20U)) {
        if (++s_feedback_low_ms > FEEDBACK_LOW_TIMEOUT_MS) {
            psfb_latch_fault(FAULT_FEEDBACK_LOW_OR_DISCONNECTED);
            return;
        }
    } else {
        s_feedback_low_ms = 0U;
    }

    if (phase_cmd > max_permille) {
        phase_cmd = max_permille;
    }
    g_psfb.phase_cmd_permille = phase_cmd;

    if (phase_cmd > g_psfb.phase_actual_permille) {
        uint32_t next = (uint32_t)g_psfb.phase_actual_permille +
                        PHASE_SHIFT_SLEW_UP_PERMILLE_PER_MS;
        g_psfb.phase_actual_permille = (next > phase_cmd) ? phase_cmd : (uint16_t)next;
    } else {
        g_psfb.phase_actual_permille = phase_cmd;
    }

    schedule_phase_update(phase_permille_to_ticks(g_psfb.phase_actual_permille));
}
