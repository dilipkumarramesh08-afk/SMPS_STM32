#include "psfb_control.h"
#include "board_pins.h"

#include "stm32f1xx.h"

volatile psfb_status_t g_psfb = {0};

enum {
    TIM1_CCMR1_TOGGLE_MODES = (3U << TIM_CCMR1_OC1M_Pos) |
                              (3U << TIM_CCMR1_OC2M_Pos),
    TIM1_CCMR1_REQUIRED = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE |
                          TIM1_CCMR1_TOGGLE_MODES
};

static uint16_t s_half_period_ticks = 0;
static volatile uint8_t s_outputs_allowed = 0;
static int32_t s_integrator_q8 = 0;
static uint32_t s_slew_up_accum_q = 0;
static uint32_t s_slew_down_accum_q = 0;

static void psfb_set_phase_raw(uint16_t phase_permille);

static uint16_t psfb_phase_limit(void)
{
    return PSFB_PHASE_MAX_PERMILLE;
}

static uint16_t psfb_adc_trigger_tick(uint32_t delayed_toggle_tick)
{
    const uint32_t leading_toggle_tick = 1UL;
    uint32_t trigger_tick;
    uint32_t before_delay_ticks;
    uint32_t after_delay_ticks;
    uint32_t guard_ticks = ADC_TRIGGER_EDGE_GUARD_TICKS;

    if (guard_ticks >= (s_half_period_ticks / 2UL)) {
        guard_ticks = 4UL;
    }

    if (delayed_toggle_tick <= leading_toggle_tick) {
        delayed_toggle_tick = leading_toggle_tick + 1UL;
    } else if (delayed_toggle_tick >= s_half_period_ticks) {
        delayed_toggle_tick = s_half_period_ticks - 1UL;
    }

    before_delay_ticks = delayed_toggle_tick - leading_toggle_tick;
    after_delay_ticks = s_half_period_ticks - delayed_toggle_tick;

    if (before_delay_ticks >= after_delay_ticks) {
        trigger_tick = leading_toggle_tick + (before_delay_ticks / 2UL);
    } else {
        trigger_tick = delayed_toggle_tick + (after_delay_ticks / 2UL);
    }

    if (trigger_tick < guard_ticks) {
        trigger_tick = guard_ticks;
    } else if (trigger_tick > (s_half_period_ticks - guard_ticks)) {
        trigger_tick = s_half_period_ticks - guard_ticks;
    }

    return (uint16_t)trigger_tick;
}

static void adc_trigger_timer_configure(uint16_t trigger_tick)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;

    TIM3->CR1 = 0U;
    TIM3->CR2 = TIM_CR2_MMS_0 | TIM_CR2_MMS_1;
    TIM3->SMCR = TIM_SMCR_SMS_2;
    TIM3->DIER = 0U;
    TIM3->PSC = 0U;
    TIM3->ARR = (uint16_t)((2UL * s_half_period_ticks) - 1UL);
    TIM3->CCR1 = trigger_tick;
    TIM3->CCMR1 = TIM_CCMR1_OC1PE;
    TIM3->CCER = 0U;
    TIM3->CNT = 0U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void tim1_disable_outputs(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM3->CR1 &= ~TIM_CR1_CEN;
}

static void tim1_enable_outputs_if_allowed(void)
{
    if ((s_outputs_allowed != 0U) && (g_psfb.fault == FAULT_NONE)) {
        TIM1->BDTR |= TIM_BDTR_MOE;
    }
}

static inline void psfb_emergency_shutdown(void)
{
    s_outputs_allowed = 0U;
    tim1_disable_outputs();
    g_psfb.phase_actual_permille = 0U;
    g_psfb.phase_limit_permille = psfb_phase_limit();
    g_psfb.feedback_normalized_raw = 0U;
    g_psfb.feedback_error_raw = 0;
    s_integrator_q8 = 0;
    s_slew_up_accum_q = 0U;
    s_slew_down_accum_q = 0U;
    psfb_set_phase_raw(0U);
    __DSB();
}

static uint32_t tim1_output_polarity_bits(void)
{
    uint32_t bits = 0U;

#if !LEFT_GATE_DRIVER_ACTIVE_HIGH
    bits |= TIM_CCER_CC1P | TIM_CCER_CC1NP;
#endif
#if !RIGHT_GATE_DRIVER_ACTIVE_HIGH
    bits |= TIM_CCER_CC2P | TIM_CCER_CC2NP;
#endif

    return bits;
}

static uint32_t tim1_off_state_bits(void)
{
    uint32_t bits = 0U;

#if !LEFT_GATE_DRIVER_ACTIVE_HIGH
    bits |= TIM_CR2_OIS1 | TIM_CR2_OIS1N;
#endif
#if !RIGHT_GATE_DRIVER_ACTIVE_HIGH
    bits |= TIM_CR2_OIS2 | TIM_CR2_OIS2N;
#endif

    return bits;
}

static uint32_t deadtime_ticks_raw(void)
{
    return TIM1_DEADTIME_RAW_TICKS;
}

static uint8_t psfb_deadtime_dtg(void)
{
    uint32_t ticks = deadtime_ticks_raw();

    /*
     * RM0008 TIMx_BDTR.DTG encoding:
     * 0xx: DT = DTG * tDTS
     * 10x: DT = (64 + DTG[5:0]) * 2 * tDTS
     * 110: DT = (32 + DTG[4:0]) * 8 * tDTS
     * 111: DT = (32 + DTG[4:0]) * 16 * tDTS
     */
    if (ticks <= 127UL) {
        return (uint8_t)ticks;
    }
    if (ticks <= 254UL) {
        return (uint8_t)(0x80UL | (((ticks + 1UL) / 2UL) - 64UL));
    }
    if (ticks <= 504UL) {
        return (uint8_t)(0xC0UL | (((ticks + 7UL) / 8UL) - 32UL));
    }
    if (ticks <= 1008UL) {
        return (uint8_t)(0xE0UL | (((ticks + 15UL) / 16UL) - 32UL));
    }
    return 0xFFU;
}

static void psfb_set_phase_raw(uint16_t phase_permille)
{
    uint32_t phase_ticks;
    uint32_t right_toggle_tick;
    uint16_t phase_limit = psfb_phase_limit();

    if (phase_permille > phase_limit) {
        phase_permille = phase_limit;
    }

    phase_ticks = ((uint32_t)phase_permille * s_half_period_ticks) / 1000UL;
    right_toggle_tick = 1UL + phase_ticks;
    if (right_toggle_tick >= TIM1->ARR) {
        right_toggle_tick = TIM1->ARR - 1UL;
    }

#if PSFB_LAGGING_LEG_RIGHT
    TIM1->CCR1 = 1U;
    TIM1->CCR2 = (uint16_t)right_toggle_tick;
#else
    TIM1->CCR1 = (uint16_t)right_toggle_tick;
    TIM1->CCR2 = 1U;
#endif
    TIM3->CCR1 = psfb_adc_trigger_tick(right_toggle_tick);
    TIM1->CCR4 = 0U;

    g_psfb.phase_actual_permille = phase_permille;
    g_psfb.phase_limit_permille = phase_limit;
}

void psfb_set_phase_permille(uint16_t phase_permille)
{
    psfb_set_phase_raw(phase_permille);
    tim1_enable_outputs_if_allowed();
}

void psfb_latch_fault(fault_t fault)
{
    if (g_psfb.fault == FAULT_NONE) {
        g_psfb.fault = fault;
    }
    psfb_emergency_shutdown();
}

void psfb_init_timer(void)
{
    uint32_t ccmr1;

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    s_outputs_allowed = 0U;
    TIM1->BDTR = 0U;
    TIM1->CR1 = 0U;
    TIM1->CR2 = tim1_off_state_bits() | TIM_CR2_MMS_1;
    TIM1->CCER = 0U;
    TIM1->DIER = 0U;
    TIM1->RCR = 1U;
    TIM1->PSC = 0U;

    s_half_period_ticks = (uint16_t)(TIM1_ARR_VALUE + 1UL);
    TIM1->ARR = (uint16_t)TIM1_ARR_VALUE;
    adc_trigger_timer_configure(psfb_adc_trigger_tick(1UL));

    /*
     * True PSFB timing:
     * - CH1/CH1N left leg toggles every half-cycle.
     * - CH2/CH2N right leg toggles after phase_ticks.
     * - At phase = 0 both legs toggle together, so transformer voltage is zero.
     */
    ccmr1 = TIM1_CCMR1_REQUIRED;
    TIM1->CCMR1 = ccmr1;
    TIM1->CCMR2 = 0U;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 tim1_output_polarity_bits();
    TIM1->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR | psfb_deadtime_dtg();

    psfb_set_phase_raw(PSFB_PHASE_START_PERMILLE);

    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;
    TIM1->DIER = 0U;
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void psfb_start_outputs(void)
{
    if (g_psfb.fault == FAULT_NONE) {
        s_outputs_allowed = 1U;
        TIM1->CR1 &= ~TIM_CR1_CEN;
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM1->CNT = 0U;
        TIM3->CNT = 0U;
        TIM1->SR = 0U;
        TIM3->SR = 0U;
        TIM3->CR1 |= TIM_CR1_CEN;
        TIM1->EGR = TIM_EGR_UG;
        TIM1->CR1 |= TIM_CR1_CEN;
        tim1_enable_outputs_if_allowed();
    }
}

static uint16_t adc_iir_filter(uint16_t previous, uint16_t sample)
{
    int32_t delta = (int32_t)sample - (int32_t)previous;
    uint32_t abs_delta = (delta < 0) ? (uint32_t)(-delta) : (uint32_t)delta;
    uint32_t shift = (abs_delta >= ADC_FILTER_FAST_DELTA_RAW) ?
        ADC_FILTER_FAST_SHIFT : ADC_FILTER_SLOW_SHIFT;
    int32_t step;

    if (delta >= 0) {
        step = (delta + (int32_t)(1UL << (shift - 1U))) >> shift;
    } else {
        step = -(((-delta) + (int32_t)(1UL << (shift - 1U))) >> shift);
    }

    return (uint16_t)((int32_t)previous + step);
}

static uint16_t pi_controller(int32_t error_mv, uint16_t max_permille)
{
    int32_t p;
    int32_t candidate_integrator_q8;
    int32_t integrator_permille;
    int32_t integrator_min_q8;
    int32_t integrator_max_q8;
    int32_t out;
    uint16_t saturated;

    p = error_mv >> PI_KP_SHIFT;
    candidate_integrator_q8 = s_integrator_q8 + (error_mv >> PI_KI_SHIFT);
    integrator_min_q8 = -((int32_t)max_permille << 8);
    integrator_max_q8 = ((int32_t)max_permille << 8);

    if (candidate_integrator_q8 > integrator_max_q8) {
        candidate_integrator_q8 = integrator_max_q8;
    } else if (candidate_integrator_q8 < integrator_min_q8) {
        candidate_integrator_q8 = integrator_min_q8;
    }

    integrator_permille = candidate_integrator_q8 >> 8;
    out = p + integrator_permille;
    if (out < 0) {
        saturated = 0U;
    } else if (out > max_permille) {
        saturated = max_permille;
    } else {
        saturated = (uint16_t)out;
    }

    if (((saturated > 0U) && (saturated < max_permille)) ||
        ((saturated == 0U) && (error_mv > 0)) ||
        ((saturated == max_permille) && (error_mv < 0))) {
        s_integrator_q8 = candidate_integrator_q8;
    }

    return saturated;
}

static uint16_t opto_error_feedforward_phase(int32_t error_raw, uint16_t phase_limit)
{
    uint32_t positive_error;
    uint32_t phase;

    if (error_raw <= 0) {
        return 0U;
    }

    positive_error = (uint32_t)error_raw;
    if (positive_error >= OPTO_FULL_POWER_ERROR_RAW) {
        return phase_limit;
    }

    phase = (positive_error * phase_limit) / OPTO_FULL_POWER_ERROR_RAW;
    return (uint16_t)((phase > phase_limit) ? phase_limit : phase);
}

static uint16_t opto_normalized_feedback(uint16_t adc_filtered)
{
#if OPTO_OUTPUT_HIGH_PULLS_LOW
    return (uint16_t)(ADC_FULL_SCALE_RAW - adc_filtered);
#else
    return adc_filtered;
#endif
}

static uint8_t opto_feedback_near_target(void)
{
    int32_t error = g_psfb.feedback_error_raw;

    if (error < 0) {
        error = -error;
    }

    return (error <= (int32_t)OPTO_FAST_ERROR_RAW) ? 1U : 0U;
}

static uint16_t active_slew_up_permille(void)
{
    if (opto_feedback_near_target() != 0U) {
        return PHASE_SLEW_UP_HIGH_PERMILLE_PER_SEC;
    }
    return PHASE_SLEW_UP_LOW_PERMILLE_PER_SEC;
}

static uint16_t active_slew_down_permille(void)
{
    if (opto_feedback_near_target() != 0U) {
        return PHASE_SLEW_DOWN_HIGH_PERMILLE_PER_SEC;
    }
    return PHASE_SLEW_DOWN_LOW_PERMILLE_PER_SEC;
}

static uint16_t slew_steps_from_rate(uint16_t permille_per_sec, uint32_t *accum_q)
{
    uint32_t step_q =
        ((uint32_t)permille_per_sec << PHASE_SLEW_Q_SHIFT) /
        CONTROL_LOOP_HZ;
    uint16_t steps;

    *accum_q += step_q;
    steps = (uint16_t)(*accum_q >> PHASE_SLEW_Q_SHIFT);
    *accum_q &= (1UL << PHASE_SLEW_Q_SHIFT) - 1UL;

    return steps;
}

void psfb_control_step(uint16_t adc_raw)
{
    uint16_t phase_cmd;
    uint16_t phase_ff;
    uint16_t phase_limit;
    uint16_t slew_up;
    uint16_t slew_down;
    int32_t error_mv;

    if (g_psfb.fault != FAULT_NONE) {
        psfb_emergency_shutdown();
        return;
    }

    g_psfb.adc_raw = adc_raw;
    if (g_psfb.adc_filtered == 0U) {
        g_psfb.adc_filtered = adc_raw;
    } else {
        g_psfb.adc_filtered = adc_iir_filter(g_psfb.adc_filtered, adc_raw);
    }

    phase_limit = psfb_phase_limit();
    g_psfb.phase_limit_permille = phase_limit;
    g_psfb.feedback_normalized_raw = opto_normalized_feedback(g_psfb.adc_filtered);

    error_mv =
        (int32_t)OPTO_TARGET_FEEDBACK_RAW -
        (int32_t)g_psfb.feedback_normalized_raw;
    g_psfb.feedback_error_raw = error_mv;

    if ((error_mv > -(int32_t)OPTO_CONTROL_DEADBAND_RAW) &&
        (error_mv < (int32_t)OPTO_CONTROL_DEADBAND_RAW)) {
        phase_cmd = g_psfb.phase_actual_permille;
    } else {
        phase_cmd = pi_controller(error_mv, phase_limit);
    }

    phase_ff = opto_error_feedforward_phase(error_mv, phase_limit);
    if (phase_cmd < phase_ff) {
        phase_cmd = phase_ff;
    }

    if (phase_cmd > phase_limit) {
        phase_cmd = phase_limit;
    }

    if (phase_cmd > g_psfb.phase_actual_permille) {
        slew_up = slew_steps_from_rate(active_slew_up_permille(), &s_slew_up_accum_q);
        s_slew_down_accum_q = 0U;
        uint32_t next = (uint32_t)g_psfb.phase_actual_permille + slew_up;
        g_psfb.phase_actual_permille = (next > phase_cmd) ? phase_cmd : (uint16_t)next;
    } else if (phase_cmd < g_psfb.phase_actual_permille) {
        uint16_t current = g_psfb.phase_actual_permille;
        slew_down = slew_steps_from_rate(active_slew_down_permille(), &s_slew_down_accum_q);
        s_slew_up_accum_q = 0U;
        g_psfb.phase_actual_permille =
            ((current - phase_cmd) > slew_down) ?
            (uint16_t)(current - slew_down) : phase_cmd;
    } else {
        s_slew_up_accum_q = 0U;
        s_slew_down_accum_q = 0U;
        g_psfb.phase_actual_permille = phase_cmd;
    }

    psfb_set_phase_permille(g_psfb.phase_actual_permille);
}
