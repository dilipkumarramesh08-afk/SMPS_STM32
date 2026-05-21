#include "psfb_control.h"

#include "stm32f1xx.h"

volatile psfb_status_t g_psfb = {0};

enum {
    TIM1_CCER_OUTPUT_ENABLES = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                               TIM_CCER_CC2E | TIM_CCER_CC2NE |
                               TIM_CCER_CC3E
};

static uint16_t s_half_period_ticks = 0;
static volatile uint8_t s_outputs_allowed = 0;
static int32_t s_integrator_q8 = 0;
static uint32_t s_ramp_target_mv_q = 0;
static uint32_t s_ramp_config_target_mv = 0;
static uint32_t s_ramp_step_q = 0;
static uint32_t s_slew_up_accum_q = 0;
static uint32_t s_slew_down_accum_q = 0;
static uint32_t s_control_ticks = 0;
static uint32_t s_control_target_mv = 0;
static uint32_t s_control_ovp_limit_mv = 0;
static uint16_t s_feedback_low_ticks = 0;
static uint8_t s_ovp_confirm_count = 0;
static uint8_t s_adc_full_confirm_count = 0;

static uint16_t psfb_phase_limit(void)
{
    return PSFB_PHASE_MAX_PERMILLE;
}

static uint16_t psfb_adc_trigger_tick(uint32_t right_toggle_tick)
{
    uint32_t trigger_tick;

    if (right_toggle_tick <= 4UL) {
        return (uint16_t)(s_half_period_ticks / 2UL);
    }

    /*
     * Sample in the long zero/freewheel interval between the left-leg toggle
     * and the delayed right-leg toggle. At high phase, the interval after the
     * right-leg toggle is short and can put the ADC aperture on the next edge.
     * TIM1_CH3 is internal only.
     */
    trigger_tick = right_toggle_tick / 2UL;

    if (trigger_tick < 2UL) {
        trigger_tick = 2UL;
    } else if (trigger_tick >= TIM1->ARR) {
        trigger_tick = TIM1->ARR - 1UL;
    }

    return (uint16_t)trigger_tick;
}

static void tim1_disable_outputs(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
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
    g_psfb.phase_cmd_permille = 0U;
    g_psfb.phase_target_permille = 0U;
    g_psfb.phase_limit_permille = psfb_phase_limit();
    g_psfb.target_ramped_mv = 0U;
    g_psfb.error_mv = 0;
    s_integrator_q8 = 0;
    s_ramp_target_mv_q = 0U;
    s_ramp_config_target_mv = 0U;
    s_ramp_step_q = 0U;
    s_slew_up_accum_q = 0U;
    s_slew_down_accum_q = 0U;
    s_control_ticks = 0U;
    psfb_set_phase_permille(0U);
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

uint32_t psfb_adc_raw_to_vout_mv(uint16_t adc_raw)
{
    uint32_t adc_mv = ((uint32_t)adc_raw * ADC_REF_MV) / ADC_FULL_SCALE;
    return (adc_mv * VOUT_DIVIDER_GAIN_NUM) / VOUT_DIVIDER_GAIN_DEN;
}

uint32_t psfb_ovp_limit_mv(uint32_t target_vout_mv)
{
    return (target_vout_mv * OVP_MULTIPLIER_NUM) / OVP_MULTIPLIER_DEN;
}

static uint32_t deadtime_ticks_raw(void)
{
    return ((NORMAL_DEADTIME_NS * (TIM1_CLK_HZ / 1000000UL)) + 999UL) / 1000UL;
}

uint8_t psfb_deadtime_dtg(void)
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

void psfb_set_phase_permille(uint16_t phase_permille)
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

    TIM1->CCR1 = 1U;
    TIM1->CCR2 = (uint16_t)right_toggle_tick;
    TIM1->CCR3 = psfb_adc_trigger_tick(right_toggle_tick);
    TIM1->CCR4 = 0U;

    g_psfb.phase_actual_permille = phase_permille;
    g_psfb.phase_limit_permille = phase_limit;
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

    TIM1->BDTR = 0U;
    TIM1->CR1 = 0U;
    TIM1->CR2 = tim1_off_state_bits();
    TIM1->CCER = 0U;
    TIM1->DIER = 0U;
    TIM1->RCR = 0U;
    TIM1->PSC = 0U;

    s_half_period_ticks = (uint16_t)(TIM1_ARR_VALUE + 1UL);
    TIM1->ARR = (uint16_t)TIM1_ARR_VALUE;

    /*
     * True PSFB timing:
     * - CH1/CH1N left leg toggles every half-cycle.
     * - CH2/CH2N right leg toggles after phase_ticks.
     * - At phase = 0 both legs toggle together, so transformer voltage is zero.
     */
    ccmr1 = TIM_CCMR1_OC1PE |
            TIM_CCMR1_OC2PE |
            (3U << TIM_CCMR1_OC1M_Pos) |
            (3U << TIM_CCMR1_OC2M_Pos);
    TIM1->CCMR1 = ccmr1;
    TIM1->CCMR2 = TIM_CCMR2_OC3PE |
                  (3U << TIM_CCMR2_OC3M_Pos);
    TIM1->CCER = TIM1_CCER_OUTPUT_ENABLES | tim1_output_polarity_bits();
    TIM1->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR | psfb_deadtime_dtg();

    psfb_set_phase_permille(PSFB_PHASE_START_PERMILLE);

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
        tim1_enable_outputs_if_allowed();
    }
}

static uint32_t ms_to_control_ticks(uint32_t ms)
{
    return (ms * CONTROL_LOOP_HZ) / 1000UL;
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

static void update_softstart_target(uint32_t target_vout_mv)
{
    uint32_t target_q = target_vout_mv << SOFTSTART_RAMP_Q_SHIFT;

    if (s_ramp_config_target_mv != target_vout_mv) {
        uint32_t ramp_ticks = ms_to_control_ticks(SOFTSTART_TIME_MS);

        if (ramp_ticks == 0UL) {
            ramp_ticks = 1UL;
        }

        s_ramp_step_q = target_q / ramp_ticks;
        if (s_ramp_step_q == 0UL) {
            s_ramp_step_q = 1UL;
        }
        s_ramp_config_target_mv = target_vout_mv;
    }

    if ((g_psfb.vout_mv + VOUT_CONTROL_DEADBAND_MV) >= target_vout_mv) {
        s_ramp_target_mv_q = target_q;
        g_psfb.target_ramped_mv = target_vout_mv;
        return;
    }

    if (s_ramp_target_mv_q >= target_q) {
        g_psfb.target_ramped_mv = target_vout_mv;
        return;
    }

    s_ramp_target_mv_q += s_ramp_step_q;
    if (s_ramp_target_mv_q > target_q) {
        s_ramp_target_mv_q = target_q;
    }

    g_psfb.target_ramped_mv =
        (s_ramp_target_mv_q + ((1UL << SOFTSTART_RAMP_Q_SHIFT) - 1UL)) >>
        SOFTSTART_RAMP_Q_SHIFT;
}

static uint16_t active_slew_up_permille(void)
{
    uint32_t fast_threshold_mv =
        (g_psfb.target_ramped_mv * FAST_CONTROL_THRESHOLD_NUM) /
        FAST_CONTROL_THRESHOLD_DEN;

    if ((g_psfb.target_ramped_mv > 0U) &&
        (g_psfb.vout_mv >= fast_threshold_mv)) {
        return PHASE_SLEW_UP_HIGH_PERMILLE_PER_SEC;
    }

    return PHASE_SLEW_UP_LOW_PERMILLE_PER_SEC;
}

static uint16_t active_slew_down_permille(void)
{
    uint32_t fast_threshold_mv =
        (g_psfb.target_ramped_mv * FAST_CONTROL_THRESHOLD_NUM) /
        FAST_CONTROL_THRESHOLD_DEN;

    if ((g_psfb.target_ramped_mv > 0U) &&
        (g_psfb.vout_mv >= fast_threshold_mv)) {
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

static void update_target_cache(uint32_t target_vout_mv)
{
    if (s_control_target_mv != target_vout_mv) {
        s_control_target_mv = target_vout_mv;
        s_control_ovp_limit_mv = psfb_ovp_limit_mv(target_vout_mv);
    }
}

void psfb_control_step(uint32_t target_vout_mv, uint16_t adc_raw)
{
    uint16_t phase_cmd;
    uint16_t phase_limit;
    uint16_t slew_up;
    uint16_t slew_down;
    int32_t error_mv;
    uint8_t feedback_blank_active;

    if (g_psfb.fault != FAULT_NONE) {
        psfb_emergency_shutdown();
        return;
    }

    if ((target_vout_mv < TARGET_VOUT_MIN_MV) || (target_vout_mv > TARGET_VOUT_MAX_MV)) {
        psfb_latch_fault(FAULT_INVALID_TARGET);
        return;
    }
    update_target_cache(target_vout_mv);

    if (s_control_ticks < 0xFFFFFFFFUL) {
        s_control_ticks++;
    }
    feedback_blank_active =
        (s_control_ticks <= ms_to_control_ticks(FEEDBACK_PROTECTION_BLANKING_MS)) ? 1U : 0U;

    g_psfb.adc_raw = adc_raw;
    if (g_psfb.adc_filtered == 0U) {
        g_psfb.adc_filtered = adc_raw;
    } else {
        g_psfb.adc_filtered = adc_iir_filter(g_psfb.adc_filtered, adc_raw);
    }

    g_psfb.vout_mv = psfb_adc_raw_to_vout_mv(g_psfb.adc_filtered);

    if (g_psfb.adc_filtered > ADC_NEAR_FULL_SCALE_LIMIT) {
        if (++s_adc_full_confirm_count >= ADC_NEAR_FULL_CONFIRM_COUNT) {
            psfb_latch_fault(FAULT_ADC_NEAR_FULL_SCALE);
            return;
        }
    } else {
        s_adc_full_confirm_count = 0U;
    }

    if (g_psfb.vout_mv > s_control_ovp_limit_mv) {
        if (++s_ovp_confirm_count >= OVP_CONFIRM_COUNT) {
            psfb_latch_fault(FAULT_OVERVOLTAGE);
            return;
        }
    } else {
        s_ovp_confirm_count = 0U;
    }

    update_softstart_target(target_vout_mv);
    phase_limit = psfb_phase_limit();
    g_psfb.phase_limit_permille = phase_limit;

    error_mv = (int32_t)g_psfb.target_ramped_mv - (int32_t)g_psfb.vout_mv;
    g_psfb.error_mv = error_mv;

    if ((error_mv > -(int32_t)VOUT_CONTROL_DEADBAND_MV) &&
        (error_mv < (int32_t)VOUT_CONTROL_DEADBAND_MV)) {
        phase_cmd = g_psfb.phase_actual_permille;
    } else {
        phase_cmd = pi_controller(error_mv, phase_limit);
    }

    if ((feedback_blank_active == 0U) &&
        (g_psfb.target_ramped_mv > 5000UL) &&
        (g_psfb.phase_actual_permille > 10U) &&
        (g_psfb.adc_filtered < 20U)) {
        if (++s_feedback_low_ticks > ms_to_control_ticks(FEEDBACK_LOW_TIMEOUT_MS)) {
            psfb_latch_fault(FAULT_FEEDBACK_LOW_OR_DISCONNECTED);
            return;
        }
    } else {
        s_feedback_low_ticks = 0U;
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

    g_psfb.phase_cmd_permille = phase_cmd;
    g_psfb.phase_target_permille = phase_cmd;
    psfb_set_phase_permille(g_psfb.phase_actual_permille);
}
