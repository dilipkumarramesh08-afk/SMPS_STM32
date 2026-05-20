#include "hbridge_control.h"

#include "stm32f1xx.h"

volatile hbridge_status_t g_hbridge = {0};

enum {
    TIM1_CCER_OUTPUT_ENABLES = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                               TIM_CCER_CC2E | TIM_CCER_CC2NE,
    TIM1_CCER_POLARITY_BITS = TIM_CCER_CC1P | TIM_CCER_CC1NP |
                              TIM_CCER_CC2P | TIM_CCER_CC2NP
};

static uint16_t s_period_ticks = 0;
static volatile uint16_t s_pulse_ticks = 0;
static volatile uint8_t s_outputs_armed = 0;
static int32_t s_integrator_q8 = 0;
static uint32_t s_ramp_target_mv_q = 0;
static uint32_t s_ramp_config_target_mv = 0;
static uint32_t s_ramp_step_q = 0;
static uint32_t s_slew_up_accum_q = 0;
static uint32_t s_slew_down_accum_q = 0;
static uint32_t s_control_ms = 0;
static uint32_t s_control_target_mv = 0;
static uint32_t s_control_ovp_limit_mv = 0;
static uint16_t s_feedback_low_ms = 0;
static uint8_t s_ovp_confirm_count = 0;

static inline void hbridge_emergency_shutdown(void)
{
    s_outputs_armed = 0U;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    g_hbridge.duty_actual_permille = 0U;
    g_hbridge.duty_cmd_permille = 0U;
    g_hbridge.duty_target_permille = 0U;
    g_hbridge.target_ramped_mv = 0U;
    g_hbridge.error_mv = 0;
    s_integrator_q8 = 0;
    s_ramp_target_mv_q = 0U;
    s_ramp_config_target_mv = 0U;
    s_ramp_step_q = 0U;
    s_slew_up_accum_q = 0U;
    s_slew_down_accum_q = 0U;
    s_control_ms = 0U;
    hbridge_set_duty_permille(0U);
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

static void tim1_force_outputs_idle(void)
{
    s_outputs_armed = 0U;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

static void tim1_arm_outputs_if_allowed(uint16_t duty_permille)
{
    if ((duty_permille > 0U) && (g_hbridge.fault == FAULT_NONE)) {
        s_outputs_armed = 1U;
    }
}

static void tim1_set_oc_modes(uint32_t oc1_mode, uint32_t oc2_mode)
{
    TIM1->CCMR1 = (TIM1->CCMR1 &
                   ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk)) |
                  ((oc1_mode << TIM_CCMR1_OC1M_Pos) |
                   (oc2_mode << TIM_CCMR1_OC2M_Pos));
}

static void hbridge_all_off(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

static void hbridge_positive_on(void)
{
    if ((s_outputs_armed != 0U) && (g_hbridge.fault == FAULT_NONE)) {
        hbridge_all_off();
        tim1_set_oc_modes(5U, 4U);   /* CH1 active, CH2 inactive -> Q1 + Q4. */
        TIM1->BDTR |= TIM_BDTR_MOE;
    }
}

static void hbridge_negative_on(void)
{
    if ((s_outputs_armed != 0U) && (g_hbridge.fault == FAULT_NONE)) {
        hbridge_all_off();
        tim1_set_oc_modes(4U, 5U);   /* CH1 inactive, CH2 active -> Q2 + Q3. */
        TIM1->BDTR |= TIM_BDTR_MOE;
    }
}

uint32_t hbridge_adc_raw_to_vout_mv(uint16_t adc_raw)
{
    uint32_t adc_mv = ((uint32_t)adc_raw * ADC_REF_MV) / ADC_FULL_SCALE;
    return (adc_mv * VOUT_DIVIDER_GAIN_NUM) / VOUT_DIVIDER_GAIN_DEN;
}

uint32_t hbridge_ovp_limit_mv(uint32_t target_vout_mv)
{
    return (target_vout_mv * OVP_MULTIPLIER_NUM) / OVP_MULTIPLIER_DEN;
}

static uint32_t deadtime_ticks_raw(void)
{
    return ((NORMAL_DEADTIME_NS * (TIM1_CLK_HZ / 1000000UL)) + 999UL) / 1000UL;
}

uint8_t hbridge_deadtime_dtg(void)
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

void hbridge_set_duty_permille(uint16_t duty_permille)
{
    uint32_t pulse_permille;
    uint32_t pulse_ticks;
    uint32_t half_period;
    uint32_t negative_off_edge;

    if (duty_permille > DUTY_MAX_PERMILLE) {
        duty_permille = DUTY_MAX_PERMILLE;
    }

    if (duty_permille == 0U) {
        tim1_force_outputs_idle();
        s_pulse_ticks = 0U;
        TIM1->CCR1 = 0U;
        TIM1->CCR2 = 0U;
        TIM1->CCR3 = 0U;
        TIM1->CCR4 = 0U;
        g_hbridge.duty_actual_permille = 0U;
        return;
    }

    /*
     * duty_permille is total bipolar active duty.
     * +VIN and -VIN pulse widths are each duty/2.
     */
    pulse_permille = (uint32_t)duty_permille / 2UL;
    pulse_ticks = (pulse_permille * s_period_ticks) / 1000UL;
    half_period = (uint32_t)s_period_ticks / 2UL;
    if (pulse_ticks >= half_period) {
        pulse_ticks = half_period - 1UL;
    }

    negative_off_edge = half_period + pulse_ticks;
    if (negative_off_edge > TIM1->ARR) {
        negative_off_edge = TIM1->ARR;
    }

    s_pulse_ticks = (uint16_t)pulse_ticks;
    TIM1->CCR1 = (uint16_t)pulse_ticks;
    TIM1->CCR2 = (uint16_t)half_period;
    TIM1->CCR3 = 0U;
    TIM1->CCR4 = (uint16_t)negative_off_edge;
    g_hbridge.duty_actual_permille = duty_permille;
    tim1_arm_outputs_if_allowed(duty_permille);
}

void hbridge_latch_fault(fault_t fault)
{
    if (g_hbridge.fault == FAULT_NONE) {
        g_hbridge.fault = fault;
    }
    hbridge_emergency_shutdown();
}

void hbridge_init_timer(void)
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

    s_period_ticks = (uint16_t)(TIM1_ARR_VALUE + 1UL);
    TIM1->ARR = (uint16_t)TIM1_ARR_VALUE;

    /*
     * Scheduled H-bridge states:
     * update: Q1 + Q4 positive pulse
     * CC1:    all off
     * CC2:    Q2 + Q3 negative pulse
     * CC4:    all off
     */
    ccmr1 = (4U << TIM_CCMR1_OC1M_Pos) |
            (4U << TIM_CCMR1_OC2M_Pos);
    TIM1->CCMR1 = ccmr1;
    TIM1->CCMR2 = 0U;
    TIM1->CCER = TIM1_CCER_OUTPUT_ENABLES | tim1_output_polarity_bits();
    TIM1->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR | hbridge_deadtime_dtg();

    hbridge_set_duty_permille(DUTY_START_PERMILLE);

    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;
    TIM1->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC4IE;
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, IRQ_PRIORITY_TIM1);
    NVIC_SetPriority(TIM1_CC_IRQn, IRQ_PRIORITY_TIM1);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void hbridge_start_outputs(void)
{
    tim1_arm_outputs_if_allowed(g_hbridge.duty_actual_permille);
}

void TIM1_UP_IRQHandler(void)
{
    if ((TIM1->SR & TIM_SR_UIF) != 0U) {
        TIM1->SR &= ~TIM_SR_UIF;
        if (s_pulse_ticks > 0U) {
            hbridge_positive_on();
        } else {
            hbridge_all_off();
        }
    }
}

void TIM1_CC_IRQHandler(void)
{
    uint32_t sr = TIM1->SR;

    if ((sr & TIM_SR_CC1IF) != 0U) {
        TIM1->SR &= ~TIM_SR_CC1IF;
        hbridge_all_off();
    }

    if ((sr & TIM_SR_CC2IF) != 0U) {
        TIM1->SR &= ~TIM_SR_CC2IF;
        if (s_pulse_ticks > 0U) {
            hbridge_negative_on();
        } else {
            hbridge_all_off();
        }
    }

    if ((sr & TIM_SR_CC4IF) != 0U) {
        TIM1->SR &= ~TIM_SR_CC4IF;
        hbridge_all_off();
    }
}

static uint32_t ms_to_control_ticks(uint32_t ms)
{
    return (ms * CONTROL_LOOP_HZ) / 1000UL;
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

    if ((g_hbridge.vout_mv + VOUT_CONTROL_DEADBAND_MV) >= target_vout_mv) {
        s_ramp_target_mv_q = target_q;
        g_hbridge.target_ramped_mv = target_vout_mv;
        return;
    }

    if (s_ramp_target_mv_q >= target_q) {
        g_hbridge.target_ramped_mv = target_vout_mv;
        return;
    }

    s_ramp_target_mv_q += s_ramp_step_q;
    if (s_ramp_target_mv_q > target_q) {
        s_ramp_target_mv_q = target_q;
    }

    g_hbridge.target_ramped_mv =
        (s_ramp_target_mv_q + ((1UL << SOFTSTART_RAMP_Q_SHIFT) - 1UL)) >>
        SOFTSTART_RAMP_Q_SHIFT;
}

static uint16_t active_slew_up_permille(void)
{
    uint32_t fast_threshold_mv =
        (g_hbridge.target_ramped_mv * FAST_CONTROL_THRESHOLD_NUM) /
        FAST_CONTROL_THRESHOLD_DEN;

    if ((g_hbridge.target_ramped_mv > 0U) &&
        (g_hbridge.vout_mv >= fast_threshold_mv)) {
        return DUTY_SLEW_UP_HIGH_PERMILLE_PER_SEC;
    }

    return DUTY_SLEW_UP_LOW_PERMILLE_PER_SEC;
}

static uint16_t active_slew_down_permille(void)
{
    uint32_t fast_threshold_mv =
        (g_hbridge.target_ramped_mv * FAST_CONTROL_THRESHOLD_NUM) /
        FAST_CONTROL_THRESHOLD_DEN;

    if ((g_hbridge.target_ramped_mv > 0U) &&
        (g_hbridge.vout_mv >= fast_threshold_mv)) {
        return DUTY_SLEW_DOWN_HIGH_PERMILLE_PER_SEC;
    }

    return DUTY_SLEW_DOWN_LOW_PERMILLE_PER_SEC;
}

static uint16_t slew_steps_from_rate(uint16_t permille_per_sec, uint32_t *accum_q)
{
    uint32_t step_q =
        ((uint32_t)permille_per_sec << DUTY_SLEW_Q_SHIFT) /
        CONTROL_LOOP_HZ;
    uint16_t steps;

    *accum_q += step_q;
    steps = (uint16_t)(*accum_q >> DUTY_SLEW_Q_SHIFT);
    *accum_q &= (1UL << DUTY_SLEW_Q_SHIFT) - 1UL;

    return steps;
}

static void update_target_cache(uint32_t target_vout_mv)
{
    if (s_control_target_mv != target_vout_mv) {
        s_control_target_mv = target_vout_mv;
        s_control_ovp_limit_mv = hbridge_ovp_limit_mv(target_vout_mv);
    }
}

void hbridge_control_step(uint32_t target_vout_mv, uint16_t adc_raw)
{
    uint16_t duty_cmd;
    uint16_t slew_up;
    uint16_t slew_down;
    int32_t error_mv;
    uint8_t feedback_blank_active;

    if (g_hbridge.fault != FAULT_NONE) {
        hbridge_emergency_shutdown();
        return;
    }

    if ((target_vout_mv < TARGET_VOUT_MIN_MV) || (target_vout_mv > TARGET_VOUT_MAX_MV)) {
        hbridge_latch_fault(FAULT_INVALID_TARGET);
        return;
    }
    update_target_cache(target_vout_mv);

    if (s_control_ms < 0xFFFFFFFFUL) {
        s_control_ms++;
    }
    feedback_blank_active =
        (s_control_ms <= ms_to_control_ticks(FEEDBACK_PROTECTION_BLANKING_MS)) ? 1U : 0U;

    g_hbridge.adc_raw = adc_raw;
    if (g_hbridge.adc_filtered == 0U) {
        g_hbridge.adc_filtered = adc_raw;
    } else {
        int32_t delta = (int32_t)adc_raw - (int32_t)g_hbridge.adc_filtered;
        g_hbridge.adc_filtered = (uint16_t)((int32_t)g_hbridge.adc_filtered +
                                            (delta >> ADC_FILTER_SHIFT));
    }

    g_hbridge.vout_mv = hbridge_adc_raw_to_vout_mv(g_hbridge.adc_filtered);

    if (g_hbridge.adc_filtered > ADC_NEAR_FULL_SCALE_LIMIT) {
        hbridge_latch_fault(FAULT_ADC_NEAR_FULL_SCALE);
        return;
    }

    if (g_hbridge.vout_mv > s_control_ovp_limit_mv) {
        if (++s_ovp_confirm_count >= OVP_CONFIRM_COUNT) {
            hbridge_latch_fault(FAULT_OVERVOLTAGE);
            return;
        }
    } else {
        s_ovp_confirm_count = 0U;
    }

    update_softstart_target(target_vout_mv);

    error_mv = (int32_t)g_hbridge.target_ramped_mv - (int32_t)g_hbridge.vout_mv;
    g_hbridge.error_mv = error_mv;

    if ((error_mv > -(int32_t)VOUT_CONTROL_DEADBAND_MV) &&
        (error_mv < (int32_t)VOUT_CONTROL_DEADBAND_MV)) {
        duty_cmd = g_hbridge.duty_actual_permille;
    } else {
        duty_cmd = pi_controller(error_mv, DUTY_MAX_PERMILLE);
    }

    if ((feedback_blank_active == 0U) &&
        (g_hbridge.target_ramped_mv > 5000UL) &&
        (g_hbridge.duty_actual_permille > 50U) &&
        (g_hbridge.adc_filtered < 20U)) {
        if (++s_feedback_low_ms > ms_to_control_ticks(FEEDBACK_LOW_TIMEOUT_MS)) {
            hbridge_latch_fault(FAULT_FEEDBACK_LOW_OR_DISCONNECTED);
            return;
        }
    } else {
        s_feedback_low_ms = 0U;
    }

    if (duty_cmd > DUTY_MAX_PERMILLE) {
        duty_cmd = DUTY_MAX_PERMILLE;
    }

    if (duty_cmd > g_hbridge.duty_actual_permille) {
        slew_up = slew_steps_from_rate(active_slew_up_permille(), &s_slew_up_accum_q);
        s_slew_down_accum_q = 0U;
        uint32_t next = (uint32_t)g_hbridge.duty_actual_permille + slew_up;
        g_hbridge.duty_actual_permille = (next > duty_cmd) ? duty_cmd : (uint16_t)next;
    } else if (duty_cmd < g_hbridge.duty_actual_permille) {
        uint16_t current = g_hbridge.duty_actual_permille;
        slew_down = slew_steps_from_rate(active_slew_down_permille(), &s_slew_down_accum_q);
        s_slew_up_accum_q = 0U;
        g_hbridge.duty_actual_permille =
            ((current - duty_cmd) > slew_down) ?
            (uint16_t)(current - slew_down) : duty_cmd;
    } else {
        s_slew_up_accum_q = 0U;
        s_slew_down_accum_q = 0U;
        g_hbridge.duty_actual_permille = duty_cmd;
    }

    g_hbridge.duty_cmd_permille = duty_cmd;
    g_hbridge.duty_target_permille = duty_cmd;
    hbridge_set_duty_permille(g_hbridge.duty_actual_permille);
}
