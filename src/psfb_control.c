#include "psfb_control.h"

#include "stm32f1xx.h"

volatile uint16_t g_phase_ticks = 0;
volatile psfb_state_t g_psfb_state = STATE_IDLE;
volatile psfb_fault_t g_psfb_fault = FAULT_NONE;

psfb_config_t g_psfb_config = {
    CONTROL_MODE,
    0u,
    TARGET_VOUT_DEFAULT,
    5.0f,
    MANUAL_EST_OUTPUT_VOLTAGE,
    MANUAL_EST_INPUT_VOLTAGE
};

psfb_runtime_status_t g_psfb_status = {0.0f, 0.0f, 0.0f};

static float s_integrator = 0.0f;
static float s_open_loop_softstart_percent = 0.0f;
static float s_target_softstart_vout = 0.0f;
static uint8_t s_outputs_enabled = 0u;
static uint16_t s_low_feedback_count = 0u;
static uint16_t s_feedback_run_count = 0u;

static void apply_command_percent(float command_percent);

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static float open_loop_max_percent(void)
{
#if TEST_UNLOCK_HIGH_DUTY
    return DUTY_MAX_ABSOLUTE_PERCENT;
#else
    return DUTY_MAX_INITIAL_PERCENT;
#endif
}

static float feedback_max_percent(void)
{
    return DUTY_MAX_ABSOLUTE_PERCENT;
}

static void reset_control_state(void)
{
    s_integrator = 0.0f;
    s_open_loop_softstart_percent = 0.0f;
    s_target_softstart_vout = 0.0f;
    s_low_feedback_count = 0u;
    s_feedback_run_count = 0u;
    g_psfb_status.command_percent = 0.0f;
    g_psfb_status.applied_percent = 0.0f;
    g_psfb_status.ramped_target_vout = 0.0f;
}

uint8_t psfb_deadtime_ns_to_dtg(uint32_t deadtime_ns)
{
    uint32_t ticks = ((deadtime_ns * (SYSCLK_HZ / 1000000u)) + 999u) / 1000u;

    if (ticks > 127u) {
        ticks = 127u;
    }

    return (uint8_t)ticks;
}

uint16_t psfb_percent_to_phase_ticks(float phase_percent)
{
    float ticks;
    uint16_t max_ticks;

    phase_percent = clampf(phase_percent, 0.0f, DUTY_MAX_ABSOLUTE_PERCENT);
    ticks = (phase_percent * 0.01f) * (float)TIM1_PERIOD_TICKS;
    max_ticks = (uint16_t)(((DUTY_MAX_ABSOLUTE_PERCENT * 0.01f) *
                            (float)TIM1_PERIOD_TICKS) + 0.5f);

    if (ticks < 0.0f) {
        return 0u;
    }
    if (ticks > (float)max_ticks) {
        return max_ticks;
    }

    return (uint16_t)(ticks + 0.5f);
}
void psfb_set_phase_ticks(uint16_t ticks)
{
    uint16_t max_ticks = psfb_percent_to_phase_ticks(DUTY_MAX_ABSOLUTE_PERCENT);
    uint16_t base_compare = 1u;
    uint16_t ccr2;

    if (ticks > max_ticks) {
        ticks = max_ticks;
    }

    ccr2 = (uint16_t)(base_compare + ticks);
    if (ccr2 > TIM1_ARR_VALUE) {
        ccr2 = TIM1_ARR_VALUE;
    }

    g_phase_ticks = ticks;
    TIM1->CCR1 = base_compare;
    TIM1->CCR2 = ccr2;
}

static void apply_command_percent(float command_percent)
{
    command_percent = clampf(command_percent, 0.0f, DUTY_MAX_ABSOLUTE_PERCENT);

    psfb_set_phase_ticks(psfb_percent_to_phase_ticks(command_percent));

    g_psfb_status.applied_percent = command_percent;
}

static float estimate_open_loop_voltage_percent(void)
{
    float secondary_per_primary;
    float denominator;
    float duty;

    if ((g_psfb_config.manual_est_input_voltage <= 0.0f) ||
        (PRIMARY_TURNS <= 0.0f) ||
        (SECONDARY_TURNS_TOTAL <= 0.0f) ||
        (g_psfb_config.manual_est_output_voltage < 0.0f)) {
        psfb_disable_outputs(FAULT_INVALID_COMMAND);
        return 0.0f;
    }

    secondary_per_primary = SECONDARY_TURNS_TOTAL / PRIMARY_TURNS;
    denominator = g_psfb_config.manual_est_input_voltage * secondary_per_primary;
    duty = (g_psfb_config.manual_est_output_voltage + RECTIFIER_DIODE_DROP_TOTAL) / denominator;

    return duty * 100.0f;
}

float psfb_get_command_percent(void)
{
    float command = 0.0f;

    if (g_psfb_config.control_mode == CONTROL_MODE_OPEN_LOOP_DUTY) {
        command = g_psfb_config.manual_command_percent;
    } else if (g_psfb_config.control_mode == CONTROL_MODE_OPEN_LOOP_EST_VOLT) {
        command = estimate_open_loop_voltage_percent();
    } else {
        command = feedback_max_percent();
    }

    if (command < 0.0f) {
        psfb_disable_outputs(FAULT_INVALID_COMMAND);
        return 0.0f;
    }

    command = clampf(command, 0.0f, open_loop_max_percent());
    g_psfb_status.command_percent = command;
    return command;
}

void psfb_disable_outputs(psfb_fault_t fault)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    apply_command_percent(0.0f);
    g_psfb_fault = fault;
    g_psfb_state = STATE_FAULT;
    s_outputs_enabled = 0u;
}

// cppcheck-suppress unusedFunction
void psfb_clear_fault(void)
{
    TIM1->SR &= ~(TIM_SR_BIF | TIM_SR_UIF);
    g_psfb_fault = FAULT_NONE;
    g_psfb_state = STATE_IDLE;
    s_outputs_enabled = 0u;
    reset_control_state();
    apply_command_percent(0.0f);
}

// cppcheck-suppress unusedFunction
void psfb_init(void)
{
    uint16_t ccer = 0u;
    uint32_t ccmr1 = 0u;
    uint32_t bdtr;

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->BDTR = 0u;
    TIM1->CR1 = 0u;
    TIM1->CR2 = 0u;
    TIM1->CCER = 0u;
    TIM1->PSC = 0u;
    TIM1->ARR = TIM1_ARR_VALUE;             /* 72 MHz / (2 * 100 kHz) - 1 = 359 */
    TIM1->RCR = 0u;

    TIM1->CR1 = TIM_CR1_ARPE;
    ccmr1 |= (3u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    ccmr1 |= (3u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR1 = ccmr1;
    psfb_set_phase_ticks(0u);

    ccer |= TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
#if INVERT_TIM1_OUTPUT_POLARITY
    ccer |= TIM_CCER_CC1P | TIM_CCER_CC2P;
#endif
#if INVERT_TIM1_N_OUTPUT_POLARITY
    ccer |= TIM_CCER_CC1NP | TIM_CCER_CC2NP;
#endif
    TIM1->CCER = ccer;

    bdtr = TIM_BDTR_OSSI | TIM_BDTR_OSSR | psfb_deadtime_ns_to_dtg(DEADTIME_NS);
#if ENABLE_TIM1_BKIN
    bdtr |= TIM_BDTR_BKE;
#endif
    TIM1->BDTR = bdtr;                      /* MOE intentionally remains 0. */

    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0u;
    TIM1->CR1 |= TIM_CR1_CEN;
}

// cppcheck-suppress unusedFunction
void psfb_start(void)
{
    if (g_psfb_fault != FAULT_NONE) {
        return;
    }

    if ((g_psfb_config.control_mode == CONTROL_MODE_CLOSED_LOOP_FEEDBACK) &&
        g_psfb_config.feedback_enabled) {
        g_psfb_state = STATE_SOFTSTART;
    } else {
        g_psfb_state = STATE_OPEN_LOOP_RAMP;
    }
}

static void enable_outputs_once_ready(void)
{
    if (!s_outputs_enabled) {
        TIM1->BDTR |= TIM_BDTR_MOE;
        s_outputs_enabled = 1u;
    }
}

static void run_open_loop(const psfb_adc_sample_t *adc)
{
    float target;

    (void)adc;

    target = psfb_get_command_percent();
    if (SOFTSTART_TIME_MS == 0u) {
        s_open_loop_softstart_percent = target;
    } else {
        float step_per_ms = (target * (float)CONTROL_LOOP_PERIOD_MS) /
                            (float)SOFTSTART_TIME_MS;
        s_open_loop_softstart_percent += step_per_ms;
        if (s_open_loop_softstart_percent > target) {
            s_open_loop_softstart_percent = target;
        }
    }

    apply_command_percent(s_open_loop_softstart_percent);
    enable_outputs_once_ready();

    if (s_open_loop_softstart_percent >= target) {
        g_psfb_state = STATE_RUN;
    }
}

static void run_closed_loop(const psfb_adc_sample_t *adc)
{
    const float kp = FEEDBACK_KP;
    const float ki = FEEDBACK_KI;
    float max_percent = feedback_max_percent();
    float ovp = g_psfb_config.target_vout * OVP_MULTIPLIER;
    float target_step;
    float error;
    float candidate_integrator;
    float command;

    if (adc == 0) {
        psfb_disable_outputs(FAULT_FEEDBACK_LOW_OR_DISCONNECTED);
        return;
    }

    if (adc->raw > ADC_NEAR_FULL_SCALE_LIMIT) {
        psfb_disable_outputs(FAULT_ADC_NEAR_FULL_SCALE);
        return;
    }

    if (adc->vout > ovp) {
        psfb_disable_outputs(FAULT_OVERVOLTAGE);
        return;
    }

    if (s_feedback_run_count < 0xffffu) {
        s_feedback_run_count++;
    }

    target_step = (g_psfb_config.target_vout * (float)CONTROL_LOOP_PERIOD_MS) /
                  (float)SOFTSTART_TIME_MS;
    s_target_softstart_vout += target_step;
    if (s_target_softstart_vout > g_psfb_config.target_vout) {
        s_target_softstart_vout = g_psfb_config.target_vout;
    }
    g_psfb_status.ramped_target_vout = s_target_softstart_vout;

    if ((s_target_softstart_vout > TARGET_VOUT_MIN) &&
        (adc->vout < (s_target_softstart_vout * VOUT_COLLAPSE_RATIO))) {
        s_integrator = 0.0f;
        command = DUTY_SAFE_LIMIT_PERCENT;
        if (g_psfb_status.applied_percent > command) {
            apply_command_percent(command);
        }
        g_psfb_status.command_percent = command;
        enable_outputs_once_ready();
        return;
    }

    if ((g_psfb_status.applied_percent > 5.0f) &&
        (s_target_softstart_vout > 5.0f) &&
        (adc->raw < 20u) &&
        ((uint32_t)s_feedback_run_count >
         (FEEDBACK_LOW_BLANKING_MS / CONTROL_LOOP_PERIOD_MS))) {
        if (++s_low_feedback_count > 500u) {
            psfb_disable_outputs(FAULT_FEEDBACK_LOW_OR_DISCONNECTED);
            return;
        }
    } else {
        s_low_feedback_count = 0u;
    }

    error = s_target_softstart_vout - adc->vout;
    candidate_integrator = s_integrator + (ki * error);
    command = (kp * error) + candidate_integrator;
    command = clampf(command, 0.0f, max_percent);

    if (((command > 0.0f) && (command < max_percent)) ||
        ((command <= 0.0f) && (error > 0.0f)) ||
        ((command >= max_percent) && (error < 0.0f))) {
        s_integrator = candidate_integrator;
    }

    if (command > (g_psfb_status.applied_percent + DUTY_SLEW_UP_PERCENT_PER_MS)) {
        command = g_psfb_status.applied_percent +
                  (DUTY_SLEW_UP_PERCENT_PER_MS * (float)CONTROL_LOOP_PERIOD_MS);
    }

    g_psfb_status.command_percent = command;
    apply_command_percent(command);
    enable_outputs_once_ready();

    if (s_target_softstart_vout >= g_psfb_config.target_vout) {
        g_psfb_state = STATE_RUN;
    }
}

// cppcheck-suppress unusedFunction
void psfb_control_1khz(const psfb_adc_sample_t *adc)
{
    if (g_psfb_state == STATE_FAULT) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        return;
    }

#if ENABLE_TIM1_BKIN
    if (TIM1->SR & TIM_SR_BIF) {
        psfb_disable_outputs(FAULT_EXTERNAL_BREAK);
        return;
    }
#endif

    if ((g_psfb_config.control_mode == CONTROL_MODE_CLOSED_LOOP_FEEDBACK) &&
        g_psfb_config.feedback_enabled) {
        run_closed_loop(adc);
    } else {
        run_open_loop(adc);
    }
}

// cppcheck-suppress unusedFunction
