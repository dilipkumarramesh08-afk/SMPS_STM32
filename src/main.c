#include "psfb_control.h"
#include "board_pins.h"

#include "stm32f1xx.h"

/*
 * User startup settings.
 *
 * Keep USER_ENABLE_FEEDBACK_CONTROL = 0 to preserve the current working
 * open-loop/manual mode. In that mode PA0 feedback is not used for regulation.
 *
 * Set USER_ENABLE_FEEDBACK_CONTROL = 1 only after the 100k/10k divider,
 * isolation/grounding, and ADC reading are verified.
 */
#define USER_ENABLE_FEEDBACK_CONTROL      0u
#define USER_TARGET_OUTPUT_VOLTAGE        24.0f
#define USER_OPEN_LOOP_COMMAND_PERCENT    85.0f
#define USER_EST_INPUT_VOLTAGE            90.0f
#define USER_EST_OUTPUT_VOLTAGE           12.0f

static volatile uint32_t g_ms_ticks = 0;
static psfb_adc_sample_t g_adc_sample = {0u, 0.0f, 0.0f, 0.0f};
static uint8_t g_adc_filter_initialized = 0u;

// cppcheck-suppress unusedFunction
void SysTick_Handler(void)
{
    g_ms_ticks++;
}

static uint32_t millis(void)
{
    return g_ms_ticks;
}

static void clock_init(void)
{
#if CLOCK_SOURCE == CLOCK_SOURCE_HSE_8MHZ_PLL_72MHZ
    /* Enable HSE and wait until the 8 MHz crystal/oscillator is stable. */
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0u) {
    }

    /* Flash wait states for 72 MHz and prefetch enabled. */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /*
     * AHB = 72 MHz, APB2 = 72 MHz, APB1 = 36 MHz.
     * ADC prescaler = PCLK2 / 6 = 12 MHz, inside STM32F103 ADC limit.
     * PLL source = HSE, PLL multiplier = x9.
     */
    RCC->CFGR = RCC_CFGR_PPRE1_DIV2 |
                RCC_CFGR_ADCPRE_DIV6 |
                RCC_CFGR_PLLSRC |
                RCC_CFGR_PLLMULL9;
#else
    /* Use internal HSI. PLL input is HSI/2 = 4 MHz. PLL x16 gives 64 MHz. */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0u) {
    }

    /* Flash wait states for 64 MHz and prefetch enabled. */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /*
     * AHB = 64 MHz, APB2 = 64 MHz, APB1 = 32 MHz.
     * ADC prescaler = PCLK2 / 6 = 10.67 MHz, inside STM32F103 ADC limit.
     * PLL source bit left at 0: HSI/2. PLL multiplier = x16.
     */
    RCC->CFGR = RCC_CFGR_PPRE1_DIV2 |
                RCC_CFGR_ADCPRE_DIV6 |
                RCC_CFGR_PLLMULL16;
#endif

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0u) {
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }

    SystemCoreClockUpdate();
}

static void gpio_set_cr(GPIO_TypeDef *gpio, uint32_t pin, uint32_t mode_cnf)
{
    volatile uint32_t *cr;
    uint32_t shift;

    if (pin < 8u) {
        cr = &gpio->CRL;
        shift = pin * 4u;
    } else {
        cr = &gpio->CRH;
        shift = (pin - 8u) * 4u;
    }

    *cr = (*cr & ~(0xFu << shift)) | ((mode_cnf & 0xFu) << shift);
}

static void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_AFIOEN;

    /*
     * PA8/PA9/PB13/PB14: alternate-function push-pull, 50 MHz.
     * External gate-driver input pull-down resistors are mandatory.
     */
    gpio_set_cr(GPIOA, PIN_Q1_HIGH_LEFT_PIN, 0xBu);
    gpio_set_cr(GPIOA, PIN_Q3_HIGH_RIGHT_PIN, 0xBu);
    gpio_set_cr(GPIOB, PIN_Q2_LOW_LEFT_PIN, 0xBu);
    gpio_set_cr(GPIOB, PIN_Q4_LOW_RIGHT_PIN, 0xBu);

    /* PA0 ADC1_IN0: analog input. */
    gpio_set_cr(GPIOA, PIN_VOUT_FB_PIN, 0x0u);

#if ENABLE_TIM1_BKIN
    /*
     * PB12 TIM1_BKIN: input with pull-down. Add external comparator before
     * serious high-power testing; firmware OVP is not cycle-by-cycle current protection.
     */
    gpio_set_cr(GPIOB, PIN_TIM1_BKIN_PIN, 0x8u);
    GPIOB->BRR = (1u << PIN_TIM1_BKIN_PIN);
#endif

    /* Keep SWD pins PA13/PA14 usable; no JTAG remap is changed here. */
}

static void apply_user_startup_settings(void)
{
    g_psfb_config.target_vout = USER_TARGET_OUTPUT_VOLTAGE;
    g_psfb_config.manual_command_percent = USER_OPEN_LOOP_COMMAND_PERCENT;
    g_psfb_config.manual_est_input_voltage = USER_EST_INPUT_VOLTAGE;
    g_psfb_config.manual_est_output_voltage = USER_EST_OUTPUT_VOLTAGE;

#if USER_ENABLE_FEEDBACK_CONTROL
    g_psfb_config.feedback_enabled = 1u;
    g_psfb_config.control_mode = CONTROL_MODE_CLOSED_LOOP_FEEDBACK;
#else
    g_psfb_config.feedback_enabled = 0u;
    g_psfb_config.control_mode = CONTROL_MODE_OPEN_LOOP_DUTY;
#endif
}

static void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC1->CR2 = 0u;
    ADC1->SQR1 = 0u;                         /* One conversion. */
    ADC1->SQR3 = 0u;                         /* Rank 1 = ADC channel 0 / PA0. */
    ADC1->SMPR2 = ADC_SMPR2_SMP0;            /* 239.5 cycles for stable divider reading. */

    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t i = 0u; i < 10000u; i++) {
    }

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL) {
    }

    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL) {
    }
}

static uint16_t adc_read_pa0(void)
{
    ADC1->SQR3 = 0u;
    ADC1->CR2 |= ADC_CR2_ADON;               /* Starts conversion in single-conversion mode. */
    while ((ADC1->SR & ADC_SR_EOC) == 0u) {
    }
    return (uint16_t)ADC1->DR;
}

static void adc_update_filter(void)
{
    uint32_t accum = 0u;
    uint16_t raw;

    /*
     * Feedback filtering:
     * - average several ADC conversions to reduce switching noise,
     * - then apply a low-pass IIR filter at the 1 kHz control-loop rate,
     * - initialize the filter from the first averaged sample so it does not
     *   ramp from artificial zero at startup.
     */
    for (uint32_t i = 0u; i < ADC_OVERSAMPLE_COUNT; i++) {
        accum += adc_read_pa0();
    }
    raw = (uint16_t)((accum + (ADC_OVERSAMPLE_COUNT / 2u)) / ADC_OVERSAMPLE_COUNT);

    g_adc_sample.raw = raw;
    if (!g_adc_filter_initialized) {
        g_adc_sample.filtered = (float)raw;
        g_adc_filter_initialized = 1u;
    } else {
        g_adc_sample.filtered += (((float)raw) - g_adc_sample.filtered) * ADC_IIR_ALPHA;
    }
    g_adc_sample.adc_voltage = (g_adc_sample.filtered * ADC_REF_VOLTAGE) / ADC_FULL_SCALE;
    g_adc_sample.vout = g_adc_sample.adc_voltage * VOUT_FEEDBACK_SCALE;
}

#if ENABLE_IWDG
static void iwdg_init(void)
{
    IWDG->KR = 0x5555u;       /* Enable write access. */
    IWDG->PR = 4u;            /* LSI / 64. */
    IWDG->RLR = 625u;         /* About 1 s at 40 kHz LSI. */
    IWDG->KR = 0xAAAAu;       /* Reload. */
    IWDG->KR = 0xCCCCu;       /* Start. */
}

static void iwdg_kick(void)
{
    IWDG->KR = 0xAAAAu;
}
#endif

int main(void)
{
    uint32_t next_control_ms;

    clock_init();
    SysTick_Config(SYSCLK_HZ / 1000u);

    gpio_init();
    adc_init();
    apply_user_startup_settings();
    psfb_init();

    psfb_set_phase_ticks(0u);
    TIM1->EGR = TIM_EGR_UG;

    for (uint32_t i = 0u; i < ADC_PREFILL_SAMPLES; i++) {
        adc_update_filter();
    }

#if ENABLE_IWDG
    if (RCC->CSR & RCC_CSR_IWDGRSTF) {
        g_psfb_fault = FAULT_WATCHDOG_RESET;
        g_psfb_state = STATE_FAULT;
    }
    RCC->CSR |= RCC_CSR_RMVF;
    iwdg_init();
#endif

    psfb_start();
    next_control_ms = millis();

    while (1) {
        uint32_t now = millis();

        if ((int32_t)(now - next_control_ms) >= 0) {
            next_control_ms += CONTROL_LOOP_PERIOD_MS;

            adc_update_filter();
            psfb_control_1khz(&g_adc_sample);

#if ENABLE_IWDG
            iwdg_kick();
#endif
        }
    }
}
