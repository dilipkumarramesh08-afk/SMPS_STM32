#include "psfb_control.h"
#include "board_pins.h"

#include "stm32f1xx.h"

static volatile uint16_t g_adc_dma[ADC_DMA_SAMPLES];
static volatile uint16_t g_startup_adc_raw = 0;

static uint16_t adc_opto_window_sample(uint32_t start, uint32_t count)
{
    uint32_t sum = 0U;
    uint16_t min_raw = 0xFFFFU;
    uint16_t max_raw = 0U;

    for (uint32_t i = start; i < (start + count); i++) {
        uint16_t sample = g_adc_dma[i];

        sum += sample;
        if (sample < min_raw) {
            min_raw = sample;
        }
        if (sample > max_raw) {
            max_raw = sample;
        }
    }

    if (count > 2U) {
        sum -= min_raw;
        sum -= max_raw;
        count -= 2U;
    }

    return (uint16_t)((sum + (count / 2U)) / count);
}

void TIM1_UP_IRQHandler(void)
{
    static uint16_t control_divider = 0U;

    if ((TIM1->SR & TIM_SR_UIF) != 0U) {
        TIM1->SR &= (uint16_t)~TIM_SR_UIF;

        control_divider++;
        if (control_divider >= CONTROL_LOOP_FSW_DIVIDER) {
            control_divider = 0U;
            psfb_control_step(adc_opto_window_sample(0U, ADC_DMA_SAMPLES));
        }
    }
}

static void clock_init_72mhz_hse_pll(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
    }

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    RCC->CFGR = RCC_CFGR_PPRE1_DIV2 |
                RCC_CFGR_PPRE2_DIV1 |
                RCC_CFGR_ADCPRE_DIV6 |
                RCC_CFGR_PLLSRC |
                RCC_CFGR_PLLMULL9;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }

    SystemCoreClockUpdate();
}

static void gpio_set_inactive(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio->BSRR = (1UL << (pin + 16U));
}

static void gpio_set_cr(GPIO_TypeDef *gpio, uint32_t pin, uint32_t mode_cnf)
{
    volatile uint32_t *cr;
    uint32_t shift;

    if (pin < 8U) {
        cr = &gpio->CRL;
        shift = pin * 4U;
    } else {
        cr = &gpio->CRH;
        shift = (pin - 8U) * 4U;
    }

    *cr = (*cr & ~(0xFUL << shift)) | ((mode_cnf & 0xFUL) << shift);
}

static void gpio_set_output_inactive(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio_set_inactive(gpio, pin);
    gpio_set_cr(gpio, pin, 0x2U);
}

static void fault_led_set(uint8_t on)
{
#if PIN_FAULT_LED_ACTIVE_LOW
    if (on != 0U) {
        PIN_FAULT_LED_PORT->BSRR = (1UL << (PIN_FAULT_LED_PIN + 16U));
    } else {
        PIN_FAULT_LED_PORT->BSRR = (1UL << PIN_FAULT_LED_PIN);
    }
#else
    if (on != 0U) {
        PIN_FAULT_LED_PORT->BSRR = (1UL << PIN_FAULT_LED_PIN);
    } else {
        PIN_FAULT_LED_PORT->BSRR = (1UL << (PIN_FAULT_LED_PIN + 16U));
    }
#endif
}

static void gpio_set_af(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio_set_cr(gpio, pin, 0xBU);
}

static void gpio_set_analog(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio_set_cr(gpio, pin, 0x0U);
}

static void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_AFIOEN;
    (void)RCC->APB2ENR;

    gpio_set_output_inactive(PIN_Q1_HIGH_LEFT_PORT, PIN_Q1_HIGH_LEFT_PIN);
    gpio_set_output_inactive(PIN_Q2_LOW_LEFT_PORT, PIN_Q2_LOW_LEFT_PIN);
    gpio_set_output_inactive(PIN_Q3_HIGH_RIGHT_PORT, PIN_Q3_HIGH_RIGHT_PIN);
    gpio_set_output_inactive(PIN_Q4_LOW_RIGHT_PORT, PIN_Q4_LOW_RIGHT_PIN);
    gpio_set_output_inactive(PIN_FAULT_LED_PORT, PIN_FAULT_LED_PIN);
    fault_led_set(0U);

    gpio_set_af(PIN_Q1_HIGH_LEFT_PORT, PIN_Q1_HIGH_LEFT_PIN);
    gpio_set_af(PIN_Q2_LOW_LEFT_PORT, PIN_Q2_LOW_LEFT_PIN);
    gpio_set_af(PIN_Q3_HIGH_RIGHT_PORT, PIN_Q3_HIGH_RIGHT_PIN);
    gpio_set_af(PIN_Q4_LOW_RIGHT_PORT, PIN_Q4_LOW_RIGHT_PIN);
    gpio_set_analog(PIN_VOUT_FB_PORT, PIN_VOUT_FB_PIN);
}

static void adc1_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->AHBENR;
    (void)RCC->APB2ENR;

    DMA1_Channel1->CCR = 0U;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)g_adc_dma;
    DMA1_Channel1->CNDTR = ADC_DMA_SAMPLES;
    DMA1_Channel1->CCR = DMA_CCR_PL_1 |
                         DMA_CCR_MSIZE_0 |
                         DMA_CCR_PSIZE_0 |
                         DMA_CCR_MINC |
                         DMA_CCR_CIRC;

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC1->SQR1 = 0U;
    ADC1->SQR3 = 0U;
    ADC1->SMPR2 = ADC_SMPR2_CH0_55_5_CYCLES;

    ADC1->CR2 = ADC_CR2_ADON;

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0U) {
    }

    ADC1->CR2 |= ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0U) {
    }

    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->SR = 0U;
    ADC1->CR2 = ADC_CR2_DMA |
                ADC_CR2_EXTSEL_2 |
                ADC_CR2_EXTTRIG |
                ADC_CR2_ADON;
    g_startup_adc_raw = adc_opto_window_sample(0U, ADC_DMA_SAMPLES);
}

static void control_interrupt_init(void)
{
    NVIC_SetPriority(TIM1_UP_IRQn, IRQ_PRIORITY_CONTROL);
    NVIC_EnableIRQ(TIM1_UP_IRQn);
}

static void control_interrupt_start(void)
{
    TIM1->SR &= (uint16_t)~TIM_SR_UIF;
    TIM1->DIER |= TIM_DIER_UIE;
}

static void latch_startup_fault_if_needed(uint16_t adc_raw)
{
    g_psfb.adc_raw = adc_raw;
    g_psfb.adc_filtered = adc_raw;

    if ((OPTO_TARGET_FEEDBACK_RAW < 200U) ||
        (OPTO_TARGET_FEEDBACK_RAW > 3800U)) {
        psfb_latch_fault(FAULT_INVALID_CONFIG);
    }
}

int main(void)
{
    clock_init_72mhz_hse_pll();

    gpio_init();
    adc1_init();
    psfb_init_timer();
    control_interrupt_init();
    psfb_set_phase_permille(PSFB_PHASE_START_PERMILLE);

    latch_startup_fault_if_needed(g_startup_adc_raw);

    psfb_start_outputs();
    control_interrupt_start();

    while (1) {
        fault_led_set(g_psfb.fault != FAULT_NONE);
    }
}
