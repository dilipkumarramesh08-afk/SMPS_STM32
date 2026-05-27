/*
 * Change only this value to set output voltage.
 * Allowed range: 12000U to 30000U.
 */
#define TARGET_VOUT_MV 28000U

#include "psfb_control.h"
#include "board_pins.h"

#include "stm32f1xx.h"

static volatile uint32_t g_control_ticks = 0;
static volatile uint16_t g_adc_dma[ADC_DMA_SAMPLES];
static volatile uint16_t g_adc_latest_raw = 0;
static volatile uint8_t g_adc_ready = 0;

void SysTick_Handler(void)
{
    g_control_ticks++;
}

static uint16_t adc_average_window(uint32_t start, uint32_t count)
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

void DMA1_Channel1_IRQHandler(void)
{
    uint32_t isr = DMA1->ISR;

    if ((isr & DMA_ISR_TEIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CTEIF1;
    }

    if ((isr & DMA_ISR_HTIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CHTIF1;
        g_adc_latest_raw = adc_average_window(0U, ADC_DMA_HALF_SAMPLES);
        g_adc_ready = 1U;
    }

    if ((isr & DMA_ISR_TCIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;
        g_adc_latest_raw = adc_average_window(ADC_DMA_HALF_SAMPLES, ADC_DMA_HALF_SAMPLES);
        g_adc_ready = 1U;
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

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_control_ticks;
    uint32_t ticks = ((ms * CONTROL_LOOP_HZ) + 999UL) / 1000UL;

    while ((uint32_t)(g_control_ticks - start) < ticks) {
    }
}

static uint8_t fault_blink_count(fault_t fault)
{
    switch (fault) {
    case FAULT_INVALID_TARGET:
        return 1U;
    case FAULT_OVERVOLTAGE:
        return 2U;
    case FAULT_ADC_NEAR_FULL_SCALE:
        return 3U;
    case FAULT_FEEDBACK_LOW_OR_DISCONNECTED:
        return 4U;
    case FAULT_NONE:
    default:
        return 0U;
    }
}

static void fault_led_blink_code(fault_t fault)
{
    uint8_t count = fault_blink_count(fault);

    if (count == 0U) {
        fault_led_set(0U);
        return;
    }

    for (uint8_t i = 0U; i < count; i++) {
        fault_led_set(1U);
        delay_ms(150U);
        fault_led_set(0U);
        delay_ms(250U);
    }
    delay_ms(1000U);
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
                         DMA_CCR_CIRC |
                         DMA_CCR_HTIE |
                         DMA_CCR_TCIE |
                         DMA_CCR_TEIE;

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

    NVIC_SetPriority(DMA1_Channel1_IRQn, IRQ_PRIORITY_DMA);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->SR = 0U;
    ADC1->CR2 = ADC_CR2_DMA |
                ADC_CR2_EXTSEL_2 |
                ADC_CR2_EXTTRIG |
                ADC_CR2_ADON;
}

static uint16_t adc1_latest_raw(void)
{
    if (g_adc_ready == 0U) {
        return adc_average_window(0U, ADC_DMA_SAMPLES);
    }

    return g_adc_latest_raw;
}

static void latch_startup_fault_if_needed(uint16_t adc_raw)
{
    uint32_t vout_mv;

    if ((TARGET_VOUT_MV < TARGET_VOUT_MIN_MV) ||
        (TARGET_VOUT_MV > TARGET_VOUT_MAX_MV)) {
        psfb_latch_fault(FAULT_INVALID_TARGET);
        return;
    }

    vout_mv = psfb_adc_raw_to_vout_mv(adc_raw);
    g_psfb.adc_raw = adc_raw;
    g_psfb.adc_filtered = adc_raw;
    g_psfb.vout_mv = vout_mv;

    if (adc_raw > ADC_NEAR_FULL_SCALE_LIMIT) {
        psfb_latch_fault(FAULT_ADC_NEAR_FULL_SCALE);
    } else if (vout_mv > psfb_ovp_limit_mv(TARGET_VOUT_MV)) {
        psfb_latch_fault(FAULT_OVERVOLTAGE);
    }
}

int main(void)
{
    uint32_t next_control_tick;

    clock_init_72mhz_hse_pll();
    SysTick_Config(SYSCLK_HZ / CONTROL_LOOP_HZ);
    NVIC_SetPriority(SysTick_IRQn, IRQ_PRIORITY_SYSTICK);

    gpio_init();
    adc1_init();
    psfb_init_timer();
    psfb_set_phase_permille(PSFB_PHASE_START_PERMILLE);

    latch_startup_fault_if_needed(adc1_latest_raw());

    psfb_start_outputs();
    next_control_tick = g_control_ticks;

    while (1) {
        if (g_psfb.fault != FAULT_NONE) {
            fault_led_blink_code(g_psfb.fault);
            continue;
        }

        if ((int32_t)(g_control_ticks - next_control_tick) >= 0) {
            next_control_tick++;
            psfb_control_step(TARGET_VOUT_MV, adc1_latest_raw());
        }
    }
}
