/*
 * Change only this value to set output voltage.
 * Allowed range: 12000U to 28000U.
 */
#define TARGET_VOUT_MV 26000U

#include "hbridge_control.h"
#include "board_pins.h"

#include "stm32f1xx.h"

static volatile uint32_t g_control_ticks = 0;
static volatile uint16_t g_adc_dma[ADC_DMA_SAMPLES];

void SysTick_Handler(void)
{
    g_control_ticks++;
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

    while ((uint32_t)(g_control_ticks - start) < ms) {
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
    case FAULT_SOFTWARE_LIMIT:
        return 5U;
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
                         DMA_CCR_CIRC;

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC1->SQR1 = 0U;
    ADC1->SQR3 = 0U;
    ADC1->SMPR2 = ADC_SMPR2_SMP0;

    ADC1->CR2 = ADC_CR2_ADON;
    for (volatile uint32_t i = 0U; i < 10000U; i++) {
    }

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0U) {
    }

    ADC1->CR2 |= ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0U) {
    }

    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_CONT | ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_ADON;       /* Start continuous conversions. */

    for (volatile uint32_t i = 0U; i < 10000U; i++) {
    }
}

static uint16_t adc1_latest_raw(void)
{
    uint32_t sum = 0U;

    for (uint32_t i = 0U; i < ADC_DMA_SAMPLES; i++) {
        sum += g_adc_dma[i];
    }

    return (uint16_t)((sum + (ADC_DMA_SAMPLES / 2U)) / ADC_DMA_SAMPLES);
}

static void latch_startup_fault_if_needed(uint16_t adc_raw)
{
    uint32_t vout_mv;

    if ((TARGET_VOUT_MV < TARGET_VOUT_MIN_MV) ||
        (TARGET_VOUT_MV > TARGET_VOUT_MAX_MV)) {
        hbridge_latch_fault(FAULT_INVALID_TARGET);
        return;
    }

    vout_mv = hbridge_adc_raw_to_vout_mv(adc_raw);
    g_hbridge.adc_raw = adc_raw;
    g_hbridge.adc_filtered = adc_raw;
    g_hbridge.vout_mv = vout_mv;

    if (adc_raw > ADC_NEAR_FULL_SCALE_LIMIT) {
        hbridge_latch_fault(FAULT_ADC_NEAR_FULL_SCALE);
    } else if (vout_mv > hbridge_ovp_limit_mv(TARGET_VOUT_MV)) {
        hbridge_latch_fault(FAULT_OVERVOLTAGE);
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
    hbridge_init_timer();
    hbridge_set_duty_permille(DUTY_START_PERMILLE);

    for (volatile uint32_t i = 0U; i < 50000U; i++) {
    }
    latch_startup_fault_if_needed(adc1_latest_raw());

    hbridge_start_outputs();
    next_control_tick = g_control_ticks;

    while (1) {
        if (g_hbridge.fault != FAULT_NONE) {
            fault_led_blink_code(g_hbridge.fault);
            continue;
        }

        if ((int32_t)(g_control_ticks - next_control_tick) >= 0) {
            next_control_tick++;
            hbridge_control_step(TARGET_VOUT_MV, adc1_latest_raw());
        }
    }
}
