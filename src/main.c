/*
 * Change only this value to set output voltage.
 * Allowed range: 24000U to 28000U.
 */
#define TARGET_VOUT_MV 24000U

#include "psfb_control.h"
#include "board_pins.h"

#include "stm32f1xx.h"

static volatile uint32_t g_ms_ticks = 0;
static volatile uint16_t g_adc_dma[ADC_DMA_SAMPLES];

// cppcheck-suppress unusedFunction
void SysTick_Handler(void)
{
    g_ms_ticks++;
}

static void clock_init_72mhz_hse(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
    }

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    RCC->CFGR = RCC_CFGR_PPRE1_DIV2 |
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

static void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_AFIOEN;

    /*
     * External pull-down resistors are required on every isolated gate-driver
     * input. First test with no high DC bus and verify gate-source waveforms.
     */
    gpio_set_cr(GPIOA, PIN_Q1_HIGH_LEFT_PIN, 0xBU);  /* PA8  AF push-pull 50 MHz */
    gpio_set_cr(GPIOA, PIN_Q3_HIGH_RIGHT_PIN, 0xBU); /* PA9  AF push-pull 50 MHz */
    gpio_set_cr(GPIOB, PIN_Q2_LOW_LEFT_PIN, 0xBU);   /* PB13 AF push-pull 50 MHz */
    gpio_set_cr(GPIOB, PIN_Q4_LOW_RIGHT_PIN, 0xBU);  /* PB14 AF push-pull 50 MHz */

    gpio_set_cr(GPIOA, PIN_VOUT_FB_PIN, 0x0U);       /* PA0 analog input */

    /* PB12/TIM1_BKIN reserved for future current-trip hardware. */
    gpio_set_cr(GPIOB, PIN_TIM1_BKIN_PIN, 0x4U);     /* Floating input for now */

    /* PA13/PA14 are untouched for SWD. */
}

static void adc_dma_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    DMA1_Channel1->CCR = 0U;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)g_adc_dma;
    DMA1_Channel1->CNDTR = ADC_DMA_SAMPLES;
    DMA1_Channel1->CCR = DMA_CCR_MINC |
                         DMA_CCR_CIRC |
                         DMA_CCR_PSIZE_0 |
                         DMA_CCR_MSIZE_0;

    ADC1->CR2 = 0U;
    ADC1->SQR1 = 0U;
    ADC1->SQR3 = 0U;
    ADC1->SMPR2 = ADC_SMPR2_SMP0;            /* 239.5 ADC cycles */

    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t i = 0U; i < 10000U; i++) {
    }

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0U) {
    }

    ADC1->CR2 |= ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0U) {
    }

    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_ADON;               /* Start continuous conversions. */
}

static uint16_t adc_dma_average(void)
{
    uint32_t sum = 0U;

    for (uint32_t i = 0U; i < ADC_DMA_SAMPLES; i++) {
        sum += g_adc_dma[i];
    }

    return (uint16_t)((sum + (ADC_DMA_SAMPLES / 2U)) / ADC_DMA_SAMPLES);
}

int main(void)
{
    uint32_t next_control_ms;

    if ((TARGET_VOUT_MV < TARGET_VOUT_MIN_MV) ||
        (TARGET_VOUT_MV > TARGET_VOUT_MAX_MV)) {
        g_psfb.fault = FAULT_INVALID_TARGET;
        while (1) {
        }
    }

    clock_init_72mhz_hse();
    SysTick_Config(TIM1_CLK_HZ / 1000UL);

    gpio_init();
    adc_dma_init();
    psfb_init_timer();

    /*
     * Confirm before applying power:
     * PA8/PB13 complementary with dead time.
     * PA9/PB14 complementary with dead time.
     * Both legs fixed 50% duty.
     * Phase shift changes left/right diagonal overlap.
     * Transformer primary positive/negative pulses are symmetrical.
     * Do not test directly at 500 V.
     */
    for (uint32_t i = 0U; i < 200U; i++) {
        uint16_t raw = adc_dma_average();
        g_psfb.adc_raw = raw;
        g_psfb.adc_filtered = raw;
        g_psfb.vout_mv = ((((uint32_t)raw * ADC_REF_MV) / ADC_FULL_SCALE) *
                          VOUT_FEEDBACK_SCALE_NUM) / VOUT_FEEDBACK_SCALE_DEN;
    }

    psfb_start_outputs();
    next_control_ms = g_ms_ticks;

    while (1) {
        if ((int32_t)(g_ms_ticks - next_control_ms) >= 0) {
            next_control_ms += 1U;
            psfb_control_1khz(TARGET_VOUT_MV, adc_dma_average());
        }
    }
}
