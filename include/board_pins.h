#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/*
 * STM32F103C6T6 PSFB gate-drive pin map.
 *
 * TIM1 drives the primary bridge, and PA0 is VOUT feedback.
 * PA13/PA14 are left untouched for SWD.
 */

#define PIN_Q1_HIGH_LEFT_PORT      GPIOA
#define PIN_Q1_HIGH_LEFT_PIN       8u      /* PA8 TIM1_CH1 */

#define PIN_Q2_LOW_LEFT_PORT       GPIOB
#define PIN_Q2_LOW_LEFT_PIN        13u     /* PB13 TIM1_CH1N */

#define PIN_Q3_HIGH_RIGHT_PORT     GPIOA
#define PIN_Q3_HIGH_RIGHT_PIN      9u      /* PA9 TIM1_CH2 */

#define PIN_Q4_LOW_RIGHT_PORT      GPIOB
#define PIN_Q4_LOW_RIGHT_PIN       14u     /* PB14 TIM1_CH2N */

#define PIN_VOUT_FB_PORT           GPIOA
#define PIN_VOUT_FB_PIN            0u      /* PA0 ADC1_IN0 */

#define PIN_FAULT_LED_PORT         GPIOC
#define PIN_FAULT_LED_PIN          13u     /* PC13 fault blink LED */
#define PIN_FAULT_LED_ACTIVE_LOW   1u

#endif /* BOARD_PINS_H */
