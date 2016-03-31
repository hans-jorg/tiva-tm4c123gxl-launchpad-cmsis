/**
 * @file     led.c
 * @brief    Led control using CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     CMSIS library used
 *
 **/

#include "TM4C123GH6PM.h"
#include "system_tm4c123.h"

#include "led.h"

static void xdelay(unsigned volatile v) {

    while( --v ) {}
}

/***************************************************************************
 *                                                                         *
 *              LED control routines                                      *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief LED_Init
 *
 * @note Initializes all LEDs
 */

void LED_Init(void) {
#ifdef AHB
GPIOA_Type *gpio = GPIOF_AHB;
#else
GPIOA_Type *gpio = GPIOF;
#endif

#ifdef AHB
    SYSCTL->GPIOHBCTL |= LEDBIT(5);
#endif

    SYSCTL->RCGCGPIO  |= LEDBIT(5);        /* Enable clock for Port F */

    /* Pins for led are digital output */
    gpio->DIR    |= LED_ALL;
    gpio->DEN    |= LED_ALL;

    xdelay(10);

}

/**
 * @brief Write to LEDs
 *
 * @note If a LED is in both zeroes and ones, the LED will be set
 */
void LED_Write(uint32_t zeroes, uint32_t ones) {
#ifdef AHB
GPIOA_Type *gpio = GPIOF_AHB;
#else
GPIOA_Type *gpio = GPIOF;
#endif

    gpio->DATA = (gpio->DATA&(~zeroes))|ones;

}

/**
 * @brief Toggle LEDs
 *
 * @note Invert the output of the specified LEDs
 */
void LED_Toggle(uint32_t bits) {
#ifdef AHB
GPIOA_Type *gpio = GPIOF_AHB;
#else
GPIOA_Type *gpio = GPIOF;
#endif

    gpio->DATA = (gpio->DATA^bits);

}
