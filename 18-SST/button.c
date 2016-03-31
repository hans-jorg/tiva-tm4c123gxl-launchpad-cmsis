/**
 * @file     button.c
 * @brief    Routines to access buttons
 * @version  V1.0
 * @date     23/01/2016
 *
 **/

#include "TM4C123GH6PM.h"
#include "button.h"

/**
 * @brief Quick and dirty delay routine
 */

static void xdelay(unsigned volatile v) {

    while( --v ) {}
}

/**
 * @brief Initializes button pins in GPIO Port F
 */
void
Button_Init(void) {
GPIOA_Type *gpio;

#ifdef USE_AHB
    gpio = GPIOF_AHB;
    /* Enable AHB access for Port F  */
    SYSCTL->GPIOHBCTL |= BIT(5);
#else
    gpio = GPIOF;
#endif

    /* Enable clock for Port F         */
    SYSCTL->RCGCGPIO  |= BIT(5);

    /* To modify PF0, it is necessary to unlock it. See 10.1 of Datasheet */
    if( (SW1|SW2)&1 ) {
        gpio->LOCK = 0x4C4F434B;                // unlock to set PF0
        *(uint32_t * )(&(gpio->CR))   = 0x0;    // CR is marked read-only
    }

    /* Pins for led are digital output */
    gpio->DIR    &= ~SW_ALL;         /* Button pins are inputs */
    gpio->DEN    |= SW_ALL;          /* Button pins are digital I/O */

    xdelay(10);
    if( (SW1|SW2)&1 ) {
        gpio->LOCK = 0x0;                       // lock again
        *(uint32_t * )(&(gpio->CR))   = 0x0;    // CR is marked read-only
    }

}

/**
 * @brief Read button pins in GPIO Port F
 */
uint32_t
Button_Read(void) {
GPIOA_Type *gpio;

#ifdef USE_AHB
    gpio = GPIOF_AHB;
#else
    gpio = GPIOF;
#endif
    return gpio->DATA;
}
