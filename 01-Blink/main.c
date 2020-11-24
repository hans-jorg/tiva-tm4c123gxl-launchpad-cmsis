/**
 * @file     main.c
 * @brief    Blink LEDs using counting delays and CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     The blinking frequency depends on core frequency
 * @note     Direct access to registers
 * @note     CMSIS library used
 *
 **/

#include "TM4C123GH6PM.h"
#include "system_tm4c123.h"

/**
 * @brief BIT Macro
 *
 * To avoid magic numbers
 */
#define BIT(N)        (1U<<(N))

/**
 * @brief LEDs are in PortF(3:1)
 */
 //@{
#define LED_RED       BIT(1)
#define LED_BLUE      BIT(2)
#define LED_GREEN     BIT(3)
#define LED_ALL      (LED_RED|LED_BLUE|LED_GREEN)
//@}

/**
 * @brief Blink duration
 */
#define INTERVAL 800000

/**
 * @brief Bus to be used to access GPIO
 *
 * @note AHB: Uses base address 0x4005D000
 *            1 clock access
 *            Corresponding bit in HBCTL must be set
 * @note APB: Uses base address 0x40025000
 *            Slow (2 clock access)
 *            Legacy
 *
 * @note Comment out the following to use APB
 */
#define USE_AHB


/**
 * @brief Quick and dirty delay routine
 *
 */

void xdelay(unsigned volatile v) {

    while( --v ) {}
}

/**
 * @brief Main routine
 *
 * @note Initializes GPIO
 *       Blink LEDs
 */
uint32_t clockfreq = 0;

int main(void) {
GPIOA_Type *gpio;

#ifdef USE_AHB
    gpio = GPIOF_AHB;
    SYSCTL->GPIOHBCTL |= BIT(5);
#else
    gpio = GPIOF;
#endif

    clockfreq = SystemCoreClockGet();

    SystemCoreClockSet(CLOCK_SRC_PLL_PIOSC,5);

    clockfreq = SystemCoreClockGet();

    /* Enable clock for Port F */
    SYSCTL->RCGCGPIO  |= BIT(5);

    /* Pins for led are digital output */
    gpio->DIR    = LED_ALL;
    gpio->DEN    = LED_ALL;

    xdelay(10);

    while(1) {
        /* White */
        // gpio->DATA &= ~LED_ALL;
        gpio->DATA |= LED_ALL;
        xdelay(INTERVAL);

        /* Red */
        gpio->DATA &= ~LED_ALL;
        gpio->DATA |= LED_RED;
        // gpio->DATA = (gpio->DATA&~LED_ALL)|LED_RED;
        xdelay(INTERVAL);

        /* Green */
        gpio->DATA &= ~LED_ALL;
        gpio->DATA |= LED_GREEN;
        // gpio->DATA = (gpio->DATA&~LED_ALL)|LED_GREEN;
        xdelay(INTERVAL);

        /* Blue */
        gpio->DATA &= ~LED_ALL;
        gpio->DATA |= LED_BLUE;
        // gpio->DATA = (gpio->DATA&~LED_ALL)|LED_BLUE;
        xdelay(INTERVAL);

        /* Off */
        gpio->DATA &= ~LED_ALL;
        xdelay(INTERVAL);
    }
    return 0; /* NEVER */
}
