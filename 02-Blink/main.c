/**
 * @file     main.c
 * @brief    Blink LEDs using counting delays and CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     An GPIO abstraction (API) is used
 * @note     The blinking frequency depends on core frequency
 * @note     Direct access to registers
 * @note     CMSIS library used
 *
 *
 **/

#include "TM4C123GH6PM.h"

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
#define LED_NONE     (0)
//@}

/**
 * Delay unity (value should be adjusted to give 1 ms )
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
 */

void xdelay(unsigned volatile v) {

    while( --v ) {}
}

/**
 * @brief Quick and dirty delay routine
 *
 * It gives approximately 1ms delay at 4 MHz (MSI)
 *
 */
void Delay(uint32_t delay) {
    xdelay(INTERVAL);
}


/***************************************************************************
 *                                                                         *
 *              GPIO control routines                                      *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief Initialization of GPIO Port F
 *
 * @note LED pins set to output
 */
void
GPIO_Init(uint32_t outputs) {
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

    /* Pins for led are digital output */
    gpio->DIR    = outputs;         /* Only specified bits are outputs */
    gpio->DEN    = outputs;         /* All pins are digital I/O        */

    xdelay(10);

}

/**
 * @brief Write to GPIO Port F
 *
 * @note First, pins specified by zero are zeroed. Then the pins
 *       specified by ones, area set
 */
static inline void
GPIO_WritePin(uint32_t zeroes, uint32_t ones) {
GPIOA_Type *gpio;

#ifdef USE_AHB
    gpio = GPIOF_AHB;
    SYSCTL->GPIOHBCTL |= BIT(5);
#else
    gpio = GPIOF;
#endif

    gpio->DATA = (gpio->DATA&~zeroes)|ones;
}


/***************************************************************************
 *                                                                         *
 *              Main routine                                               *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief Main routine
 *
 * Initializes GPIO
 * Blink LEDs
 */

int main(void) {

    GPIO_Init(LED_ALL);

    while(1) {
        /* White */
        GPIO_WritePin(LED_ALL, LED_ALL);
        Delay(1000);

        /* Red */
        GPIO_WritePin(LED_ALL, LED_RED);
        Delay(1000);

        /* Green */
        GPIO_WritePin(LED_ALL, LED_GREEN);
        Delay(1000);

        /* Blue */
        GPIO_WritePin(LED_ALL, LED_BLUE);
        Delay(1000);

        /* Off */
        GPIO_WritePin(LED_ALL, LED_NONE);
        Delay(1000);
    }
    return 0;
}
