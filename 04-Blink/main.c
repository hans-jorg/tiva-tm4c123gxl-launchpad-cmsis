/**
 * @file     main.c
 * @brief    Blink LEDs using delays and CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Systick is used t generate delay
 * @note     An GPIO abstraction (API) is used
 * @note     CMSIS library used
 * @note     Uses bit-banding
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
#define LED_NONE     (0)
//@}

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
    gpio->DEN    = LED_ALL;         /* All pins are digital I/O        */

    xdelay(10);

}

/**
 * @brief Write to GPIO Port F
 *
 * @note First, pins specified by zero are zeroed. Then the pins
 *       specified by ones, area set
 *
 * @note Use bit-banding
 *
 * @note GPIOA_Type defined as:
 * typedef struct {
 *   __I  uint32_t  RESERVED0[255];  // Deveria ser BITBAND[255]
 *   __IO uint32_t  DATA;
 *   __IO uint32_t  DIR;
 *    ....
 *    } GPIOA_Type;
 *
 */
void
GPIO_WritePin(uint32_t zeroes, uint32_t ones) {
uint32_t *base;

#ifdef USE_AHB
    base = (uint32_t *) GPIOF_AHB_BASE;
#else
    base = (uint32_t *) GPIOF_BASE;
#endif

    *(base+(zeroes|ones)) = ones;
}

/***************************************************************************
 *                                                                         *
 *              Systick routine                                            *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief Milliseconds counter
 *
 * @note  Overflow after 4294967 seconds, i.e. 49 days, 17 hours, 2 min, 47 sec
 */

volatile uint32_t tick = 0;

/**
 * @brief SysTick Handler
 *
 * @note  Only increments tick
 */

void SysTick_Handler(void) {
    tick++;
}

/**
 * @brief Delay routine
 *
 * @param delay in milliseconds
 */

void Delay(uint32_t delay) {
uint32_t begin = tick;

    while( (tick-begin) < delay ) {}

}

/***************************************************************************
 *                                                                         *
 *              Main procedure                                             *
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

    SysTick_Config(SystemCoreClock/1000);

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
