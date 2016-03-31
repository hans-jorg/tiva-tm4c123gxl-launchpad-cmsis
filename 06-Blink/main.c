/**
 * @file     main.c
 * @brief    Blink LEDs using interrupts and CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     The GPIO interface is implemented as separated files
 * @note     A state machine approach is used
 * @note     Systick is used to generate interrupts
 * @note     All processing in SysTick interrupt
 * @note     CMSIS library used
 * @note     Uses bit-banding
 *
 **/

#include "TM4C123GH6PM.h"
#include "system_tm4c123.h"
#include "gpio.h"

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


/***************************************************************************
 *                                                                         *
 *              State Machine routine                                      *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief GPIO to be used on StateMachine
 *
 * @note  Initialized in main
 *
 */
static GPIOA_Type *gpio = 0;

/**
 * @brief State Machine
 */

void StateMachine(void) {
static int state = 0;

    switch(state) {
    case 0:
        GPIO_WritePort(gpio,LED_ALL, LED_RED);
        state = 1;
        break;
    case 1:
        GPIO_WritePort(gpio,LED_ALL, LED_GREEN);
        state = 2;
        break;
    case 2:
        GPIO_WritePort(gpio,LED_ALL, LED_BLUE);
        state = 0;
        break;
    }
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
 * @brief SysTick_Handler
 *
 * @note  Call StateMachine every 1000 interrupts
 */

void SysTick_Handler(void) {
static uint32_t counter;

    if( counter == 0 ) {
        StateMachine();
        counter = 1000;
    }
    counter--;

    tick++; // increment milliseconds counter
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
 * Set SysTick Timer
 * Do nothing
 */


int main(void) {

#ifdef AHB
    gpio = GPIO_Init(GPIOF_AHB_BASE,0,LED_ALL);
#else
    gpio = GPIO_Init(GPIOF_BASE,0,LED_ALL);
#endif

    SysTick_Config(SystemCoreClock/1000);

    while(1) {}

    return 0;
}
