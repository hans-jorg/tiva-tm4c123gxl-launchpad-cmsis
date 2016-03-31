/**
 * @file     main.c
 * @brief    Blink LEDs using interrupts and CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     The LED interface is implemented as separated files
 * @note     A state machine approach is used
 * @note     Systick is used to generate interrupts
 * @note     All processing in SysTick interrupt
 * @note     CMSIS library used
 *
 **/

#include "TM4C123GH6PM.h"
#include "system_tm4c123.h"
#include "led.h"

/***************************************************************************
 *                                                                         *
 *              State machine                                              *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief State Machine
 */

void StateMachine(void) {
static int state = 0;

    switch(state) {
    case 0:
        LED_Write(LED_ALL, LED_RED);
        state = 1;
        break;
    case 1:
        LED_Write(LED_ALL, LED_GREEN);
        state = 2;
        break;
    case 2:
        LED_Write(LED_ALL, LED_BLUE);
        state = 0;
        break;
    }

}

/**
 * @brief SysTick_Handler
 *
 * @note  Call StateMachine every 1000 interrupts
 */

void SysTick_Handler(void) {
static uint32_t counter = 0;
static volatile uint32_t tick = 0; // overflow after 49 days

    if( counter == 0 ) {
        StateMachine();
        counter = 1000;
    }
    counter--;

    tick++;
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

    LED_Init();

    SysTick_Config(SystemCoreClock/1000);

    while(1) {}

    return 0;
}
