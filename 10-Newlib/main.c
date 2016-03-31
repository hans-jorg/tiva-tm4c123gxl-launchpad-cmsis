/**
 * @file     main.c
 * @brief    Blink LEDs using counting delays controlled by buttons (switches)
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Newlib is used
 * @note     A simple interface for UART is implemented
 * @note     The LED interface is implemented as separated files
 * @note     A state machine approach is used
 * @note     Systick is used to generate interrupts
 * @note     All processing in SysTick interrupt
 * @note     CMSIS library used
 *
 **/

#include <stdlib.h>
#include <stdio.h>

#include "TM4C123GH6PM.h"

#include "tm4c123gh6pm-fields.h"

#include "system_tm4c123.h"
#include "led.h"
#include "uart.h"

/***************************************************************************
 *                                                                         *
 *              SysTick control routines                                   *
 *                                                                         *
 ***************************************************************************/


/**
 * @brief xdelay
 *
 * @param v delay in SysTick frequency
 * @note  Arithmetic module 24 because SysTick is 24 bit counter
 *
 */

void xdelay(unsigned volatile v) {
uint32_t begin = SysTick->VAL;

    while( ((SysTick->VAL - begin)&SysTick_VAL_CURRENT_Msk) < v ) {}
}


/**
 * @brief SysTick Interrupt Handler
 *
 * @note Just increments tick
 * @note tick must be volatile
 */
//@{
volatile uint32_t tick = 0;

void SysTick_Handler(void) {
    tick++;

}
//@}

/**
 * @brief Delay in milliseconds using SysTick
 *
 * @param delay in milliseconds
 */

void Delay(uint32_t delay) {
uint32_t begin = tick;

    while( (tick-begin)<delay ) {}
}

/***************************************************************************
 *                                                                         *
 *              Main procedure                                             *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief Main routine
 *
 * @param Initializes LEDs, SysTick and UART
 * @param Sends a Hello message
 * @param Blinks the LEDs
 * @param Increments a counter printing it at each cycle
 */

int main(void) {
uint32_t counter = 0;
char str[10];

    LED_Init();

    SysTick_Config(SystemCoreClock/1000); // 1 ms

    UART_Init();

    puts("\x1B[2J\x1B[;H");
    puts("Hello\r\n");

    fgets(str,10,stdin);

    while(1) {

        counter++;

        LED_Write(LED_ALL, LED_RED);
        Delay(1000);

        LED_Write(LED_ALL, LED_GREEN);
        Delay(1000);

        LED_Write(LED_ALL, LED_BLUE);
        Delay(1000);

        UART_SendChar('\n');

        printf("counter = %d\n",counter);

    }
    return 0;
}
