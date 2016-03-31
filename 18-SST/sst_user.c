
/**
 * @file     sst_user.c
 * @brief    SST specifics for Cortex M4F Architecture
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     No library except CMSIS is used
 *
 *
 ******************************************************************************/

#include <stdint.h>
#include "sst.h"
#include "TM4C123GH6PM.h"



int idle_counter = 0;

/**
 * @brief SST Init
 *
 * @note For application specific initialization
 */
void SST_init(void) {
    // nop
}

/**
 * @brief SST Start
 *        Configure interrupt levels
 *
 * @note Lowest priority for different Cortex M4F architectures
 *      - STM32L4xx    Cortex-M4F  15   4 bits
 *      - Tiva         Cortex-M4F   7   3 bits
 *      - LPC17xxx     Cortex-M3   15   4 bits
 */
void SST_start(void) {

    NVIC_SetPriority(PendSV_IRQn,15);   // lowest priority
    NVIC_EnableIRQ(PendSV_IRQn);        //
    NVIC_SetPriority(SysTick_IRQn,0);   // low priority

}

/**
 * @brief SST onIdle
 *        Idle routine
 *
 * @note Just increment a counter
 * @note Could be a sleep (enter a low power mode)
 */
void SST_onIdle(void) {

    idle_counter++;

}

/**
 * @brief SST exit
 *        Idle routine
 *
 * @note  Should no be called
 */
void SST_exit(void) {

    // halt : a loop to stop processor
    while(1) {}

}
