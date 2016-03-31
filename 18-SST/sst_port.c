/**
 * @file     sst_port.c
 * @brief    SST specifics for Cortex M4F Architecture
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     No library except CMSIS is used
 *
 *
 ******************************************************************************/

#include "sst_port.h"

int pendsv_counter;

/**
 * @brief PendSV Handler
 *
 * @note Last handler to be processed
 */

void PendSV_Handler(void) {

    pendsv_counter++;

    SST_INT_LOCK();
    SST_schedule_();
    SST_INT_UNLOCK();

/*
 * See http://embeddedgurus.com/state-space/2011/09/whats-the-state-of-your-cortex/
 */
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}
