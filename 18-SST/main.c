/**
 * @file     main.c
 * @brief    Blink LEDs using Super Simple Tasker
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Uses state machine
 * @note     Uses polling
 * @note     User LED and button routines
 * @note     The blinking frequency does not depends on core frequency
 * @note     The button can invert the blinking cycle
 * @note     CMSIS library used
 *
 **/

#include "TM4C123GH6PM.h"
#include "system_tm4c123.h"
#include "led.h"
#include "button.h"
#include "sst.h"
#include "sst_port.h"
#include "sst_stdsignal.h"
#include "sst_usersignal.h"

/***************************************************************************
 *                                                                         *
 *              Global variables and definitions                           *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief Priorities for tasks
 */
//@{
#define BLINK_GREEN_PRIO    1
#define BLINK_RED_PRIO      2
#define BUTTON_PRIO         3
#define ISR_TICK_PRIO       200
//@}

/// Queue size
#define QUEUE_SIZE 3


/// Blinking state (ative|inactive)
uint32_t blinking = 1;                  // blinking active



/// Queue
//@{
static SSTEvent BlinkGreenQueue[QUEUE_SIZE];
static SSTEvent BlinkRedQueue[QUEUE_SIZE];
//@}

/**
 * @brief Control blinking period
 */

//@{
uint32_t semiperiod_max   = 5000;
uint32_t semiperiod_min   = 100;
uint32_t semiperiod_step  = 100;

uint32_t semiperiod_red   = 250;        // 1 sec
uint32_t semiperiod_green = 1000;        // 1.5 sec

//static uint32_t cnt_red = 0;
//static uint32_t cnt_green = 0;
//@}


/***************************************************************************
 *                                                                         *
 *              SysTick                                                    *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief SysTick Handler
 *
 * @note Almost everything happens here
 */
//@{
static volatile uint32_t msTick = 0; // overflow after 49 days

void SysTick_Handler(void) {
int pin;

    SST_ISR_ENTRY(pin,ISR_TICK_PRIO);
//  SST_post(myTask_ID,1,0);
    msTick++;
    SST_post(BLINK_GREEN_PRIO, ISR_TICK_SIG, 0);
    SST_post(BLINK_RED_PRIO, ISR_TICK_SIG, 0);
//    SST_post(BUTTON_PRIO, ISR_TICK_SIG, 0);
    SST_ISR_EXIT(pin,(SCB->ICSR = SCB_ICSR_PENDSVSET_Msk));

}
//@}


/***************************************************************************
 *                                                                         *
 *              Tasks                                                      *
 *                                                                         *
 ***************************************************************************/


/// State definition
//@{
#define Q0     0
#define Q1     1
#define Q2     2
#define Q3     3
//@}

/**
 * @brief SST Tasks
 *
 */
//@{
void Task_Blink_Red(SSTEvent event){
static uint32_t lasttick = 0;

    if(event.sig!=SST_SIGNAL_TASKINIT) {
        if( blinking ) {
            // static int state = 0
            // if (state == 0) {
            //    LED_Write(LED_RED,0);
            // } else {
            //    LED_Write(0,LED_RED);
            // }
            //
            if( msTick > (lasttick+semiperiod_red) ) {
                LED_Toggle(LED_RED);
                lasttick = msTick;
            };
        } else {
            LED_Write(0,LED_RED);
        }
    }
}

void Task_Blink_Green(SSTEvent event){
static uint32_t lasttick = 0;

    if(event.sig!=SST_SIGNAL_TASKINIT) {
        if( blinking ) {
            // static int state = 0
            // if (state == 0) {
            //    LED_Write(LED_GREEN,0);
            // } else {
            //    LED_Write(0,LED_GREEN);
            // }
            //
            if( msTick > (lasttick+semiperiod_green) ) {
                LED_Toggle(LED_GREEN);
                lasttick = msTick;
            };
        } else {
            LED_Write(0,LED_GREEN);
        }
    }

}


#define DEBOUNCE_TIME 40

void Task_Button(SSTEvent event) {
static uint32_t state = Q0;
static uint32_t start = 0;

    if(event.sig!=SST_SIGNAL_TASKINIT){
        switch(state) {
        case Q0:
            if( Button_Read()&SW1 ) {
                start = msTick;
                blinking = ! blinking;
                state = Q1;
            }
            break;
        case Q1:
            if( (msTick-start) > DEBOUNCE_TIME ) {
                state = Q2;
            }
            break;
        case Q2:
            if( Button_Read()&SW1 ) {
                state = Q2;
            } else {
                state = Q0;
            }
            break;
        }
    }
}
//@}

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
    Button_Init();


    SST_init();

    //SST_task(myTask,myTask_ID, myTask_EQ, myTask_EVQL, SST_SIGNAL_TASKINIT, 0);

    SST_task(Task_Blink_Green, BLINK_GREEN_PRIO, BlinkGreenQueue, QUEUE_SIZE,
             SST_SIGNAL_TASKINIT, 0);
    SST_task(Task_Blink_Red, BLINK_RED_PRIO,     BlinkRedQueue,   QUEUE_SIZE,
             SST_SIGNAL_TASKINIT, 0);
    SST_task(Task_Button, BUTTON_PRIO,           0, 0, SST_SIGNAL_TASKINIT, 0);

    SysTick_Config(SystemCoreClock/1000);   /* 1 ms */


    SST_run();
}
