/**
 * @file     main.c
 * @brief    Blink LEDs using Protothreads
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Uses protothreads
 * @note     Uses polling
 * @note     LED an button interface are used
 * @note     The blinking frequency does not depends on core frequency
 * @note     The button can invert the blinking cycle
 * @note     CMSIS library used
 *
 **/

#include "TM4C123GH6PM.h"
#include "system_tm4c123.h"
#include "led.h"
#include "button.h"
#include "pt.h"


/***************************************************************************
 *                                                                         *
 *              SysTick                                                    *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief SysTick_Handler
 *
 * @note  Call StateMachine every 1000 interrupts
 */
//@{
static volatile uint32_t msTick = 0; // overflow after 49 days

void SysTick_Handler(void) {

    msTick++;
}
//@}


/***************************************************************************
 *                                                                         *
 *              Tasks                                                      *
 *                                                                         *
 ***************************************************************************/

/// Blinking state (ative|inactive)
uint32_t blinking = 1;                  // blinking active

/// Protothreads context
struct pt pt_BlinkGreen, pt_BlinkRed, pt_ButtonProc;

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

/**
 * @brief Blink Red Task
 *
 * @note Called every semiperiod
 */
static
PT_THREAD(Blink_Red(struct pt *pt)) {
static uint32_t tstart;

    PT_BEGIN(pt);
    while( 1 ) {
        if( blinking )
            // static int state = 0
            // if (state == 0) {
            //    LED_Write(LED_RED,0);
            // } else {
            //    LED_Write(0,LED_RED);
            // }
            //
            LED_Toggle(LED_RED);
        else
            LED_Write(0,LED_RED);
        tstart = msTick;
        PT_WAIT_UNTIL(pt,((msTick-tstart)>=semiperiod_red));
    }
    PT_END(pt);
}

/**
 * @brief Blink Green Task
 *
 * @note Called every semiperiod
 */
static
PT_THREAD(Blink_Green(struct pt *pt)) {
static uint32_t tstart;

    PT_BEGIN(pt);
    while(1) {
        if( blinking )
            // static int state = 0
            // if (state == 0) {
            //    LED_Write(LED_GREEN,0);
            // } else {
            //    LED_Write(0,LED_GREEN);
            // }
            //
            LED_Toggle(LED_GREEN);
        else
            LED_Write(0,LED_GREEN);
        tstart = msTick;
        PT_WAIT_UNTIL(pt,((msTick-tstart)>=semiperiod_green));
    }
    PT_END(pt);
}

/// Debounce Time
#define DEBOUNCE_TIME 40

/**
 * @brief Button Task
 *
 * @note Called every 1 ms
 */
static
PT_THREAD(ButtonProc(struct pt *pt)) {
static uint32_t tstart;
uint32_t b;

    PT_BEGIN(pt);
    while(1) {
        PT_WAIT_UNTIL(pt,Button_Read()&SW1);
        blinking = ! blinking;
        tstart = msTick;
        PT_WAIT_UNTIL(pt,((msTick-tstart)>=DEBOUNCE_TIME));
        PT_WAIT_UNTIL(pt,!(Button_Read()&SW1));
    }
    PT_END(pt);
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
    Button_Init();

    PT_INIT(&pt_BlinkGreen);
    PT_INIT(&pt_BlinkRed);
    PT_INIT(&pt_ButtonProc);

    SysTick_Config(SystemCoreClock/1000);   /* 1 ms */

    for (;;) {
        PT_SCHEDULE(Blink_Green(&pt_BlinkGreen));
        PT_SCHEDULE(Blink_Red(&pt_BlinkRed));
        PT_SCHEDULE(ButtonProc(&pt_ButtonProc));
    }
}
