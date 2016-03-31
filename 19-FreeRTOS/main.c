/**
 * @file     main.c
 * @brief    Blink LEDs using FreeRTOS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Uses FreeRTOS
 * @note     CMSIS library used
 * @note     Tasks are implemented
 * @note     Uses LED and button routines
 *
 **/

#include "TM4C123GH6PM.h"
#include "system_tm4c123.h"
#include "led.h"
#include "button.h"
#include "FreeRTOS.h"
#include "task.h"

/// Specifies which form of Delay is used
#define USE_TASKDELAYUNTIL 1

/***************************************************************************
 *                                                                         *
 *              Global variables and definitions                           *
 *                                                                         *
 ***************************************************************************/



/// Blinking state (ative|inactive)
uint32_t blinking = 1;                  // blinking active

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

/***************************************************************************
 *                                                                         *
 *              Tasks                                                      *
 *                                                                         *
 ***************************************************************************/


/**
 * @brief  Blink Red LED Task
 *
 */

void Task_Blink_Red(void *pvParameters){
const portTickType xFrequency = 1000;
portTickType xLastWakeTime=xTaskGetTickCount();

    while(1) {
        if( blinking ) {
            LED_Toggle(LED_RED);
#ifdef TASKDELAYUNTIL
            vTaskDelayUntil(&xLastWakeTime,xFrequency);
#else
            vTaskDelay(semiperiod_red);
#endif
        } else {
            LED_Write(0,LED_RED);
        }
    }
}

/**
 * @brief  Blink Green LED Task
 *
 */
void Task_Blink_Green(void *pvParameters){
const portTickType xFrequency = 500;
portTickType xLastWakeTime=xTaskGetTickCount();

    while(1) {
        if( blinking ) {
            LED_Toggle(LED_GREEN);
#ifdef TASKDELAYUNTIL
            vTaskDelayUntil(&xLastWakeTime,xFrequency);
#else
            vTaskDelay(semiperiod_green);
#endif
        } else {
            LED_Write(0,LED_GREEN);
        }
    }
}
//@}


/**
 * @brief  Switch Control Task
 *
 */
//@{
#define DEBOUNCE_TIME 40

void Task_Button(void *pvParameters) {

    while(1) {
        if( (Button_Read()&SW1) != 0 ) {
            blinking = ! blinking;
            vTaskDelay(DEBOUNCE_TIME);
            while( (Button_Read()&SW1) == 0 ) {
                vTaskDelay(DEBOUNCE_TIME);
            }
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

    Button_Init();
    LED_Init();

 //   SysTick_Config(SystemCoreClock/1000);   /* 1 ms */

    xTaskCreate(Task_Blink_Red,"Red", 1000,0,1,0);
    xTaskCreate(Task_Blink_Green,"Green", 1000,0,2,0);
    xTaskCreate(Task_Button,"Button", 10,0,3,0);

    vTaskStartScheduler();

    while(1) {}
}
