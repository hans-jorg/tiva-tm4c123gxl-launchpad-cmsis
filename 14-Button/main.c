/**
 * @file     main.c
 * @brief    Blink LEDs using an API for GPIO
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Uses interrupt to monitor input
 * @note     Uses systick to count time
 * @note     The GPIO interface is implemented using callback routine
 * @note     The blinking frequency does not depends on core frequency
 * @note     The button can invert the blinking cycle
 * @note     CMSIS library used
 *
 **/

#include "TM4C123GH6PM.h"
#include "gpio.h"

/**
 * BIT Macro
 *
 * To avoid magic numbers
 */
#define BIT(N) (1U<<(N))

/**
 * LEDs are in PortF(3:1)
 */
//@{
#define LED_RED BIT(1)
#define LED_BLUE BIT(2)
#define LED_GREEN BIT(3)
#define LED_ALL (LED_RED|LED_BLUE|LED_GREEN)
#define LED_NONE 0
//@}

/**
 * Switches are in PortF(4) and PortF(0)
 */
//@{
#define SW1     BIT(4)
#define SW2     BIT(0)
#define SW_ALL  (SW1|SW2)
//@}


/***************************************************************************
 *                                                                         *
 *              SysTick Routines                                           *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief SysTick Handler
 *
 * @note  Called every 1 ms
 */

//@{
volatile uint32_t tick = 0;

void SysTick_Handler(void) {

    tick++;

}
//@}
/**
 * @brief Delay
 *
 * @note  It gives a delay milliseconds delay
 *
 */
void Delay(uint32_t delay) {
uint32_t tbegin = tick;

    while( (tick-tbegin)<delay ) {}
}

/**
 * @brief udelay
 *
 * @note  It gives a precise delay
 * @note  Units are given by the period of SysTick clock
 */
void udelay(uint32_t delay) {
uint32_t tbegin = SysTick->VAL;

    while( ((SysTick->VAL-tbegin)&0xFFFFFF)<delay ) {}

}

/**
 * @brief xdelay
 *
 * @note  It gives a very small delay
 */
void xdelay(volatile uint32_t delay) {

    while( delay-- ) {}

}

/***************************************************************************
 *                                                                         *
 *              Main procedure                                             *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief Callback routine
 *
 * @note  Reverses blink sequence
 */
//@{

int sentido = 0;

void switchmonitor(uint32_t w) {

    sentido = !sentido;
}

/**
 * @brief Main routine
 *
 * Initializes GPIO
 * Blink LEDs
 */

int main(void) {
int state;


    SysTick_Config(SystemCoreClock/1000);

    GPIO_Init(LED_ALL,SW_ALL);

    GPIO_EnableInterrupt(SW_ALL,switchmonitor);

    // EnableInterrupt
    NVIC_EnableIRQ (SysTick_IRQn);

    state = 0;
    while(1) {

        switch(state) {
        case 0:
            GPIO_WritePins(LED_ALL, LED_RED);
            Delay(1000);
            if( sentido )
                state = 1;
            else
                state = 2;
            break;
        case 1:
            GPIO_WritePins(LED_ALL, LED_GREEN);
            Delay(1000);
            if( sentido )
                state = 2;
            else
                state = 0;
            break;
        case 2:
            GPIO_WritePins(LED_ALL, LED_BLUE);
            Delay(1000);
            if( sentido )
                state = 0;
            else
                state = 1;
            break;
        }
    }
    return 0;
}
