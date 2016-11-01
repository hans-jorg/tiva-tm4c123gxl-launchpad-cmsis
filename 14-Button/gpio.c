/**
 * @file     gpio.c
 * @brief    Routines to control GPIO including callback for interrupt
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     CMSIS library used
 * @note     The GPIO interface is implemented using callback routine
 * @note     Uses bit-banding
 *
 **/


#include "TM4C123GH6PM.h"
#include "gpio.h"

#define BIT(N) (1U<<(N))


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

static void xdelay(unsigned volatile v) {

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
GPIO_Init(uint32_t outputs, uint32_t inputs) {
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

    /* To modify PF0, it is necessary to unlock it. See 10.1 of Datasheet */
    if( (inputs|outputs)&1 ) {
        gpio->LOCK = 0x4C4F434B;                // unlock to set PF0
        *(uint32_t * )(&(gpio->CR))   = 0x0;    // CR is marked read-only
    }

    /* Pins for led are digital output */
    gpio->DIR    = outputs;         /* Only specified bits are outputs */
    gpio->DEN    = inputs|outputs;  /* All pins are digital I/O        */

    xdelay(10);
    if( (inputs|outputs)&1 ) {
        gpio->LOCK = 0x0;                       // lock again
        *(uint32_t * )(&(gpio->CR))   = 0x0;    // CR is marked read-only
    }

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
GPIO_WritePins(uint32_t zeroes, uint32_t ones) {
uint32_t *base;

#ifdef USE_AHB
    base = (uint32_t *) GPIOF_AHB_BASE;
#else
    base = (uint32_t *) GPIOF_BASE;
#endif

    *(base+(zeroes|ones)) = ones;
}


/**
 * @brief Reads pins in GPIO Port F
 */
uint32_t
GPIO_ReadPins(void) {
GPIOA_Type *gpio;

#ifdef USE_AHB
    gpio = GPIOF_AHB;
#else
    gpio = GPIOF;
#endif
    return gpio->DATA;
}

/**
 * @brief global variables for interrupt routines
 */
static void (*gpiocallback)(uint32_t) = 0;      // routine to be called when interrupt occurs
static uint32_t gpiointpins = 0;                // pins enabled to interrupt

/**
 * @brief Enable interrupts from specified pins
 */
void
GPIO_EnableInterrupt(uint32_t pins, void (*callback)(uint32_t) ) {
GPIOA_Type *gpio;

    // Enable Interrupt from GPIO Port F
    NVIC_DisableIRQ (GPIOF_IRQn);

#ifdef USE_AHB
    gpio = GPIOF_AHB;
#else
    gpio = GPIOF;
#endif

    if( (pins&1)!=0 ) {
        gpio->LOCK = 0x4C4F434B;                // unlock to set PF0 (ASCII=LOCB)
        *(uint32_t * )(&(gpio->CR)) = 0x1;      // Conversion is needed because
                                                // CR is marked read-only
    }


    gpio->IM = 0;                   // Disable interrupts for this port

    gpio->DEN |= pins;              // Set pins to digital I/O
    gpio->DIR &= ~pins;              // Set pins as input
    gpio->PUR |= pins;              // Enable Pull up

    gpio->IS  &= ~pins;             // Edge interrupt
    gpio->IBE |= pins;              // Enable for both edges
    gpio->IEV |= pins;              // Which kind of event

    gpio->ICR |= pins;              // Clear interrupts

    gpiointpins  = pins;
    gpiocallback = callback;


    if( (pins&1)!=0 ) {
        gpio->LOCK = 0x0;                       // Lock again
        *(uint32_t * )(&(gpio->CR)) = 0x0;      // Conversion is needed because
                                                // CR is marked read-only
    }


    // Enable interrupt
    gpio->IM = pins;

    // Enable Interrupt from GPIO Port F
    NVIC_EnableIRQ (GPIOF_IRQn);
}
//@}

/**
 * @brief Interrupt Routine for GPIO Port F
 */

//@{
void GPIOF_IRQHandler(void)  {
GPIOA_Type *gpio;
uint32_t m;

#ifdef USE_AHB
    gpio = GPIOF_AHB;
#else
    gpio = GPIOF;
#endif

    // Clear interrupt.
    // Must be at the very beginning
    gpio->ICR = gpiointpins;

    m = gpio->RIS;

    if( gpiocallback ) gpiocallback(m);



}
//@}
