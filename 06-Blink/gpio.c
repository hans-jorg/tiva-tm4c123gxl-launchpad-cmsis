/**
 * @file     gpio.c
 * @brief    Control LEDs using CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     The blinking frequency depends on core frequency
 *
 **/


#include "TM4C123GH6PM.h"
#include "gpio.h"

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
//@{
// signalizes use of AHB bus
#define AHBBIT 0x8000000
// table of all GPIOs
static struct gpioinfo {
    uint32_t    address;
    uint32_t    bitmask;
} gpiotab[] = {
#ifdef GPIOA_BASE
    {   GPIOA_BASE,  0x01    },
#endif
#ifdef GPIOB_BASE
    {   GPIOB_BASE,  0x02    },
#endif
#ifdef GPIOC_BASE
    {   GPIOC_BASE,  0x04    },
#endif
#ifdef GPIOE_BASE
    {   GPIOD_BASE,  0x08    },
#endif
#ifdef GPIOE_BASE
    {   GPIOE_BASE,  0x10    },
#endif
#ifdef GPIOF_BASE
    {   GPIOF_BASE,  0x20    },
#endif
#ifdef GPIOG_BASE
    {   GPIOG_BASE,  0x40    },
#endif
#ifdef GPIOH_BASE
    {   GPIOH_BASE,  0x80    },
#endif
#ifdef GPIOA_AHB_BASE
    {   GPIOA_AHB_BASE,  0x01|AHBBIT    },
#endif
#ifdef GPIOB_AHB_BASE
    {   GPIOB_AHB_BASE,  0x02|AHBBIT    },
#endif
#ifdef GPIOC_AHB_BASE
    {   GPIOC_AHB_BASE,  0x04|AHBBIT    },
#endif
#ifdef GPIOD_AHB_BASE
    {   GPIOD_AHB_BASE,  0x08|AHBBIT    },
#endif
#ifdef GPIOE_AHB_BASE
    {   GPIOE_AHB_BASE,  0x10|AHBBIT    },
#endif
#ifdef GPIOF_AHB_BASE
    {   GPIOF_AHB_BASE,  0x20|AHBBIT    },
#endif
#ifdef GPIOG_AHB_BASE
    {   GPIOG_AHB_BASE,  0x40|AHBBIT    },
#endif
#ifdef GPIOH_AHB_BASE
    {   GPIOH_AHB_BASE,  0x80|AHBBIT    },
#endif
// End of table
    {   0,          0       }
};
//@}

/**
 * @brief Initialization of GPIO Port F
 *
 * @param gpiobaseaddr : Use symbol GPIOx_BASE or GPIOx_AHB_BASE defined in header
 * @param inputbits    : bits to be configured as input
 * @param outputbits   : bits to be configured as output
 */
GPIOA_Type *
GPIO_Init(uint32_t gpiobaseaddr,uint32_t inputbits, uint32_t outputbits) {
GPIOA_Type *gpio = (GPIOA_Type *) gpiobaseaddr;

    // Search in table info about GPIO
    struct gpioinfo *p = gpiotab;

    while( 1 ) {
        if( ! p->address )
            return 0;
        if( p->address == gpiobaseaddr )
            break;
        p++;
    }

    if( p->address == 0 )
        return 0;                   // Not found in table

    // If AHB address specified (AHBBIT is set), AHB access must be enabled
    if( p->bitmask & AHBBIT ) {
        SYSCTL->GPIOHBCTL |= p->bitmask&0x0FF;
    }

    SYSCTL->RCGCGPIO  |= p->bitmask&0x0FF;  // Enable clock for Port F

    // Only necessary for reconfiguration
    gpio->LOCK   = 0x4C4F434BU;     // Unlock with preprogrammed password
    gpio->AMSEL  = 0x00U;           // Disable analog functions
    gpio->PCTL   = 0x00U;           // GPIO
    gpio->AFSEL  = 0x00U;           // GPIO

    // Configure digital I/O and direction
    gpio->DIR    = outputbits;
    gpio->DEN    = outputbits|inputbits;

    xdelay(10);

    return gpio;
}

#ifdef USE_NOINLINE
/**
 * @brief Write to GPIO
 *
 * @note First, pins specified by zero are zeroed. Then the pins
 *       specified by ones, area set
 *
 * @note Defining INLINEWORKAROUND, a macro base implementation is used
 */

void
GPIO_WritePort(GPIOA_Type *gpio, uint32_t zeroes, uint32_t ones) {
    gpio->DATA = (gpio->DATA&(~zeroes))|ones;
}

/**
 * @brief Read from GPIO
 *
 */

uint32_t
GPIO_ReadPort(GPIOA_Type *gpio) {
    return gpio->DATA;
}
#endif
