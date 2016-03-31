#ifndef GPIO_H
#define GPIO_H
/**
 * @file     gpio.h
 * @brief    Header for gpio.c
 * @version  V1.00
 * @date     25/3/2016
 *
 * @note
 *
 **/
#include "TM4C123GH6PM.h"

GPIOA_Type *GPIO_Init(uint32_t gpiobaseaddr,uint32_t inputbits, uint32_t outputbits);

#ifdef INLINE_WORKAROUND
#define GPIO_WritePort(GPIO,ZEROES,ONES) do { (GPIO)->DATA = (gpio->DATA&(~ZEROES))|ONES;  } while(0);
#else
static inline void
GPIO_WritePort(GPIOA_Type *gpio, uint32_t zeroes, uint32_t ones) {
    gpio->DATA = (gpio->DATA&(~zeroes))|ones;
}
#endif

static inline uint32_t GPIO_ReadPort(GPIOA_Type *gpio) {  return gpio->DATA; }

#endif // GPIO_H
