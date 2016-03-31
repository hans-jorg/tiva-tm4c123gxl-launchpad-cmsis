#ifndef LED_H
#define LED_H
/**
 * @file     led.h
 * @brief    Header for led.c
 *
 * @version  V1.00
 * @date     25/3/2016
 *
 * @note
 *
 **/
#define LEDBIT(N)    (1U<<(N))

#define LED_RED      LEDBIT(1)
#define LED_BLUE     LEDBIT(2)
#define LED_GREEN    LEDBIT(3)

#define LED_ALL      (LED_RED|LED_BLUE|LED_GREEN)
#define LED_NONE     (0)



/*
 * LED_DONT_USE_AHB Flag
 *    Set it to access GPIO thru APB: 0x40025000 (legacy)
 *    Do not set it to access GPIO thru AHB: 0x4005D000 (faster!)
 */

#ifndef LED_DONT_USE_AHB
#define USE_AHB
#endif


void LED_Init(void);
void LED_Write(uint32_t zeroes, uint32_t ones);

#endif
