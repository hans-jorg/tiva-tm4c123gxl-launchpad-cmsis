#ifndef GPIO_H
#define GPIO_H
/**
 * @file     gpio.h
 * @brief    Header for gpio.c
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     CMSIS library used
 * @note     Uses bit-banding
 *
 **/

void GPIO_Init(uint32_t outputs, uint32_t inputs);
void GPIO_WritePins(uint32_t zeroes, uint32_t ones);
uint32_t GPIO_ReadPins(void);
void GPIO_EnableInterrupt(uint32_t pins, void (*callback)(uint32_t) );

#endif
