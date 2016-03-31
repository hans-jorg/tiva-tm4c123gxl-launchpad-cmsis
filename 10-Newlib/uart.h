#ifndef UART_H
#define UART_H
/**
 * @file     uart.h
 * @brief    Header for uart.c
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     115200 bps, 8 bits, no parity
 * @note     Assumes 16 MHz clock frequency
 *
 **/
#include "uart.h"

void UART_Init(void);
void UART_SendChar(char ch);
void UART_SendString(char *p );
void UART_SendEOL(void);

int  UART_ReceiveChar(void);
int  UART_GetStatus(void);

#endif
