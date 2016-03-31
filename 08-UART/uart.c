/**
 * @file     uart.c
 * @brief    UART interface using CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     115200 bps, 8 bits, no parity
 * @note     Assumes 16 MHz clock frequency
 *
 **/

#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm-fields.h"
#include "uart.h"

/***************************************************************************
 *                                                                         *
 *              UART routines                                              *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief UART_Init
 *
 * @note  Initializes UART0 to 115200, 8 bit, no parity
 * @note  Assumes 16 MHz frequency
 *
 */
void
UART_Init(void) {
unsigned w;

    //
    // Enable UART0
    //
    SYSCTL->RCGCUART |= SYSCTL_RCGCUART_R0_B;

    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SYSCTL->RCGCGPIO |= SYSCTL_RCGCGPIO_R0_B;
    //
    // Configure GPIO Pins for UART mode.
    //
    w = GPIOA->PCTL;
    w &= ~(GPIO_PCTL_PA0_M|GPIO_PCTL_PA1_M);
    w |= GPIO_PCTL_PA0_U0RX_V|GPIO_PCTL_PA1_U0TX_V;
    GPIOA->PCTL = w;

    GPIOA->AFSEL   |=  0x03;
    GPIOA->DIR     &= ~0x03;
    GPIOA->DEN     |=  0x03;

    SYSCTL->PPUART |=  0x01;

    __NOP();
    __NOP();
    __NOP();

    //
    // Disable UART0
    //
    UART0->CTL &= ~(UART_CTL_UARTEN_B);


    //
    // Configure UART0
    //
    UART0->CC = UART_CC_CS_PIOSC_V;

    UART0->CTL &= ~(UART_CTL_HSE_B);
    UART0->LCRH = (UART0->LCRH&~UART_LCRH_WLEN_M)|UART_LCRH_WLEN_8_V;

    // Assumes system clock to be 16 MHz
    UART0->IBRD   = 8;                  // 115200
    UART0->FBRD   = 43;

    UART0->CTL  = (UART_CTL_TXE_B|UART_CTL_RXE_B);
    UART0->FR   = 0;

    //
    // Re-enable UART0
    //
    UART0->CTL  |= UART_CTL_UARTEN_B;

}

/**
 * @brief UART_SendChar
 *
 * @note  Waits (Blocks) until UART is free, then sends a char
 * @param ch char to be sent
 *
 */

void UART_SendChar(char ch) {

    while( (UART0->FR&UART_FR_TXFE_B)==0 ) {} // wait until last char transmitted
    UART0->DR = ch&0xFF;
}

/**
 * @brief UART_SendString
 *
 * @note  Sends an string
 * @param p : string to be sent
 */

void UART_SendString(char *p ) {

    while( *p )
        UART_SendChar(*p++);
}

/**
 * @brief UART_SendEOL
 *
 * @note  Sends an End of Line (EOL) sequence
 */

void UART_SendEOL(void) {
    UART_SendString("\n\r");
}
