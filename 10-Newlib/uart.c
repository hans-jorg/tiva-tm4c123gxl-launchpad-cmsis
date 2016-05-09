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

#define BIT(N) (1U<<(N))

static void xdelay(volatile uint32_t v) { while (v--) {__NOP();} }

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
    // Enable clock for GPIO used by the UART.
    //
    // Careful: Can be configured to use AHB
    //
    SYSCTL->RCGCGPIO |= BIT(0); //SYSCTL_RCGCGPIO_R0_B;

    //
    // Enable Clock for UART0
    //
    SYSCTL->RCGCUART |= BIT(0); //SYSCTL_RCGCUART_R0_B;


    //
    // Configure GPIO Pins for UART mode.
    //
    w = GPIOA->PCTL;
    w &= ~(GPIO_PCTL_PA0_M|GPIO_PCTL_PA1_M);
    w |= GPIO_PCTL_PA0_U0RX_V|GPIO_PCTL_PA1_U0TX_V;
    GPIOA->PCTL = w;


    GPIOA->AFSEL   |=  (BIT(1)|BIT(0));
    GPIOA->DIR     &= ~(BIT(1)|BIT(0));
    GPIOA->DR2R    |=  (BIT(1)|BIT(0));
    GPIOA->DEN     |=  (BIT(1)|BIT(0));

    //
    // Disable UART0
    //
    UART0->CTL &= ~(UART_CTL_UARTEN_B);


    //
    // Configure UART0 to use PIOSC (16 MHz) as clock source
    //
    UART0->CC = UART_CC_CS_PIOSC_V;

    //
    // Set divisor (HSE=0, divisor=16   HSE=1, divisor=8)
    //
    UART0->CTL &= ~(UART_CTL_HSE_B); // Divisor = 16
//    UART0->CTL |= UART_CTL_HSE_B; // Divisor = 16

    //
    // Set WCommunication parameters (Word length, Parity, Stop bits, FIFO)
    // To enable FIFO: Set FEN
    // To enable Parity: Set PEN
    // To use Even Parity: Set EPS
    //
    UART0->LCRH &= ~(UART_LCRH_PEN_B);
    UART0->LCRH = (UART0->LCRH&~UART_LCRH_WLEN_M)|UART_LCRH_WLEN_8_V;

    // Assumes system clock to be 16 MHz
    //                                 UART Clock
    //   Baud rate Divisor = --------------------------------
    //                           Clock Divisor * Baud rate
    //
    //   IBRD = INTEGER(Baud rate divisor)
    //   FBRD = FRAC(Baud rate divisor)*64
    //
    //   UART Clock = 16 MHz
    //                           HSE=1           HSE=0
    //    115200                17  23            8  44
    //
    //
    UART0->IBRD   = 8;                  // 115200
    UART0->FBRD   = 44;

    //
    // Configure Line Control to no line control
    //
    UART0->CTL &= ~(UART_CTL_CTSEN_B|UART_CTL_RTSEN_B);

    //
    // Clear flags
    //
    UART0->FR = 0;

    //
    // Enable FIFO
    //
    UART0->LCRH |= UART_LCRH_FEN_B;

    //
    // Re-enable UART0 enable Receiver and Transmitter
    //
    UART0->CTL  |= (UART_CTL_UARTEN_B|UART_CTL_TXE_B|UART_CTL_RXE_B);

}

/**
 * @brief UART_GetStatus
 *
 * @note  Get status of UART
 * @note  Returns 0 if there is no char in buffer and 1, if there is one
 */

int  UART_GetStatus(void) {

    return (UART0->FR&UART_FR_RXFE_B)==0; // wait until char received

}

/**
 * @brief UART_ReceiveChar
 *
 * @note  Waits (Blocks) until a character is received, then reads and return it
 *
 */

int UART_ReceiveChar(void) {

    while( (UART0->FR&UART_FR_RXFE_B)==0 ) {} // wait until char received
    return (UART0->DR&0xFF);
}

/**
 * @brief UART_SendChar
 *
 * @note  Waits (Blocks) until UART is free, then sends a char
 * @param ch char to be sent
 *
 */

int UART_SendChar(char ch) {

    while ( UART0->FR&UART_FR_TXFF_B ) {}

    UART0->DR = ch&0xFF;
    return 0;
}

/**
 * @brief UART_SendCharNoBlocking
 *
 * @note  Waits (Blocks) until UART is free, then sends a char
 * @param ch char to be sent
 *
 */

int UART_SendCharNoBlocking(char ch) {

    if ( UART0->FR&UART_FR_TXFF_B ) {
        return 1;
    }
    UART0->DR = ch&0xFF;
    return 0;
}

/**
 * @brief UART_SendString
 *
 * @note  Sends an string
 * @param p : string to be sent
 */

int UART_SendString(char *p ) {

    while( *p ) {
        if( UART_SendChar(*p++) ) return 1;
    }
    return 0;
}

/**
 * @brief UART_SendEOL
 *
 * @note  Sends an End of Line (EOL) sequence
 */

int UART_SendEOL(void) {
    return UART_SendString("\n\r");
}
