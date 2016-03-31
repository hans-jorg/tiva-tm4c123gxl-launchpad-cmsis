#ifndef BUTTON_H
#define BUTTON_H
/**
 * @file     button.h
 * @brief    Header for button.c
 * @version  V1.0
 * @date     23/01/2016
 *
 **/
#ifndef BIT
#define BIT(N) (1U<<(N))
#endif
/**
 * Switches are in PortF(4) and PortF(0)
 */
//@{
#define SW1     BIT(4)
#define SW2     BIT(0)
#define SW_ALL  (SW1|SW2)
//@}

void Button_Init(void);
uint32_t Button_Read(void);

#endif
