/**
 * @file     tm4c123gh6pm-fields.h
 * @brief    Contains bit and bit fields definition for tm4c123gh6pm
 *
 * @version  V1.00
 * @date     25/3/2016
 *
 * @note     Symbols ended in _B are bits
 * @note     Symbols ended in _M are bit masks
 * @note     Symbols ended in _V are values, e.g., for bit fields
 *
 **/
#ifndef TM4C123_FIELDS_H
#define TM4C123_FIELDS_H

/******************************************************************************/
/*                                                                            */
/*                      INT                                                   */
/*                                                                            */
/******************************************************************************/

#define INT_GPIOA_V                 16 // GPIO Port A
#define INT_GPIOB_V                 17 // GPIO Port B
#define INT_GPIOC_V                 18 // GPIO Port C
#define INT_GPIOD_V                 19 // GPIO Port D
#define INT_GPIOE_V                 20 // GPIO Port E
#define INT_UART0_V                 21 // UART0
#define INT_UART1_V                 22 // UART1
#define INT_SSI0_V                  23 // SSI0
#define INT_I2C0_V                  24 // I2C0
#define INT_PWM0_FAULT_V            25 // PWM0 Fault
#define INT_PWM0_0_V                26 // PWM0 Generator 0
#define INT_PWM0_1_V                27 // PWM0 Generator 1
#define INT_PWM0_2_V                28 // PWM0 Generator 2
#define INT_QEI0_V                  29 // QEI0
#define INT_ADC0SS0_V               30 // ADC0 Sequence 0
#define INT_ADC0SS1_V               31 // ADC0 Sequence 1
#define INT_ADC0SS2_V               32 // ADC0 Sequence 2
#define INT_ADC0SS3_V               33 // ADC0 Sequence 3
#define INT_WATCHDOG_V              34 // Watchdog Timers 0 and 1
#define INT_TIMER0A_V               35 // 16/32-Bit Timer 0A
#define INT_TIMER0B_V               36 // 16/32-Bit Timer 0B
#define INT_TIMER1A_V               37 // 16/32-Bit Timer 1A
#define INT_TIMER1B_V               38 // 16/32-Bit Timer 1B
#define INT_TIMER2A_V               39 // 16/32-Bit Timer 2A
#define INT_TIMER2B_V               40 // 16/32-Bit Timer 2B
#define INT_COMP0_V                 41 // Analog Comparator 0
#define INT_COMP1_V                 42 // Analog Comparator 1
#define INT_SYSCTL_V                44 // System Control
#define INT_FLASH_V                 45 // Flash Memory Control and EEPROM
#define INT_GPIOF_V                 46 // GPIO Port F
#define INT_UART2_V                 49 // UART2
#define INT_SSI1_V                  50 // SSI1
#define INT_TIMER3A_V               51 // 16/32-Bit Timer 3A
#define INT_TIMER3B_V               52 // Timer 3B
#define INT_I2C1_V                  53 // I2C1
#define INT_QEI1_V                  54 // QEI1
#define INT_CAN0_V                  55 // CAN0
#define INT_CAN1_V                  56 // CAN1
#define INT_HIBERNATE_V             59 // Hibernation Module
#define INT_USB0_V                  60 // USB
#define INT_PWM0_3_V                61 // PWM Generator 3
#define INT_UDMA_V                  62 // uDMA Software
#define INT_UDMAERR_V               63 // uDMA Error
#define INT_ADC1SS0_V               64 // ADC1 Sequence 0
#define INT_ADC1SS1_V               65 // ADC1 Sequence 1
#define INT_ADC1SS2_V               66 // ADC1 Sequence 2
#define INT_ADC1SS3_V               67 // ADC1 Sequence 3
#define INT_SSI2_V                  73 // SSI2
#define INT_SSI3_V                  74 // SSI3
#define INT_UART3_V                 75 // UART3
#define INT_UART4_V                 76 // UART4
#define INT_UART5_V                 77 // UART5
#define INT_UART6_V                 78 // UART6
#define INT_UART7_V                 79 // UART7
#define INT_I2C2_V                  84 // I2C2
#define INT_I2C3_V                  85 // I2C3
#define INT_TIMER4A_V               86 // 16/32-Bit Timer 4A
#define INT_TIMER4B_V               87 // 16/32-Bit Timer 4B
#define INT_TIMER5A_V               108 // 16/32-Bit Timer 5A
#define INT_TIMER5B_V               109 // 16/32-Bit Timer 5B
#define INT_WTIMER0A_V              110 // 32/64-Bit Timer 0A
#define INT_WTIMER0B_V              111 // 32/64-Bit Timer 0B
#define INT_WTIMER1A_V              112 // 32/64-Bit Timer 1A
#define INT_WTIMER1B_V              113 // 32/64-Bit Timer 1B
#define INT_WTIMER2A_V              114 // 32/64-Bit Timer 2A
#define INT_WTIMER2B_V              115 // 32/64-Bit Timer 2B
#define INT_WTIMER3A_V              116 // 32/64-Bit Timer 3A
#define INT_WTIMER3B_V              117 // 32/64-Bit Timer 3B
#define INT_WTIMER4A_V              118 // 32/64-Bit Timer 4A
#define INT_WTIMER4B_V              119 // 32/64-Bit Timer 4B
#define INT_WTIMER5A_V              120 // 32/64-Bit Timer 5A
#define INT_WTIMER5B_V              121 // 32/64-Bit Timer 5B
#define INT_SYSEXC_V                122 // System Exception (imprecise)
#define INT_PWM1_0_V                150 // PWM1 Generator 0
#define INT_PWM1_1_V                151 // PWM1 Generator 1
#define INT_PWM1_2_V                152 // PWM1 Generator 2
#define INT_PWM1_3_V                153 // PWM1 Generator 3
#define INT_PWM1_FAULT_V            154 // PWM1 Fault


/******************************************************************************/
/*                                                                            */
/*                      UDMA                                                  */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register SRCENDP of Module UDMA                              */
#define UDMA_SRCENDP_V              0x00000000 // DMA Channel Source Address End

/* Bit/Fields in Register DSTENDP of Module UDMA                              */
#define UDMA_DSTENDP_B              0x00000004 // DMA Channel Destination Address

/* Bit/Fields in Register CHCTL of Module UDMA                                */
#define UDMA_CHCTL_B                0x00000008 // DMA Channel Control Word


/******************************************************************************/
/*                                                                            */
/*                      WDT                                                   */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register LOAD of Module WDT                                  */
#define WDT_LOAD_M                  0xFFFFFFFF // Watchdog Load Value
#define WDT_LOAD_S                  0          // Watchdog Load Value

/* Bit/Fields in Register VALUE of Module WDT                                 */
#define WDT_VALUE_M                 0xFFFFFFFF // Watchdog Value
#define WDT_VALUE_S                 0          // Watchdog Value

/* Bit/Fields in Register CTL of Module WDT                                   */
#define WDT_CTL_WRC_B               0x80000000 // Write Complete
#define WDT_CTL_INTTYPE_B           0x00000004 // Watchdog Interrupt Type
#define WDT_CTL_RESEN_B             0x00000002 // Watchdog Reset Enable
#define WDT_CTL_INTEN_B             0x00000001 // Watchdog Interrupt Enable

/* Bit/Fields in Register ICR of Module WDT                                   */
#define WDT_ICR_M                   0xFFFFFFFF // Watchdog Interrupt Clear
#define WDT_ICR_S                   0          // Watchdog Interrupt Clear

/* Bit/Fields in Register RIS of Module WDT                                   */
#define WDT_RIS_WDTRIS_B            0x00000001 // Watchdog Raw Interrupt Status

/* Bit/Fields in Register MIS of Module WDT                                   */
#define WDT_MIS_WDTMIS_B            0x00000001 // Watchdog Masked Interrupt Status

/* Bit/Fields in Register TEST of Module WDT                                  */
#define WDT_TEST_STALL_B            0x00000100 // Watchdog Stall Enable

/* Bit/Fields in Register LOCK of Module WDT                                  */
#define WDT_LOCK_M                  0xFFFFFFFF // Watchdog Lock
#define WDT_LOCK_S                  0          // Watchdog Lock
#define WDT_LOCK_UNLOCKED_V         0x00000000 // Unlocked
#define WDT_LOCK_LOCKED_V           0x00000001 // Locked
#define WDT_LOCK_UNLOCK_V           0x1ACCE551 // Unlocks the watchdog timer


/******************************************************************************/
/*                                                                            */
/*                      GPIO                                                  */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register IM of Module GPIO                                   */
#define GPIO_IM_GPIO_M              0x000000FF // GPIO Interrupt Mask Enable
#define GPIO_IM_GPIO_S              0          // GPIO Interrupt Mask Enable

/* Bit/Fields in Register RIS of Module GPIO                                  */
#define GPIO_RIS_GPIO_M             0x000000FF // GPIO Interrupt Raw Status
#define GPIO_RIS_GPIO_S             0          // GPIO Interrupt Raw Status

/* Bit/Fields in Register MIS of Module GPIO                                  */
#define GPIO_MIS_GPIO_M             0x000000FF // GPIO Masked Interrupt Status
#define GPIO_MIS_GPIO_S             0          // GPIO Masked Interrupt Status

/* Bit/Fields in Register ICR of Module GPIO                                  */
#define GPIO_ICR_GPIO_M             0x000000FF // GPIO Interrupt Clear
#define GPIO_ICR_GPIO_S             0          // GPIO Interrupt Clear

/* Bit/Fields in Register LOCK of Module GPIO                                 */
#define GPIO_LOCK_M                 0xFFFFFFFF // GPIO Lock
#define GPIO_LOCK_S                 0          // GPIO Lock
#define GPIO_LOCK_UNLOCKED_V        0x00000000 // The GPIOCR register is unlocked
#define GPIO_LOCK_LOCKED_V          0x00000001 // The GPIOCR register is locked
#define GPIO_LOCK_KEY_V             0x4C4F434B // Unlocks the GPIO_CR register

/* Bit/Fields in Register PCTL of Module GPIO                                 */
#define GPIO_PCTL_PA7_M             0xF0000000 // PA7 Mask
#define GPIO_PCTL_PA7_S             28         // PA7 Mask
#define GPIO_PCTL_PA7_I2C1SDA_V     0x30000000 // I2C1SDA on PA7
#define GPIO_PCTL_PA7_M1PWM3_V      0x50000000 // M1PWM3 on PA7
#define GPIO_PCTL_PA6_M             0x0F000000 // PA6 Mask
#define GPIO_PCTL_PA6_S             24         // PA6 Mask
#define GPIO_PCTL_PA6_I2C1SCL_V     0x03000000 // I2C1SCL on PA6
#define GPIO_PCTL_PA6_M1PWM2_V      0x05000000 // M1PWM2 on PA6
#define GPIO_PCTL_PA5_M             0x00F00000 // PA5 Mask
#define GPIO_PCTL_PA5_S             20         // PA5 Mask
#define GPIO_PCTL_PA5_SSI0TX_V      0x00200000 // SSI0TX on PA5
#define GPIO_PCTL_PA4_M             0x000F0000 // PA4 Mask
#define GPIO_PCTL_PA4_S             16         // PA4 Mask
#define GPIO_PCTL_PA4_SSI0RX_V      0x00020000 // SSI0RX on PA4
#define GPIO_PCTL_PA3_M             0x0000F000 // PA3 Mask
#define GPIO_PCTL_PA3_S             12         // PA3 Mask
#define GPIO_PCTL_PA3_SSI0FSS_V     0x00002000 // SSI0FSS on PA3
#define GPIO_PCTL_PA2_M             0x00000F00 // PA2 Mask
#define GPIO_PCTL_PA2_S             8          // PA2 Mask
#define GPIO_PCTL_PA2_SSI0CLK_V     0x00000200 // SSI0CLK on PA2
#define GPIO_PCTL_PA1_M             0x000000F0 // PA1 Mask
#define GPIO_PCTL_PA1_S             4          // PA1 Mask
#define GPIO_PCTL_PA1_U0TX_V        0x00000010 // U0TX on PA1
#define GPIO_PCTL_PA1_CAN1TX_V      0x00000080 // CAN1TX on PA1
#define GPIO_PCTL_PA0_M             0x0000000F // PA0 Mask
#define GPIO_PCTL_PA0_S             0          // PA0 Mask
#define GPIO_PCTL_PA0_U0RX_V        0x00000001 // U0RX on PA0
#define GPIO_PCTL_PA0_CAN1RX_V      0x00000008 // CAN1RX on PA0
#define GPIO_PCTL_PB7_M             0xF0000000 // PB7 Mask
#define GPIO_PCTL_PB7_S             28         // PB7 Mask
#define GPIO_PCTL_PB7_SSI2TX_V      0x20000000 // SSI2TX on PB7
#define GPIO_PCTL_PB7_M0PWM1_V      0x40000000 // M0PWM1 on PB7
#define GPIO_PCTL_PB7_T0CCP1_V      0x70000000 // T0CCP1 on PB7
#define GPIO_PCTL_PB6_M             0x0F000000 // PB6 Mask
#define GPIO_PCTL_PB6_S             24         // PB6 Mask
#define GPIO_PCTL_PB6_SSI2RX_V      0x02000000 // SSI2RX on PB6
#define GPIO_PCTL_PB6_M0PWM0_V      0x04000000 // M0PWM0 on PB6
#define GPIO_PCTL_PB6_T0CCP0_V      0x07000000 // T0CCP0 on PB6
#define GPIO_PCTL_PB5_M             0x00F00000 // PB5 Mask
#define GPIO_PCTL_PB5_S             20         // PB5 Mask
#define GPIO_PCTL_PB5_SSI2FSS_V     0x00200000 // SSI2FSS on PB5
#define GPIO_PCTL_PB5_M0PWM3_V      0x00400000 // M0PWM3 on PB5
#define GPIO_PCTL_PB5_T1CCP1_V      0x00700000 // T1CCP1 on PB5
#define GPIO_PCTL_PB5_CAN0TX_V      0x00800000 // CAN0TX on PB5
#define GPIO_PCTL_PB4_M             0x000F0000 // PB4 Mask
#define GPIO_PCTL_PB4_S             16         // PB4 Mask
#define GPIO_PCTL_PB4_SSI2CLK_V     0x00020000 // SSI2CLK on PB4
#define GPIO_PCTL_PB4_M0PWM2_V      0x00040000 // M0PWM2 on PB4
#define GPIO_PCTL_PB4_T1CCP0_V      0x00070000 // T1CCP0 on PB4
#define GPIO_PCTL_PB4_CAN0RX_V      0x00080000 // CAN0RX on PB4
#define GPIO_PCTL_PB3_M             0x0000F000 // PB3 Mask
#define GPIO_PCTL_PB3_S             12         // PB3 Mask
#define GPIO_PCTL_PB3_I2C0SDA_V     0x00003000 // I2C0SDA on PB3
#define GPIO_PCTL_PB3_T3CCP1_V      0x00007000 // T3CCP1 on PB3
#define GPIO_PCTL_PB2_M             0x00000F00 // PB2 Mask
#define GPIO_PCTL_PB2_S             8          // PB2 Mask
#define GPIO_PCTL_PB2_I2C0SCL_V     0x00000300 // I2C0SCL on PB2
#define GPIO_PCTL_PB2_T3CCP0_V      0x00000700 // T3CCP0 on PB2
#define GPIO_PCTL_PB1_M             0x000000F0 // PB1 Mask
#define GPIO_PCTL_PB1_S             4          // PB1 Mask
#define GPIO_PCTL_PB1_USB0VBUS_V    0x00000000 // USB0VBUS on PB1
#define GPIO_PCTL_PB1_U1TX_V        0x00000010 // U1TX on PB1
#define GPIO_PCTL_PB1_T2CCP1_V      0x00000070 // T2CCP1 on PB1
#define GPIO_PCTL_PB0_M             0x0000000F // PB0 Mask
#define GPIO_PCTL_PB0_S             0          // PB0 Mask
#define GPIO_PCTL_PB0_USB0ID_V      0x00000000 // USB0ID on PB0
#define GPIO_PCTL_PB0_U1RX_V        0x00000001 // U1RX on PB0
#define GPIO_PCTL_PB0_T2CCP0_V      0x00000007 // T2CCP0 on PB0
#define GPIO_PCTL_PC7_M             0xF0000000 // PC7 Mask
#define GPIO_PCTL_PC7_S             28         // PC7 Mask
#define GPIO_PCTL_PC7_U3TX_V        0x10000000 // U3TX on PC7
#define GPIO_PCTL_PC7_WT1CCP1_V     0x70000000 // WT1CCP1 on PC7
#define GPIO_PCTL_PC7_USB0PFLT_V    0x80000000 // USB0PFLT on PC7
#define GPIO_PCTL_PC6_M             0x0F000000 // PC6 Mask
#define GPIO_PCTL_PC6_S             24         // PC6 Mask
#define GPIO_PCTL_PC6_U3RX_V        0x01000000 // U3RX on PC6
#define GPIO_PCTL_PC6_PHB1_V        0x06000000 // PHB1 on PC6
#define GPIO_PCTL_PC6_WT1CCP0_V     0x07000000 // WT1CCP0 on PC6
#define GPIO_PCTL_PC6_USB0EPEN_V    0x08000000 // USB0EPEN on PC6
#define GPIO_PCTL_PC5_M             0x00F00000 // PC5 Mask
#define GPIO_PCTL_PC5_S             20         // PC5 Mask
#define GPIO_PCTL_PC5_U4TX_V        0x00100000 // U4TX on PC5
#define GPIO_PCTL_PC5_U1TX_V        0x00200000 // U1TX on PC5
#define GPIO_PCTL_PC5_M0PWM7_V      0x00400000 // M0PWM7 on PC5
#define GPIO_PCTL_PC5_PHA1_V        0x00600000 // PHA1 on PC5
#define GPIO_PCTL_PC5_WT0CCP1_V     0x00700000 // WT0CCP1 on PC5
#define GPIO_PCTL_PC5_U1CTS_V       0x00800000 // U1CTS on PC5
#define GPIO_PCTL_PC4_M             0x000F0000 // PC4 Mask
#define GPIO_PCTL_PC4_S             16         // PC4 Mask
#define GPIO_PCTL_PC4_U4RX_V        0x00010000 // U4RX on PC4
#define GPIO_PCTL_PC4_U1RX_V        0x00020000 // U1RX on PC4
#define GPIO_PCTL_PC4_M0PWM6_V      0x00040000 // M0PWM6 on PC4
#define GPIO_PCTL_PC4_IDX1_V        0x00060000 // IDX1 on PC4
#define GPIO_PCTL_PC4_WT0CCP0_V     0x00070000 // WT0CCP0 on PC4
#define GPIO_PCTL_PC4_U1RTS_V       0x00080000 // U1RTS on PC4
#define GPIO_PCTL_PC3_M             0x0000F000 // PC3 Mask
#define GPIO_PCTL_PC3_S             12         // PC3 Mask
#define GPIO_PCTL_PC3_TDO_V         0x00001000 // TDO on PC3
#define GPIO_PCTL_PC3_T5CCP1_V      0x00007000 // T5CCP1 on PC3
#define GPIO_PCTL_PC2_M             0x00000F00 // PC2 Mask
#define GPIO_PCTL_PC2_S             8          // PC2 Mask
#define GPIO_PCTL_PC2_TDI_V         0x00000100 // TDI on PC2
#define GPIO_PCTL_PC2_T5CCP0_V      0x00000700 // T5CCP0 on PC2
#define GPIO_PCTL_PC1_M             0x000000F0 // PC1 Mask
#define GPIO_PCTL_PC1_S             4          // PC1 Mask
#define GPIO_PCTL_PC1_TMS_V         0x00000010 // TMS on PC1
#define GPIO_PCTL_PC1_T4CCP1_V      0x00000070 // T4CCP1 on PC1
#define GPIO_PCTL_PC0_M             0x0000000F // PC0 Mask
#define GPIO_PCTL_PC0_S             0          // PC0 Mask
#define GPIO_PCTL_PC0_TCK_V         0x00000001 // TCK on PC0
#define GPIO_PCTL_PC0_T4CCP0_V      0x00000007 // T4CCP0 on PC0
#define GPIO_PCTL_PD7_M             0xF0000000 // PD7 Mask
#define GPIO_PCTL_PD7_S             28         // PD7 Mask
#define GPIO_PCTL_PD7_U2TX_V        0x10000000 // U2TX on PD7
#define GPIO_PCTL_PD7_PHB0_V        0x60000000 // PHB0 on PD7
#define GPIO_PCTL_PD7_WT5CCP1_V     0x70000000 // WT5CCP1 on PD7
#define GPIO_PCTL_PD7_NMI_V         0x80000000 // NMI on PD7
#define GPIO_PCTL_PD6_M             0x0F000000 // PD6 Mask
#define GPIO_PCTL_PD6_S             24         // PD6 Mask
#define GPIO_PCTL_PD6_U2RX_V        0x01000000 // U2RX on PD6
#define GPIO_PCTL_PD6_M0FAULT0_V    0x04000000 // M0FAULT0 on PD6
#define GPIO_PCTL_PD6_PHA0_V        0x06000000 // PHA0 on PD6
#define GPIO_PCTL_PD6_WT5CCP0_V     0x07000000 // WT5CCP0 on PD6
#define GPIO_PCTL_PD5_M             0x00F00000 // PD5 Mask
#define GPIO_PCTL_PD5_S             20         // PD5 Mask
#define GPIO_PCTL_PD5_USB0DP_V      0x00000000 // USB0DP on PD5
#define GPIO_PCTL_PD5_U6TX_V        0x00100000 // U6TX on PD5
#define GPIO_PCTL_PD5_WT4CCP1_V     0x00700000 // WT4CCP1 on PD5
#define GPIO_PCTL_PD4_M             0x000F0000 // PD4 Mask
#define GPIO_PCTL_PD4_S             16         // PD4 Mask
#define GPIO_PCTL_PD4_USB0DM_V      0x00000000 // USB0DM on PD4
#define GPIO_PCTL_PD4_U6RX_V        0x00010000 // U6RX on PD4
#define GPIO_PCTL_PD4_WT4CCP0_V     0x00070000 // WT4CCP0 on PD4
#define GPIO_PCTL_PD3_M             0x0000F000 // PD3 Mask
#define GPIO_PCTL_PD3_S             12         // PD3 Mask
#define GPIO_PCTL_PD3_AIN4_V        0x00000000 // AIN4 on PD3
#define GPIO_PCTL_PD3_SSI3TX_V      0x00001000 // SSI3TX on PD3
#define GPIO_PCTL_PD3_SSI1TX_V      0x00002000 // SSI1TX on PD3
#define GPIO_PCTL_PD3_IDX0_V        0x00006000 // IDX0 on PD3
#define GPIO_PCTL_PD3_WT3CCP1_V     0x00007000 // WT3CCP1 on PD3
#define GPIO_PCTL_PD3_USB0PFLT_V    0x00008000 // USB0PFLT on PD3
#define GPIO_PCTL_PD2_M             0x00000F00 // PD2 Mask
#define GPIO_PCTL_PD2_S             8          // PD2 Mask
#define GPIO_PCTL_PD2_AIN5_V        0x00000000 // AIN5 on PD2
#define GPIO_PCTL_PD2_SSI3RX_V      0x00000100 // SSI3RX on PD2
#define GPIO_PCTL_PD2_SSI1RX_V      0x00000200 // SSI1RX on PD2
#define GPIO_PCTL_PD2_M0FAULT0_V    0x00000400 // M0FAULT0 on PD2
#define GPIO_PCTL_PD2_WT3CCP0_V     0x00000700 // WT3CCP0 on PD2
#define GPIO_PCTL_PD2_USB0EPEN_V    0x00000800 // USB0EPEN on PD2
#define GPIO_PCTL_PD1_M             0x000000F0 // PD1 Mask
#define GPIO_PCTL_PD1_S             4          // PD1 Mask
#define GPIO_PCTL_PD1_AIN6_V        0x00000000 // AIN6 on PD1
#define GPIO_PCTL_PD1_SSI3FSS_V     0x00000010 // SSI3FSS on PD1
#define GPIO_PCTL_PD1_SSI1FSS_V     0x00000020 // SSI1FSS on PD1
#define GPIO_PCTL_PD1_I2C3SDA_V     0x00000030 // I2C3SDA on PD1
#define GPIO_PCTL_PD1_M0PWM7_V      0x00000040 // M0PWM7 on PD1
#define GPIO_PCTL_PD1_M1PWM1_V      0x00000050 // M1PWM1 on PD1
#define GPIO_PCTL_PD1_WT2CCP1_V     0x00000070 // WT2CCP1 on PD1
#define GPIO_PCTL_PD0_M             0x0000000F // PD0 Mask
#define GPIO_PCTL_PD0_S             0          // PD0 Mask
#define GPIO_PCTL_PD0_AIN7_V        0x00000000 // AIN7 on PD0
#define GPIO_PCTL_PD0_SSI3CLK_V     0x00000001 // SSI3CLK on PD0
#define GPIO_PCTL_PD0_SSI1CLK_V     0x00000002 // SSI1CLK on PD0
#define GPIO_PCTL_PD0_I2C3SCL_V     0x00000003 // I2C3SCL on PD0
#define GPIO_PCTL_PD0_M0PWM6_V      0x00000004 // M0PWM6 on PD0
#define GPIO_PCTL_PD0_M1PWM0_V      0x00000005 // M1PWM0 on PD0
#define GPIO_PCTL_PD0_WT2CCP0_V     0x00000007 // WT2CCP0 on PD0
#define GPIO_PCTL_PE5_M             0x00F00000 // PE5 Mask
#define GPIO_PCTL_PE5_S             20         // PE5 Mask
#define GPIO_PCTL_PE5_AIN8_V        0x00000000 // AIN8 on PE5
#define GPIO_PCTL_PE5_U5TX_V        0x00100000 // U5TX on PE5
#define GPIO_PCTL_PE5_I2C2SDA_V     0x00300000 // I2C2SDA on PE5
#define GPIO_PCTL_PE5_M0PWM5_V      0x00400000 // M0PWM5 on PE5
#define GPIO_PCTL_PE5_M1PWM3_V      0x00500000 // M1PWM3 on PE5
#define GPIO_PCTL_PE5_CAN0TX_V      0x00800000 // CAN0TX on PE5
#define GPIO_PCTL_PE4_M             0x000F0000 // PE4 Mask
#define GPIO_PCTL_PE4_S             16         // PE4 Mask
#define GPIO_PCTL_PE4_AIN9_V        0x00000000 // AIN9 on PE4
#define GPIO_PCTL_PE4_U5RX_V        0x00010000 // U5RX on PE4
#define GPIO_PCTL_PE4_I2C2SCL_V     0x00030000 // I2C2SCL on PE4
#define GPIO_PCTL_PE4_M0PWM4_V      0x00040000 // M0PWM4 on PE4
#define GPIO_PCTL_PE4_M1PWM2_V      0x00050000 // M1PWM2 on PE4
#define GPIO_PCTL_PE4_CAN0RX_V      0x00080000 // CAN0RX on PE4
#define GPIO_PCTL_PE3_M             0x0000F000 // PE3 Mask
#define GPIO_PCTL_PE3_S             12         // PE3 Mask
#define GPIO_PCTL_PE3_AIN0_V        0x00000000 // AIN0 on PE3
#define GPIO_PCTL_PE2_M             0x00000F00 // PE2 Mask
#define GPIO_PCTL_PE2_S             8          // PE2 Mask
#define GPIO_PCTL_PE2_AIN1_V        0x00000000 // AIN1 on PE2
#define GPIO_PCTL_PE1_M             0x000000F0 // PE1 Mask
#define GPIO_PCTL_PE1_S             4          // PE1 Mask
#define GPIO_PCTL_PE1_AIN2_V        0x00000000 // AIN2 on PE1
#define GPIO_PCTL_PE1_U7TX_V        0x00000010 // U7TX on PE1
#define GPIO_PCTL_PE0_M             0x0000000F // PE0 Mask
#define GPIO_PCTL_PE0_S             0          // PE0 Mask
#define GPIO_PCTL_PE0_AIN3_V        0x00000000 // AIN3 on PE0
#define GPIO_PCTL_PE0_U7RX_V        0x00000001 // U7RX on PE0
#define GPIO_PCTL_PF4_M             0x000F0000 // PF4 Mask
#define GPIO_PCTL_PF4_S             16         // PF4 Mask
#define GPIO_PCTL_PF4_M1FAULT0_V    0x00050000 // M1FAULT0 on PF4
#define GPIO_PCTL_PF4_IDX0_V        0x00060000 // IDX0 on PF4
#define GPIO_PCTL_PF4_T2CCP0_V      0x00070000 // T2CCP0 on PF4
#define GPIO_PCTL_PF4_USB0EPEN_V    0x00080000 // USB0EPEN on PF4
#define GPIO_PCTL_PF3_M             0x0000F000 // PF3 Mask
#define GPIO_PCTL_PF3_S             12         // PF3 Mask
#define GPIO_PCTL_PF3_SSI1FSS_V     0x00002000 // SSI1FSS on PF3
#define GPIO_PCTL_PF3_CAN0TX_V      0x00003000 // CAN0TX on PF3
#define GPIO_PCTL_PF3_M1PWM7_V      0x00005000 // M1PWM7 on PF3
#define GPIO_PCTL_PF3_T1CCP1_V      0x00007000 // T1CCP1 on PF3
#define GPIO_PCTL_PF3_TRCLK_V       0x0000E000 // TRCLK on PF3
#define GPIO_PCTL_PF2_M             0x00000F00 // PF2 Mask
#define GPIO_PCTL_PF2_S             8          // PF2 Mask
#define GPIO_PCTL_PF2_SSI1CLK_V     0x00000200 // SSI1CLK on PF2
#define GPIO_PCTL_PF2_M0FAULT0_V    0x00000400 // M0FAULT0 on PF2
#define GPIO_PCTL_PF2_M1PWM6_V      0x00000500 // M1PWM6 on PF2
#define GPIO_PCTL_PF2_T1CCP0_V      0x00000700 // T1CCP0 on PF2
#define GPIO_PCTL_PF2_TRD0_V        0x00000E00 // TRD0 on PF2
#define GPIO_PCTL_PF1_M             0x000000F0 // PF1 Mask
#define GPIO_PCTL_PF1_S             4          // PF1 Mask
#define GPIO_PCTL_PF1_U1CTS_V       0x00000010 // U1CTS on PF1
#define GPIO_PCTL_PF1_SSI1TX_V      0x00000020 // SSI1TX on PF1
#define GPIO_PCTL_PF1_M1PWM5_V      0x00000050 // M1PWM5 on PF1
#define GPIO_PCTL_PF1_PHB0_V        0x00000060 // PHB0 on PF1
#define GPIO_PCTL_PF1_T0CCP1_V      0x00000070 // T0CCP1 on PF1
#define GPIO_PCTL_PF1_C1O_V         0x00000090 // C1O on PF1
#define GPIO_PCTL_PF1_TRD1_V        0x000000E0 // TRD1 on PF1
#define GPIO_PCTL_PF0_M             0x0000000F // PF0 Mask
#define GPIO_PCTL_PF0_S             0          // PF0 Mask
#define GPIO_PCTL_PF0_U1RTS_V       0x00000001 // U1RTS on PF0
#define GPIO_PCTL_PF0_SSI1RX_V      0x00000002 // SSI1RX on PF0
#define GPIO_PCTL_PF0_CAN0RX_V      0x00000003 // CAN0RX on PF0
#define GPIO_PCTL_PF0_M1PWM4_V      0x00000005 // M1PWM4 on PF0
#define GPIO_PCTL_PF0_PHA0_V        0x00000006 // PHA0 on PF0
#define GPIO_PCTL_PF0_T0CCP0_V      0x00000007 // T0CCP0 on PF0
#define GPIO_PCTL_PF0_NMI_V         0x00000008 // NMI on PF0
#define GPIO_PCTL_PF0_C0O_V         0x00000009 // C0O on PF0


/******************************************************************************/
/*                                                                            */
/*                      SSI                                                   */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register CR0 of Module SSI                                   */
#define SSI_CR0_SCR_M               0x0000FF00 // SSI Serial Clock Rate
#define SSI_CR0_SCR_S               8          // SSI Serial Clock Rate
#define SSI_CR0_SPH_B               0x00000080 // SSI Serial Clock Phase
#define SSI_CR0_SPO_B               0x00000040 // SSI Serial Clock Polarity
#define SSI_CR0_FRF_M               0x00000030 // SSI Frame Format Select
#define SSI_CR0_FRF_S               4          // SSI Frame Format Select
#define SSI_CR0_FRF_MOTO_V          0x00000000 // Freescale SPI Frame Format
#define SSI_CR0_FRF_TI_V            0x00000010 // Synchronous Serial Frame Format
#define SSI_CR0_FRF_NMW_V           0x00000020 // MICROWIRE Frame Format
#define SSI_CR0_DSS_M               0x0000000F // SSI Data Size Select
#define SSI_CR0_DSS_S               0          // SSI Data Size Select
#define SSI_CR0_DSS_4_V             0x00000003 // 4-bit data
#define SSI_CR0_DSS_5_V             0x00000004 // 5-bit data
#define SSI_CR0_DSS_6_V             0x00000005 // 6-bit data
#define SSI_CR0_DSS_7_V             0x00000006 // 7-bit data
#define SSI_CR0_DSS_8_V             0x00000007 // 8-bit data
#define SSI_CR0_DSS_9_V             0x00000008 // 9-bit data
#define SSI_CR0_DSS_10_V            0x00000009 // 10-bit data
#define SSI_CR0_DSS_11_V            0x0000000A // 11-bit data
#define SSI_CR0_DSS_12_V            0x0000000B // 12-bit data
#define SSI_CR0_DSS_13_V            0x0000000C // 13-bit data
#define SSI_CR0_DSS_14_V            0x0000000D // 14-bit data
#define SSI_CR0_DSS_15_V            0x0000000E // 15-bit data
#define SSI_CR0_DSS_16_V            0x0000000F // 16-bit data

/* Bit/Fields in Register CR1 of Module SSI                                   */
#define SSI_CR1_EOT_B               0x00000010 // End of Transmission
#define SSI_CR1_MS_B                0x00000004 // SSI Master/Slave Select
#define SSI_CR1_SSE_B               0x00000002 // SSI Synchronous Serial Port
#define SSI_CR1_LBM_B               0x00000001 // SSI Loopback Mode

/* Bit/Fields in Register DR of Module SSI                                    */
#define SSI_DR_DATA_M               0x0000FFFF // SSI Receive/Transmit Data
#define SSI_DR_DATA_S               0          // SSI Receive/Transmit Data

/* Bit/Fields in Register SR of Module SSI                                    */
#define SSI_SR_BSY_B                0x00000010 // SSI Busy Bit
#define SSI_SR_RFF_B                0x00000008 // SSI Receive FIFO Full
#define SSI_SR_RNE_B                0x00000004 // SSI Receive FIFO Not Empty
#define SSI_SR_TNF_B                0x00000002 // SSI Transmit FIFO Not Full
#define SSI_SR_TFE_B                0x00000001 // SSI Transmit FIFO Empty

/* Bit/Fields in Register CPSR of Module SSI                                  */
#define SSI_CPSR_CPSDVSR_M          0x000000FF // SSI Clock Prescale Divisor
#define SSI_CPSR_CPSDVSR_S          0          // SSI Clock Prescale Divisor

/* Bit/Fields in Register IM of Module SSI                                    */
#define SSI_IM_TXIM_B               0x00000008 // SSI Transmit FIFO Interrupt Mask
#define SSI_IM_RXIM_B               0x00000004 // SSI Receive FIFO Interrupt Mask
#define SSI_IM_RTIM_B               0x00000002 // SSI Receive Time-Out Interrupt
#define SSI_IM_RORIM_B              0x00000001 // SSI Receive Overrun Interrupt

/* Bit/Fields in Register RIS of Module SSI                                   */
#define SSI_RIS_TXRIS_B             0x00000008 // SSI Transmit FIFO Raw Interrupt
#define SSI_RIS_RXRIS_B             0x00000004 // SSI Receive FIFO Raw Interrupt
#define SSI_RIS_RTRIS_B             0x00000002 // SSI Receive Time-Out Raw
#define SSI_RIS_RORRIS_B            0x00000001 // SSI Receive Overrun Raw

/* Bit/Fields in Register MIS of Module SSI                                   */
#define SSI_MIS_TXMIS_B             0x00000008 // SSI Transmit FIFO Masked
#define SSI_MIS_RXMIS_B             0x00000004 // SSI Receive FIFO Masked
#define SSI_MIS_RTMIS_B             0x00000002 // SSI Receive Time-Out Masked
#define SSI_MIS_RORMIS_B            0x00000001 // SSI Receive Overrun Masked

/* Bit/Fields in Register ICR of Module SSI                                   */
#define SSI_ICR_RTIC_B              0x00000002 // SSI Receive Time-Out Interrupt
#define SSI_ICR_RORIC_B             0x00000001 // SSI Receive Overrun Interrupt

/* Bit/Fields in Register DMACTL of Module SSI                                */
#define SSI_DMACTL_TXDMAE_B         0x00000002 // Transmit DMA Enable
#define SSI_DMACTL_RXDMAE_B         0x00000001 // Receive DMA Enable

/* Bit/Fields in Register CC of Module SSI                                    */
#define SSI_CC_CS_M                 0x0000000F // SSI Baud Clock Source
#define SSI_CC_CS_S                 0          // SSI Baud Clock Source
#define SSI_CC_CS_SYSPLL_V          0x00000000 // System clock (based on clock
#define SSI_CC_CS_PIOSC_V           0x00000005 // PIOSC


/******************************************************************************/
/*                                                                            */
/*                      UART                                                  */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register DR of Module UART                                   */
#define UART_DR_OE_B                0x00000800 // UART Overrun Error
#define UART_DR_BE_B                0x00000400 // UART Break Error
#define UART_DR_PE_B                0x00000200 // UART Parity Error
#define UART_DR_FE_B                0x00000100 // UART Framing Error
#define UART_DR_DATA_M              0x000000FF // Data Transmitted or Received
#define UART_DR_DATA_S              0          // Data Transmitted or Received

/* Bit/Fields in Register RSR of Module UART                                  */
#define UART_RSR_OE_B               0x00000008 // UART Overrun Error
#define UART_RSR_BE_B               0x00000004 // UART Break Error
#define UART_RSR_PE_B               0x00000002 // UART Parity Error
#define UART_RSR_FE_B               0x00000001 // UART Framing Error

/* Bit/Fields in Register ECR of Module UART                                  */
#define UART_ECR_DATA_M             0x000000FF // Error Clear
#define UART_ECR_DATA_S             0          // Error Clear

/* Bit/Fields in Register FR of Module UART                                   */
#define UART_FR_TXFE_B              0x00000080 // UART Transmit FIFO Empty
#define UART_FR_RXFF_B              0x00000040 // UART Receive FIFO Full
#define UART_FR_TXFF_B              0x00000020 // UART Transmit FIFO Full
#define UART_FR_RXFE_B              0x00000010 // UART Receive FIFO Empty
#define UART_FR_BUSY_B              0x00000008 // UART Busy
#define UART_FR_CTS_B               0x00000001 // Clear To Send

/* Bit/Fields in Register ILPR of Module UART                                 */
#define UART_ILPR_ILPDVSR_M         0x000000FF // IrDA Low-Power Divisor
#define UART_ILPR_ILPDVSR_S         0          // IrDA Low-Power Divisor

/* Bit/Fields in Register IBRD of Module UART                                 */
#define UART_IBRD_DIVINT_M          0x0000FFFF // Integer Baud-Rate Divisor
#define UART_IBRD_DIVINT_S          0          // Integer Baud-Rate Divisor

/* Bit/Fields in Register FBRD of Module UART                                 */
#define UART_FBRD_DIVFRAC_M         0x0000003F // Fractional Baud-Rate Divisor
#define UART_FBRD_DIVFRAC_S         0          // Fractional Baud-Rate Divisor

/* Bit/Fields in Register LCRH of Module UART                                 */
#define UART_LCRH_SPS_B             0x00000080 // UART Stick Parity Select
#define UART_LCRH_WLEN_M            0x00000060 // UART Word Length
#define UART_LCRH_WLEN_S            5          // UART Word Length
#define UART_LCRH_WLEN_5_V          0x00000000 // 5 bits (default)
#define UART_LCRH_WLEN_6_V          0x00000020 // 6 bits
#define UART_LCRH_WLEN_7_V          0x00000040 // 7 bits
#define UART_LCRH_WLEN_8_V          0x00000060 // 8 bits
#define UART_LCRH_FEN_B             0x00000010 // UART Enable FIFOs
#define UART_LCRH_STP2_B            0x00000008 // UART Two Stop Bits Select
#define UART_LCRH_EPS_B             0x00000004 // UART Even Parity Select
#define UART_LCRH_PEN_B             0x00000002 // UART Parity Enable
#define UART_LCRH_BRK_B             0x00000001 // UART Send Break

/* Bit/Fields in Register CTL of Module UART                                  */
#define UART_CTL_CTSEN_B            0x00008000 // Enable Clear To Send
#define UART_CTL_RTSEN_B            0x00004000 // Enable Request to Send
#define UART_CTL_RTS_B              0x00000800 // Request to Send
#define UART_CTL_RXE_B              0x00000200 // UART Receive Enable
#define UART_CTL_TXE_B              0x00000100 // UART Transmit Enable
#define UART_CTL_LBE_B              0x00000080 // UART Loop Back Enable
#define UART_CTL_HSE_B              0x00000020 // High-Speed Enable
#define UART_CTL_EOT_B              0x00000010 // End of Transmission
#define UART_CTL_SMART_B            0x00000008 // ISO 7816 Smart Card Support
#define UART_CTL_SIRLP_B            0x00000004 // UART SIR Low-Power Mode
#define UART_CTL_SIREN_B            0x00000002 // UART SIR Enable
#define UART_CTL_UARTEN_B           0x00000001 // UART Enable

/* Bit/Fields in Register IFLS of Module UART                                 */
#define UART_IFLS_RX_M              0x00000038 // UART Receive Interrupt FIFO
#define UART_IFLS_RX_S              3          // UART Receive Interrupt FIFO
#define UART_IFLS_RX1_8_V           0x00000000 // RX FIFO >= 1/8 full
#define UART_IFLS_RX2_8_V           0x00000008 // RX FIFO >= 1/4 full
#define UART_IFLS_RX4_8_V           0x00000010 // RX FIFO >= 1/2 full (default)
#define UART_IFLS_RX6_8_V           0x00000018 // RX FIFO >= 3/4 full
#define UART_IFLS_RX7_8_V           0x00000020 // RX FIFO >= 7/8 full
#define UART_IFLS_TX_M              0x00000007 // UART Transmit Interrupt FIFO
#define UART_IFLS_TX_S              0          // UART Transmit Interrupt FIFO
#define UART_IFLS_TX1_8_V           0x00000000 // TX FIFO <= 1/8 full
#define UART_IFLS_TX2_8_V           0x00000001 // TX FIFO <= 1/4 full
#define UART_IFLS_TX4_8_V           0x00000002 // TX FIFO <= 1/2 full (default)
#define UART_IFLS_TX6_8_V           0x00000003 // TX FIFO <= 3/4 full
#define UART_IFLS_TX7_8_V           0x00000004 // TX FIFO <= 7/8 full

/* Bit/Fields in Register IM of Module UART                                   */
#define UART_IM_9BITIM_B            0x00001000 // 9-Bit Mode Interrupt Mask
#define UART_IM_OEIM_B              0x00000400 // UART Overrun Error Interrupt
#define UART_IM_BEIM_B              0x00000200 // UART Break Error Interrupt Mask
#define UART_IM_PEIM_B              0x00000100 // UART Parity Error Interrupt Mask
#define UART_IM_FEIM_B              0x00000080 // UART Framing Error Interrupt
#define UART_IM_RTIM_B              0x00000040 // UART Receive Time-Out Interrupt
#define UART_IM_TXIM_B              0x00000020 // UART Transmit Interrupt Mask
#define UART_IM_RXIM_B              0x00000010 // UART Receive Interrupt Mask
#define UART_IM_CTSMIM_B            0x00000002 // UART Clear to Send Modem

/* Bit/Fields in Register RIS of Module UART                                  */
#define UART_RIS_9BITRIS_B          0x00001000 // 9-Bit Mode Raw Interrupt Status
#define UART_RIS_OERIS_B            0x00000400 // UART Overrun Error Raw Interrupt
#define UART_RIS_BERIS_B            0x00000200 // UART Break Error Raw Interrupt
#define UART_RIS_PERIS_B            0x00000100 // UART Parity Error Raw Interrupt
#define UART_RIS_FERIS_B            0x00000080 // UART Framing Error Raw Interrupt
#define UART_RIS_RTRIS_B            0x00000040 // UART Receive Time-Out Raw
#define UART_RIS_TXRIS_B            0x00000020 // UART Transmit Raw Interrupt
#define UART_RIS_RXRIS_B            0x00000010 // UART Receive Raw Interrupt
#define UART_RIS_CTSRIS_B           0x00000002 // UART Clear to Send Modem Raw

/* Bit/Fields in Register MIS of Module UART                                  */
#define UART_MIS_9BITMIS_B          0x00001000 // 9-Bit Mode Masked Interrupt
#define UART_MIS_OEMIS_B            0x00000400 // UART Overrun Error Masked
#define UART_MIS_BEMIS_B            0x00000200 // UART Break Error Masked
#define UART_MIS_PEMIS_B            0x00000100 // UART Parity Error Masked
#define UART_MIS_FEMIS_B            0x00000080 // UART Framing Error Masked
#define UART_MIS_RTMIS_B            0x00000040 // UART Receive Time-Out Masked
#define UART_MIS_TXMIS_B            0x00000020 // UART Transmit Masked Interrupt
#define UART_MIS_RXMIS_B            0x00000010 // UART Receive Masked Interrupt
#define UART_MIS_CTSMIS_B           0x00000002 // UART Clear to Send Modem Masked

/* Bit/Fields in Register ICR of Module UART                                  */
#define UART_ICR_9BITIC_B           0x00001000 // 9-Bit Mode Interrupt Clear
#define UART_ICR_OEIC_B             0x00000400 // Overrun Error Interrupt Clear
#define UART_ICR_BEIC_B             0x00000200 // Break Error Interrupt Clear
#define UART_ICR_PEIC_B             0x00000100 // Parity Error Interrupt Clear
#define UART_ICR_FEIC_B             0x00000080 // Framing Error Interrupt Clear
#define UART_ICR_RTIC_B             0x00000040 // Receive Time-Out Interrupt Clear
#define UART_ICR_TXIC_B             0x00000020 // Transmit Interrupt Clear
#define UART_ICR_RXIC_B             0x00000010 // Receive Interrupt Clear
#define UART_ICR_CTSMIC_B           0x00000002 // UART Clear to Send Modem

/* Bit/Fields in Register DMACTL of Module UART                               */
#define UART_DMACTL_DMAERR_B        0x00000004 // DMA on Error
#define UART_DMACTL_TXDMAE_B        0x00000002 // Transmit DMA Enable
#define UART_DMACTL_RXDMAE_B        0x00000001 // Receive DMA Enable

/* Bit/Fields in Register 9BITADDR of Module UART                             */
#define UART_9BITADDR_9BITEN_B      0x00008000 // Enable 9-Bit Mode
#define UART_9BITADDR_ADDR_M        0x000000FF // Self Address for 9-Bit Mode
#define UART_9BITADDR_ADDR_S        0          // Self Address for 9-Bit Mode

/* Bit/Fields in Register 9BITAMASK of Module UART                            */
#define UART_9BITAMASK_MASK_M       0x000000FF // Self Address Mask for 9-Bit Mode
#define UART_9BITAMASK_MASK_S       0          // Self Address Mask for 9-Bit Mode

/* Bit/Fields in Register PP of Module UART                                   */
#define UART_PP_NB_B                0x00000002 // 9-Bit Support
#define UART_PP_SC_B                0x00000001 // Smart Card Support

/* Bit/Fields in Register CC of Module UART                                   */
#define UART_CC_CS_M                0x0000000F // UART Baud Clock Source
#define UART_CC_CS_S                0          // UART Baud Clock Source
#define UART_CC_CS_SYSCLK_V         0x00000000 // System clock (based on clock
#define UART_CC_CS_PIOSC_V          0x00000005 // PIOSC


/******************************************************************************/
/*                                                                            */
/*                      I2C                                                   */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register MSA of Module I2C                                   */
#define I2C_MSA_SA_M                0x000000FE // I2C Slave Address
#define I2C_MSA_SA_S                1          // I2C Slave Address
#define I2C_MSA_RS_B                0x00000001 // Receive not send

/* Bit/Fields in Register MCS of Module I2C                                   */
#define I2C_MCS_CLKTO_B             0x00000080 // Clock Timeout Error
#define I2C_MCS_BUSBSY_B            0x00000040 // Bus Busy
#define I2C_MCS_IDLE_B              0x00000020 // I2C Idle
#define I2C_MCS_ARBLST_B            0x00000010 // Arbitration Lost
#define I2C_MCS_HS_B                0x00000010 // High-Speed Enable
#define I2C_MCS_ACK_B               0x00000008 // Data Acknowledge Enable
#define I2C_MCS_DATACK_B            0x00000008 // Acknowledge Data
#define I2C_MCS_ADRACK_B            0x00000004 // Acknowledge Address
#define I2C_MCS_STOP_B              0x00000004 // Generate STOP
#define I2C_MCS_ERROR_B             0x00000002 // Error
#define I2C_MCS_START_B             0x00000002 // Generate START
#define I2C_MCS_RUN_B               0x00000001 // I2C Master Enable
#define I2C_MCS_BUSY_B              0x00000001 // I2C Busy

/* Bit/Fields in Register MDR of Module I2C                                   */
#define I2C_MDR_DATA_M              0x000000FF // This byte contains the data
#define I2C_MDR_DATA_S              0          // This byte contains the data

/* Bit/Fields in Register MTPR of Module I2C                                  */
#define I2C_MTPR_HS_B               0x00000080 // High-Speed Enable
#define I2C_MTPR_TPR_M              0x0000007F // Timer Period
#define I2C_MTPR_TPR_S              0          // Timer Period

/* Bit/Fields in Register MIMR of Module I2C                                  */
#define I2C_MIMR_CLKIM_B            0x00000002 // Clock Timeout Interrupt Mask
#define I2C_MIMR_IM_B               0x00000001 // Master Interrupt Mask

/* Bit/Fields in Register MRIS of Module I2C                                  */
#define I2C_MRIS_CLKRIS_B           0x00000002 // Clock Timeout Raw Interrupt
#define I2C_MRIS_RIS_B              0x00000001 // Master Raw Interrupt Status

/* Bit/Fields in Register MMIS of Module I2C                                  */
#define I2C_MMIS_CLKMIS_B           0x00000002 // Clock Timeout Masked Interrupt
#define I2C_MMIS_MIS_B              0x00000001 // Masked Interrupt Status

/* Bit/Fields in Register MICR of Module I2C                                  */
#define I2C_MICR_CLKIC_B            0x00000002 // Clock Timeout Interrupt Clear
#define I2C_MICR_IC_B               0x00000001 // Master Interrupt Clear

/* Bit/Fields in Register MCR of Module I2C                                   */
#define I2C_MCR_GFE_B               0x00000040 // I2C Glitch Filter Enable
#define I2C_MCR_SFE_B               0x00000020 // I2C Slave Function Enable
#define I2C_MCR_MFE_B               0x00000010 // I2C Master Function Enable
#define I2C_MCR_LPBK_B              0x00000001 // I2C Loopback

/* Bit/Fields in Register MCLKOCNT of Module I2C                              */
#define I2C_MCLKOCNT_CNTL_M         0x000000FF // I2C Master Count
#define I2C_MCLKOCNT_CNTL_S         0          // I2C Master Count

/* Bit/Fields in Register MBMON of Module I2C                                 */
#define I2C_MBMON_SDA_B             0x00000002 // I2C SDA Status
#define I2C_MBMON_SCL_B             0x00000001 // I2C SCL Status

/* Bit/Fields in Register MCR2 of Module I2C                                  */
#define I2C_MCR2_GFPW_M             0x00000070 // I2C Glitch Filter Pulse Width
#define I2C_MCR2_GFPW_S             4          // I2C Glitch Filter Pulse Width
#define I2C_MCR2_GFPW_BYPASS_V      0x00000000 // Bypass
#define I2C_MCR2_GFPW_1_V           0x00000010 // 1 clock
#define I2C_MCR2_GFPW_2_V           0x00000020 // 2 clocks
#define I2C_MCR2_GFPW_3_V           0x00000030 // 3 clocks
#define I2C_MCR2_GFPW_4_V           0x00000040 // 4 clocks
#define I2C_MCR2_GFPW_8_V           0x00000050 // 8 clocks
#define I2C_MCR2_GFPW_16_V          0x00000060 // 16 clocks
#define I2C_MCR2_GFPW_31_V          0x00000070 // 31 clocks

/* Bit/Fields in Register SOAR of Module I2C                                  */
#define I2C_SOAR_OAR_M              0x0000007F // I2C Slave Own Address
#define I2C_SOAR_OAR_S              0          // I2C Slave Own Address

/* Bit/Fields in Register SCSR of Module I2C                                  */
#define I2C_SCSR_OAR2SEL_B          0x00000008 // OAR2 Address Matched
#define I2C_SCSR_FBR_B              0x00000004 // First Byte Received
#define I2C_SCSR_TREQ_B             0x00000002 // Transmit Request
#define I2C_SCSR_DA_B               0x00000001 // Device Active
#define I2C_SCSR_RREQ_B             0x00000001 // Receive Request

/* Bit/Fields in Register SDR of Module I2C                                   */
#define I2C_SDR_DATA_M              0x000000FF // Data for Transfer
#define I2C_SDR_DATA_S              0          // Data for Transfer

/* Bit/Fields in Register SIMR of Module I2C                                  */
#define I2C_SIMR_STOPIM_B           0x00000004 // Stop Condition Interrupt Mask
#define I2C_SIMR_STARTIM_B          0x00000002 // Start Condition Interrupt Mask
#define I2C_SIMR_DATAIM_B           0x00000001 // Data Interrupt Mask

/* Bit/Fields in Register SRIS of Module I2C                                  */
#define I2C_SRIS_STOPRIS_B          0x00000004 // Stop Condition Raw Interrupt
#define I2C_SRIS_STARTRIS_B         0x00000002 // Start Condition Raw Interrupt
#define I2C_SRIS_DATARIS_B          0x00000001 // Data Raw Interrupt Status

/* Bit/Fields in Register SMIS of Module I2C                                  */
#define I2C_SMIS_STOPMIS_B          0x00000004 // Stop Condition Masked Interrupt
#define I2C_SMIS_STARTMIS_B         0x00000002 // Start Condition Masked Interrupt
#define I2C_SMIS_DATAMIS_B          0x00000001 // Data Masked Interrupt Status

/* Bit/Fields in Register SICR of Module I2C                                  */
#define I2C_SICR_STOPIC_B           0x00000004 // Stop Condition Interrupt Clear
#define I2C_SICR_STARTIC_B          0x00000002 // Start Condition Interrupt Clear
#define I2C_SICR_DATAIC_B           0x00000001 // Data Interrupt Clear

/* Bit/Fields in Register SOAR2 of Module I2C                                 */
#define I2C_SOAR2_OAR2EN_B          0x00000080 // I2C Slave Own Address 2 Enable
#define I2C_SOAR2_OAR2_M            0x0000007F // I2C Slave Own Address 2
#define I2C_SOAR2_OAR2_S            0          // I2C Slave Own Address 2

/* Bit/Fields in Register SACKCTL of Module I2C                               */
#define I2C_SACKCTL_ACKOVAL_B       0x00000002 // I2C Slave ACK Override Value
#define I2C_SACKCTL_ACKOEN_B        0x00000001 // I2C Slave ACK Override Enable

/* Bit/Fields in Register PP of Module I2C                                    */
#define I2C_PP_HS_B                 0x00000001 // High-Speed Capable

/* Bit/Fields in Register PC of Module I2C                                    */
#define I2C_PC_HS_B                 0x00000001 // High-Speed Capable


/******************************************************************************/
/*                                                                            */
/*                      PWM                                                   */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register CTL of Module PWM                                   */
#define PWM_CTL_GLOBALSYNC3_B       0x00000008 // Update PWM Generator 3
#define PWM_CTL_GLOBALSYNC2_B       0x00000004 // Update PWM Generator 2
#define PWM_CTL_GLOBALSYNC1_B       0x00000002 // Update PWM Generator 1
#define PWM_CTL_GLOBALSYNC0_B       0x00000001 // Update PWM Generator 0

/* Bit/Fields in Register SYNC of Module PWM                                  */
#define PWM_SYNC_SYNC3_B            0x00000008 // Reset Generator 3 Counter
#define PWM_SYNC_SYNC2_B            0x00000004 // Reset Generator 2 Counter
#define PWM_SYNC_SYNC1_B            0x00000002 // Reset Generator 1 Counter
#define PWM_SYNC_SYNC0_B            0x00000001 // Reset Generator 0 Counter

/* Bit/Fields in Register ENABLE of Module PWM                                */
#define PWM_ENABLE_PWM7EN_B         0x00000080 // MnPWM7 Output Enable
#define PWM_ENABLE_PWM6EN_B         0x00000040 // MnPWM6 Output Enable
#define PWM_ENABLE_PWM5EN_B         0x00000020 // MnPWM5 Output Enable
#define PWM_ENABLE_PWM4EN_B         0x00000010 // MnPWM4 Output Enable
#define PWM_ENABLE_PWM3EN_B         0x00000008 // MnPWM3 Output Enable
#define PWM_ENABLE_PWM2EN_B         0x00000004 // MnPWM2 Output Enable
#define PWM_ENABLE_PWM1EN_B         0x00000002 // MnPWM1 Output Enable
#define PWM_ENABLE_PWM0EN_B         0x00000001 // MnPWM0 Output Enable

/* Bit/Fields in Register INVERT of Module PWM                                */
#define PWM_INVERT_PWM7INV_B        0x00000080 // Invert MnPWM7 Signal
#define PWM_INVERT_PWM6INV_B        0x00000040 // Invert MnPWM6 Signal
#define PWM_INVERT_PWM5INV_B        0x00000020 // Invert MnPWM5 Signal
#define PWM_INVERT_PWM4INV_B        0x00000010 // Invert MnPWM4 Signal
#define PWM_INVERT_PWM3INV_B        0x00000008 // Invert MnPWM3 Signal
#define PWM_INVERT_PWM2INV_B        0x00000004 // Invert MnPWM2 Signal
#define PWM_INVERT_PWM1INV_B        0x00000002 // Invert MnPWM1 Signal
#define PWM_INVERT_PWM0INV_B        0x00000001 // Invert MnPWM0 Signal

/* Bit/Fields in Register FAULT of Module PWM                                 */
#define PWM_FAULT_FAULT7_B          0x00000080 // MnPWM7 Fault
#define PWM_FAULT_FAULT6_B          0x00000040 // MnPWM6 Fault
#define PWM_FAULT_FAULT5_B          0x00000020 // MnPWM5 Fault
#define PWM_FAULT_FAULT4_B          0x00000010 // MnPWM4 Fault
#define PWM_FAULT_FAULT3_B          0x00000008 // MnPWM3 Fault
#define PWM_FAULT_FAULT2_B          0x00000004 // MnPWM2 Fault
#define PWM_FAULT_FAULT1_B          0x00000002 // MnPWM1 Fault
#define PWM_FAULT_FAULT0_B          0x00000001 // MnPWM0 Fault

/* Bit/Fields in Register INTEN of Module PWM                                 */
#define PWM_INTEN_INTFAULT1_B       0x00020000 // Interrupt Fault 1
#define PWM_INTEN_INTFAULT0_B       0x00010000 // Interrupt Fault 0
#define PWM_INTEN_INTPWM3_B         0x00000008 // PWM3 Interrupt Enable
#define PWM_INTEN_INTPWM2_B         0x00000004 // PWM2 Interrupt Enable
#define PWM_INTEN_INTPWM1_B         0x00000002 // PWM1 Interrupt Enable
#define PWM_INTEN_INTPWM0_B         0x00000001 // PWM0 Interrupt Enable

/* Bit/Fields in Register RIS of Module PWM                                   */
#define PWM_RIS_INTFAULT1_B         0x00020000 // Interrupt Fault PWM 1
#define PWM_RIS_INTFAULT0_B         0x00010000 // Interrupt Fault PWM 0
#define PWM_RIS_INTPWM3_B           0x00000008 // PWM3 Interrupt Asserted
#define PWM_RIS_INTPWM2_B           0x00000004 // PWM2 Interrupt Asserted
#define PWM_RIS_INTPWM1_B           0x00000002 // PWM1 Interrupt Asserted
#define PWM_RIS_INTPWM0_B           0x00000001 // PWM0 Interrupt Asserted

/* Bit/Fields in Register ISC of Module PWM                                   */
#define PWM_ISC_INTFAULT1_B         0x00020000 // FAULT1 Interrupt Asserted
#define PWM_ISC_INTFAULT0_B         0x00010000 // FAULT0 Interrupt Asserted
#define PWM_ISC_INTPWM3_B           0x00000008 // PWM3 Interrupt Status
#define PWM_ISC_INTPWM2_B           0x00000004 // PWM2 Interrupt Status
#define PWM_ISC_INTPWM1_B           0x00000002 // PWM1 Interrupt Status
#define PWM_ISC_INTPWM0_B           0x00000001 // PWM0 Interrupt Status

/* Bit/Fields in Register STATUS of Module PWM                                */
#define PWM_STATUS_FAULT1_B         0x00000002 // Generator 1 Fault Status
#define PWM_STATUS_FAULT0_B         0x00000001 // Generator 0 Fault Status

/* Bit/Fields in Register FAULTVAL of Module PWM                              */
#define PWM_FAULTVAL_PWM7_B         0x00000080 // MnPWM7 Fault Value
#define PWM_FAULTVAL_PWM6_B         0x00000040 // MnPWM6 Fault Value
#define PWM_FAULTVAL_PWM5_B         0x00000020 // MnPWM5 Fault Value
#define PWM_FAULTVAL_PWM4_B         0x00000010 // MnPWM4 Fault Value
#define PWM_FAULTVAL_PWM3_B         0x00000008 // MnPWM3 Fault Value
#define PWM_FAULTVAL_PWM2_B         0x00000004 // MnPWM2 Fault Value
#define PWM_FAULTVAL_PWM1_B         0x00000002 // MnPWM1 Fault Value
#define PWM_FAULTVAL_PWM0_B         0x00000001 // MnPWM0 Fault Value

/* Bit/Fields in Register ENUPD of Module PWM                                 */
#define PWM_ENUPD_ENUPD7_M          0x0000C000 // MnPWM7 Enable Update Mode
#define PWM_ENUPD_ENUPD7_S          14         // MnPWM7 Enable Update Mode
#define PWM_ENUPD_ENUPD7_IMM_V      0x00000000 // Immediate
#define PWM_ENUPD_ENUPD7_LSYNC_V    0x00008000 // Locally Synchronized
#define PWM_ENUPD_ENUPD7_GSYNC_V    0x0000C000 // Globally Synchronized
#define PWM_ENUPD_ENUPD6_M          0x00003000 // MnPWM6 Enable Update Mode
#define PWM_ENUPD_ENUPD6_S          12         // MnPWM6 Enable Update Mode
#define PWM_ENUPD_ENUPD6_IMM_V      0x00000000 // Immediate
#define PWM_ENUPD_ENUPD6_LSYNC_V    0x00002000 // Locally Synchronized
#define PWM_ENUPD_ENUPD6_GSYNC_V    0x00003000 // Globally Synchronized
#define PWM_ENUPD_ENUPD5_M          0x00000C00 // MnPWM5 Enable Update Mode
#define PWM_ENUPD_ENUPD5_S          10         // MnPWM5 Enable Update Mode
#define PWM_ENUPD_ENUPD5_IMM_V      0x00000000 // Immediate
#define PWM_ENUPD_ENUPD5_LSYNC_V    0x00000800 // Locally Synchronized
#define PWM_ENUPD_ENUPD5_GSYNC_V    0x00000C00 // Globally Synchronized
#define PWM_ENUPD_ENUPD4_M          0x00000300 // MnPWM4 Enable Update Mode
#define PWM_ENUPD_ENUPD4_S          8          // MnPWM4 Enable Update Mode
#define PWM_ENUPD_ENUPD4_IMM_V      0x00000000 // Immediate
#define PWM_ENUPD_ENUPD4_LSYNC_V    0x00000200 // Locally Synchronized
#define PWM_ENUPD_ENUPD4_GSYNC_V    0x00000300 // Globally Synchronized
#define PWM_ENUPD_ENUPD3_M          0x000000C0 // MnPWM3 Enable Update Mode
#define PWM_ENUPD_ENUPD3_S          6          // MnPWM3 Enable Update Mode
#define PWM_ENUPD_ENUPD3_IMM_V      0x00000000 // Immediate
#define PWM_ENUPD_ENUPD3_LSYNC_V    0x00000080 // Locally Synchronized
#define PWM_ENUPD_ENUPD3_GSYNC_V    0x000000C0 // Globally Synchronized
#define PWM_ENUPD_ENUPD2_M          0x00000030 // MnPWM2 Enable Update Mode
#define PWM_ENUPD_ENUPD2_S          4          // MnPWM2 Enable Update Mode
#define PWM_ENUPD_ENUPD2_IMM_V      0x00000000 // Immediate
#define PWM_ENUPD_ENUPD2_LSYNC_V    0x00000020 // Locally Synchronized
#define PWM_ENUPD_ENUPD2_GSYNC_V    0x00000030 // Globally Synchronized
#define PWM_ENUPD_ENUPD1_M          0x0000000C // MnPWM1 Enable Update Mode
#define PWM_ENUPD_ENUPD1_S          2          // MnPWM1 Enable Update Mode
#define PWM_ENUPD_ENUPD1_IMM_V      0x00000000 // Immediate
#define PWM_ENUPD_ENUPD1_LSYNC_V    0x00000008 // Locally Synchronized
#define PWM_ENUPD_ENUPD1_GSYNC_V    0x0000000C // Globally Synchronized
#define PWM_ENUPD_ENUPD0_M          0x00000003 // MnPWM0 Enable Update Mode
#define PWM_ENUPD_ENUPD0_S          0          // MnPWM0 Enable Update Mode
#define PWM_ENUPD_ENUPD0_IMM_V      0x00000000 // Immediate
#define PWM_ENUPD_ENUPD0_LSYNC_V    0x00000002 // Locally Synchronized
#define PWM_ENUPD_ENUPD0_GSYNC_V    0x00000003 // Globally Synchronized

/* Bit/Fields in Register 0 of Module PWM                                     */
#define PWM_0_CTL_LATCH_B           0x00040000 // Latch Fault Input
#define PWM_0_CTL_MINFLTPER_B       0x00020000 // Minimum Fault Period
#define PWM_0_CTL_FLTSRC_B          0x00010000 // Fault Condition Source
#define PWM_0_CTL_DBFALLUPD_M       0x0000C000 // PWMnDBFALL Update Mode
#define PWM_0_CTL_DBFALLUPD_S       14         // PWMnDBFALL Update Mode
#define PWM_0_CTL_DBFALLUPD_I_V     0x00000000 // Immediate
#define PWM_0_CTL_DBFALLUPD_LS_V    0x00008000 // Locally Synchronized
#define PWM_0_CTL_DBFALLUPD_GS_V    0x0000C000 // Globally Synchronized
#define PWM_0_CTL_DBRISEUPD_M       0x00003000 // PWMnDBRISE Update Mode
#define PWM_0_CTL_DBRISEUPD_S       12         // PWMnDBRISE Update Mode
#define PWM_0_CTL_DBRISEUPD_I_V     0x00000000 // Immediate
#define PWM_0_CTL_DBRISEUPD_LS_V    0x00002000 // Locally Synchronized
#define PWM_0_CTL_DBRISEUPD_GS_V    0x00003000 // Globally Synchronized
#define PWM_0_CTL_DBCTLUPD_M        0x00000C00 // PWMnDBCTL Update Mode
#define PWM_0_CTL_DBCTLUPD_S        10         // PWMnDBCTL Update Mode
#define PWM_0_CTL_DBCTLUPD_I_V      0x00000000 // Immediate
#define PWM_0_CTL_DBCTLUPD_LS_V     0x00000800 // Locally Synchronized
#define PWM_0_CTL_DBCTLUPD_GS_V     0x00000C00 // Globally Synchronized
#define PWM_0_CTL_GENBUPD_M         0x00000300 // PWMnGENB Update Mode
#define PWM_0_CTL_GENBUPD_S         8          // PWMnGENB Update Mode
#define PWM_0_CTL_GENBUPD_I_V       0x00000000 // Immediate
#define PWM_0_CTL_GENBUPD_LS_V      0x00000200 // Locally Synchronized
#define PWM_0_CTL_GENBUPD_GS_V      0x00000300 // Globally Synchronized
#define PWM_0_CTL_GENAUPD_M         0x000000C0 // PWMnGENA Update Mode
#define PWM_0_CTL_GENAUPD_S         6          // PWMnGENA Update Mode
#define PWM_0_CTL_GENAUPD_I_V       0x00000000 // Immediate
#define PWM_0_CTL_GENAUPD_LS_V      0x00000080 // Locally Synchronized
#define PWM_0_CTL_GENAUPD_GS_V      0x000000C0 // Globally Synchronized
#define PWM_0_CTL_CMPBUPD_B         0x00000020 // Comparator B Update Mode
#define PWM_0_CTL_CMPAUPD_B         0x00000010 // Comparator A Update Mode
#define PWM_0_CTL_LOADUPD_B         0x00000008 // Load Register Update Mode
#define PWM_0_CTL_DEBUG_B           0x00000004 // Debug Mode
#define PWM_0_CTL_MODE_B            0x00000002 // Counter Mode
#define PWM_0_CTL_ENABLE_B          0x00000001 // PWM Block Enable
#define PWM_0_INTEN_TRCMPBD_B       0x00002000 // Trigger for Counter=PWMnCMPB
#define PWM_0_INTEN_TRCMPBU_B       0x00001000 // Trigger for Counter=PWMnCMPB Up
#define PWM_0_INTEN_TRCMPAD_B       0x00000800 // Trigger for Counter=PWMnCMPA
#define PWM_0_INTEN_TRCMPAU_B       0x00000400 // Trigger for Counter=PWMnCMPA Up
#define PWM_0_INTEN_TRCNTLOAD_B     0x00000200 // Trigger for Counter=PWMnLOAD
#define PWM_0_INTEN_TRCNTZERO_B     0x00000100 // Trigger for Counter=0
#define PWM_0_INTEN_INTCMPBD_B      0x00000020 // Interrupt for Counter=PWMnCMPB
#define PWM_0_INTEN_INTCMPBU_B      0x00000010 // Interrupt for Counter=PWMnCMPB
#define PWM_0_INTEN_INTCMPAD_B      0x00000008 // Interrupt for Counter=PWMnCMPA
#define PWM_0_INTEN_INTCMPAU_B      0x00000004 // Interrupt for Counter=PWMnCMPA
#define PWM_0_INTEN_INTCNTLOAD_B    0x00000002 // Interrupt for Counter=PWMnLOAD
#define PWM_0_INTEN_INTCNTZERO_B    0x00000001 // Interrupt for Counter=0
#define PWM_0_RIS_INTCMPBD_B        0x00000020 // Comparator B Down Interrupt
#define PWM_0_RIS_INTCMPBU_B        0x00000010 // Comparator B Up Interrupt Status
#define PWM_0_RIS_INTCMPAD_B        0x00000008 // Comparator A Down Interrupt
#define PWM_0_RIS_INTCMPAU_B        0x00000004 // Comparator A Up Interrupt Status
#define PWM_0_RIS_INTCNTLOAD_B      0x00000002 // Counter=Load Interrupt Status
#define PWM_0_RIS_INTCNTZERO_B      0x00000001 // Counter=0 Interrupt Status
#define PWM_0_ISC_INTCMPBD_B        0x00000020 // Comparator B Down Interrupt
#define PWM_0_ISC_INTCMPBU_B        0x00000010 // Comparator B Up Interrupt
#define PWM_0_ISC_INTCMPAD_B        0x00000008 // Comparator A Down Interrupt
#define PWM_0_ISC_INTCMPAU_B        0x00000004 // Comparator A Up Interrupt
#define PWM_0_ISC_INTCNTLOAD_B      0x00000002 // Counter=Load Interrupt
#define PWM_0_ISC_INTCNTZERO_B      0x00000001 // Counter=0 Interrupt
#define PWM_0_LOAD_M                0x0000FFFF // Counter Load Value
#define PWM_0_LOAD_S                0          // Counter Load Value
#define PWM_0_COUNT_M               0x0000FFFF // Counter Value
#define PWM_0_COUNT_S               0          // Counter Value
#define PWM_0_CMPA_M                0x0000FFFF // Comparator A Value
#define PWM_0_CMPA_S                0          // Comparator A Value
#define PWM_0_CMPB_M                0x0000FFFF // Comparator B Value
#define PWM_0_CMPB_S                0          // Comparator B Value
#define PWM_0_GENA_ACTCMPBD_M       0x00000C00 // Action for Comparator B Down
#define PWM_0_GENA_ACTCMPBD_S       10         // Action for Comparator B Down
#define PWM_0_GENA_ACTCMPBD_INV_V   0x00000400 // Invert pwmA
#define PWM_0_GENA_ACTCMPBD_ONE_V   0x00000C00 // Drive pwmA High
#define PWM_0_GENA_ACTCMPBU_M       0x00000300 // Action for Comparator B Up
#define PWM_0_GENA_ACTCMPBU_S       8          // Action for Comparator B Up
#define PWM_0_GENA_ACTCMPBU_INV_V   0x00000100 // Invert pwmA
#define PWM_0_GENA_ACTCMPBU_ONE_V   0x00000300 // Drive pwmA High
#define PWM_0_GENA_ACTCMPAD_M       0x000000C0 // Action for Comparator A Down
#define PWM_0_GENA_ACTCMPAD_S       6          // Action for Comparator A Down
#define PWM_0_GENA_ACTCMPAD_INV_V   0x00000040 // Invert pwmA
#define PWM_0_GENA_ACTCMPAD_ONE_V   0x000000C0 // Drive pwmA High
#define PWM_0_GENA_ACTCMPAU_M       0x00000030 // Action for Comparator A Up
#define PWM_0_GENA_ACTCMPAU_S       4          // Action for Comparator A Up
#define PWM_0_GENA_ACTCMPAU_INV_V   0x00000010 // Invert pwmA
#define PWM_0_GENA_ACTCMPAU_ONE_V   0x00000030 // Drive pwmA High
#define PWM_0_GENA_ACTLOAD_M        0x0000000C // Action for Counter=LOAD
#define PWM_0_GENA_ACTLOAD_S        2          // Action for Counter=LOAD
#define PWM_0_GENA_ACTLOAD_NONE_V   0x00000000 // Do nothing
#define PWM_0_GENA_ACTLOAD_INV_V    0x00000004 // Invert pwmA
#define PWM_0_GENA_ACTLOAD_ZERO_V   0x00000008 // Drive pwmA Low
#define PWM_0_GENA_ACTLOAD_ONE_V    0x0000000C // Drive pwmA High
#define PWM_0_GENA_ACTZERO_M        0x00000003 // Action for Counter=0
#define PWM_0_GENA_ACTZERO_S        0          // Action for Counter=0
#define PWM_0_GENA_ACTZERO_NONE_V   0x00000000 // Do nothing
#define PWM_0_GENA_ACTZERO_INV_V    0x00000001 // Invert pwmA
#define PWM_0_GENA_ACTZERO_ZERO_V   0x00000002 // Drive pwmA Low
#define PWM_0_GENA_ACTZERO_ONE_V    0x00000003 // Drive pwmA High
#define PWM_0_GENB_ACTCMPBD_M       0x00000C00 // Action for Comparator B Down
#define PWM_0_GENB_ACTCMPBD_S       10         // Action for Comparator B Down
#define PWM_0_GENB_ACTCMPBD_INV_V   0x00000400 // Invert pwmB
#define PWM_0_GENB_ACTCMPBD_ONE_V   0x00000C00 // Drive pwmB High
#define PWM_0_GENB_ACTCMPBU_M       0x00000300 // Action for Comparator B Up
#define PWM_0_GENB_ACTCMPBU_S       8          // Action for Comparator B Up
#define PWM_0_GENB_ACTCMPBU_INV_V   0x00000100 // Invert pwmB
#define PWM_0_GENB_ACTCMPBU_ONE_V   0x00000300 // Drive pwmB High
#define PWM_0_GENB_ACTCMPAD_M       0x000000C0 // Action for Comparator A Down
#define PWM_0_GENB_ACTCMPAD_S       6          // Action for Comparator A Down
#define PWM_0_GENB_ACTCMPAD_INV_V   0x00000040 // Invert pwmB
#define PWM_0_GENB_ACTCMPAD_ONE_V   0x000000C0 // Drive pwmB High
#define PWM_0_GENB_ACTCMPAU_M       0x00000030 // Action for Comparator A Up
#define PWM_0_GENB_ACTCMPAU_S       4          // Action for Comparator A Up
#define PWM_0_GENB_ACTCMPAU_INV_V   0x00000010 // Invert pwmB
#define PWM_0_GENB_ACTCMPAU_ONE_V   0x00000030 // Drive pwmB High
#define PWM_0_GENB_ACTLOAD_M        0x0000000C // Action for Counter=LOAD
#define PWM_0_GENB_ACTLOAD_S        2          // Action for Counter=LOAD
#define PWM_0_GENB_ACTLOAD_NONE_V   0x00000000 // Do nothing
#define PWM_0_GENB_ACTLOAD_INV_V    0x00000004 // Invert pwmB
#define PWM_0_GENB_ACTLOAD_ZERO_V   0x00000008 // Drive pwmB Low
#define PWM_0_GENB_ACTLOAD_ONE_V    0x0000000C // Drive pwmB High
#define PWM_0_GENB_ACTZERO_M        0x00000003 // Action for Counter=0
#define PWM_0_GENB_ACTZERO_S        0          // Action for Counter=0
#define PWM_0_GENB_ACTZERO_NONE_V   0x00000000 // Do nothing
#define PWM_0_GENB_ACTZERO_INV_V    0x00000001 // Invert pwmB
#define PWM_0_GENB_ACTZERO_ZERO_V   0x00000002 // Drive pwmB Low
#define PWM_0_GENB_ACTZERO_ONE_V    0x00000003 // Drive pwmB High
#define PWM_0_DBCTL_ENABLE_V        0x00000001 // Dead-Band Generator Enable
#define PWM_0_DBRISE_DELAY_M        0x00000FFF // Dead-Band Rise Delay
#define PWM_0_DBRISE_DELAY_S        0          // Dead-Band Rise Delay
#define PWM_0_DBFALL_DELAY_M        0x00000FFF // Dead-Band Fall Delay
#define PWM_0_DBFALL_DELAY_S        0          // Dead-Band Fall Delay
#define PWM_0_FLTSRC0_FAULT1_V      0x00000002 // Fault1 Input
#define PWM_0_FLTSRC0_FAULT0_V      0x00000001 // Fault0 Input
#define PWM_0_FLTSRC1_DCMP7_V       0x00000080 // Digital Comparator 7
#define PWM_0_FLTSRC1_DCMP6_V       0x00000040 // Digital Comparator 6
#define PWM_0_FLTSRC1_DCMP5_V       0x00000020 // Digital Comparator 5
#define PWM_0_FLTSRC1_DCMP4_V       0x00000010 // Digital Comparator 4
#define PWM_0_FLTSRC1_DCMP3_V       0x00000008 // Digital Comparator 3
#define PWM_0_FLTSRC1_DCMP2_V       0x00000004 // Digital Comparator 2
#define PWM_0_FLTSRC1_DCMP1_V       0x00000002 // Digital Comparator 1
#define PWM_0_FLTSRC1_DCMP0_V       0x00000001 // Digital Comparator 0
#define PWM_0_MINFLTPER_M           0x0000FFFF // Minimum Fault Period
#define PWM_0_MINFLTPER_S           0          // Minimum Fault Period

/* Bit/Fields in Register 1 of Module PWM                                     */
#define PWM_1_CTL_LATCH_B           0x00040000 // Latch Fault Input
#define PWM_1_CTL_MINFLTPER_B       0x00020000 // Minimum Fault Period
#define PWM_1_CTL_FLTSRC_B          0x00010000 // Fault Condition Source
#define PWM_1_CTL_DBFALLUPD_M       0x0000C000 // PWMnDBFALL Update Mode
#define PWM_1_CTL_DBFALLUPD_S       14         // PWMnDBFALL Update Mode
#define PWM_1_CTL_DBFALLUPD_I_V     0x00000000 // Immediate
#define PWM_1_CTL_DBFALLUPD_LS_V    0x00008000 // Locally Synchronized
#define PWM_1_CTL_DBFALLUPD_GS_V    0x0000C000 // Globally Synchronized
#define PWM_1_CTL_DBRISEUPD_M       0x00003000 // PWMnDBRISE Update Mode
#define PWM_1_CTL_DBRISEUPD_S       12         // PWMnDBRISE Update Mode
#define PWM_1_CTL_DBRISEUPD_I_V     0x00000000 // Immediate
#define PWM_1_CTL_DBRISEUPD_LS_V    0x00002000 // Locally Synchronized
#define PWM_1_CTL_DBRISEUPD_GS_V    0x00003000 // Globally Synchronized
#define PWM_1_CTL_DBCTLUPD_M        0x00000C00 // PWMnDBCTL Update Mode
#define PWM_1_CTL_DBCTLUPD_S        10         // PWMnDBCTL Update Mode
#define PWM_1_CTL_DBCTLUPD_I_V      0x00000000 // Immediate
#define PWM_1_CTL_DBCTLUPD_LS_V     0x00000800 // Locally Synchronized
#define PWM_1_CTL_DBCTLUPD_GS_V     0x00000C00 // Globally Synchronized
#define PWM_1_CTL_GENBUPD_M         0x00000300 // PWMnGENB Update Mode
#define PWM_1_CTL_GENBUPD_S         8          // PWMnGENB Update Mode
#define PWM_1_CTL_GENBUPD_I_V       0x00000000 // Immediate
#define PWM_1_CTL_GENBUPD_LS_V      0x00000200 // Locally Synchronized
#define PWM_1_CTL_GENBUPD_GS_V      0x00000300 // Globally Synchronized
#define PWM_1_CTL_GENAUPD_M         0x000000C0 // PWMnGENA Update Mode
#define PWM_1_CTL_GENAUPD_S         6          // PWMnGENA Update Mode
#define PWM_1_CTL_GENAUPD_I_V       0x00000000 // Immediate
#define PWM_1_CTL_GENAUPD_LS_V      0x00000080 // Locally Synchronized
#define PWM_1_CTL_GENAUPD_GS_V      0x000000C0 // Globally Synchronized
#define PWM_1_CTL_CMPBUPD_B         0x00000020 // Comparator B Update Mode
#define PWM_1_CTL_CMPAUPD_B         0x00000010 // Comparator A Update Mode
#define PWM_1_CTL_LOADUPD_B         0x00000008 // Load Register Update Mode
#define PWM_1_CTL_DEBUG_B           0x00000004 // Debug Mode
#define PWM_1_CTL_MODE_B            0x00000002 // Counter Mode
#define PWM_1_CTL_ENABLE_B          0x00000001 // PWM Block Enable
#define PWM_1_INTEN_TRCMPBD_B       0x00002000 // Trigger for Counter=PWMnCMPB
#define PWM_1_INTEN_TRCMPBU_B       0x00001000 // Trigger for Counter=PWMnCMPB Up
#define PWM_1_INTEN_TRCMPAD_B       0x00000800 // Trigger for Counter=PWMnCMPA
#define PWM_1_INTEN_TRCMPAU_B       0x00000400 // Trigger for Counter=PWMnCMPA Up
#define PWM_1_INTEN_TRCNTLOAD_B     0x00000200 // Trigger for Counter=PWMnLOAD
#define PWM_1_INTEN_TRCNTZERO_B     0x00000100 // Trigger for Counter=0
#define PWM_1_INTEN_INTCMPBD_B      0x00000020 // Interrupt for Counter=PWMnCMPB
#define PWM_1_INTEN_INTCMPBU_B      0x00000010 // Interrupt for Counter=PWMnCMPB
#define PWM_1_INTEN_INTCMPAD_B      0x00000008 // Interrupt for Counter=PWMnCMPA
#define PWM_1_INTEN_INTCMPAU_B      0x00000004 // Interrupt for Counter=PWMnCMPA
#define PWM_1_INTEN_INTCNTLOAD_B    0x00000002 // Interrupt for Counter=PWMnLOAD
#define PWM_1_INTEN_INTCNTZERO_B    0x00000001 // Interrupt for Counter=0
#define PWM_1_RIS_INTCMPBD_B        0x00000020 // Comparator B Down Interrupt
#define PWM_1_RIS_INTCMPBU_B        0x00000010 // Comparator B Up Interrupt Status
#define PWM_1_RIS_INTCMPAD_B        0x00000008 // Comparator A Down Interrupt
#define PWM_1_RIS_INTCMPAU_B        0x00000004 // Comparator A Up Interrupt Status
#define PWM_1_RIS_INTCNTLOAD_B      0x00000002 // Counter=Load Interrupt Status
#define PWM_1_RIS_INTCNTZERO_B      0x00000001 // Counter=0 Interrupt Status
#define PWM_1_ISC_INTCMPBD_B        0x00000020 // Comparator B Down Interrupt
#define PWM_1_ISC_INTCMPBU_B        0x00000010 // Comparator B Up Interrupt
#define PWM_1_ISC_INTCMPAD_B        0x00000008 // Comparator A Down Interrupt
#define PWM_1_ISC_INTCMPAU_B        0x00000004 // Comparator A Up Interrupt
#define PWM_1_ISC_INTCNTLOAD_B      0x00000002 // Counter=Load Interrupt
#define PWM_1_ISC_INTCNTZERO_B      0x00000001 // Counter=0 Interrupt
#define PWM_1_LOAD_LOAD_M           0x0000FFFF // Counter Load Value
#define PWM_1_LOAD_LOAD_S           0          // Counter Load Value
#define PWM_1_COUNT_COUNT_M         0x0000FFFF // Counter Value
#define PWM_1_COUNT_COUNT_S         0          // Counter Value
#define PWM_1_CMPA_COMPA_M          0x0000FFFF // Comparator A Value
#define PWM_1_CMPA_COMPA_S          0          // Comparator A Value
#define PWM_1_CMPB_COMPB_M          0x0000FFFF // Comparator B Value
#define PWM_1_CMPB_COMPB_S          0          // Comparator B Value
#define PWM_1_GENA_ACTCMPBD_M       0x00000C00 // Action for Comparator B Down
#define PWM_1_GENA_ACTCMPBD_S       10         // Action for Comparator B Down
#define PWM_1_GENA_ACTCMPBD_INV_V   0x00000400 // Invert pwmA
#define PWM_1_GENA_ACTCMPBD_ONE_V   0x00000C00 // Drive pwmA High
#define PWM_1_GENA_ACTCMPBU_M       0x00000300 // Action for Comparator B Up
#define PWM_1_GENA_ACTCMPBU_S       8          // Action for Comparator B Up
#define PWM_1_GENA_ACTCMPBU_INV_V   0x00000100 // Invert pwmA
#define PWM_1_GENA_ACTCMPBU_ONE_V   0x00000300 // Drive pwmA High
#define PWM_1_GENA_ACTCMPAD_M       0x000000C0 // Action for Comparator A Down
#define PWM_1_GENA_ACTCMPAD_S       6          // Action for Comparator A Down
#define PWM_1_GENA_ACTCMPAD_INV_V   0x00000040 // Invert pwmA
#define PWM_1_GENA_ACTCMPAD_ONE_V   0x000000C0 // Drive pwmA High
#define PWM_1_GENA_ACTCMPAU_M       0x00000030 // Action for Comparator A Up
#define PWM_1_GENA_ACTCMPAU_S       4          // Action for Comparator A Up
#define PWM_1_GENA_ACTCMPAU_INV_V   0x00000010 // Invert pwmA
#define PWM_1_GENA_ACTCMPAU_ONE_V   0x00000030 // Drive pwmA High
#define PWM_1_GENA_ACTLOAD_M        0x0000000C // Action for Counter=LOAD
#define PWM_1_GENA_ACTLOAD_S        2          // Action for Counter=LOAD
#define PWM_1_GENA_ACTLOAD_NONE_V   0x00000000 // Do nothing
#define PWM_1_GENA_ACTLOAD_INV_V    0x00000004 // Invert pwmA
#define PWM_1_GENA_ACTLOAD_ZERO_V   0x00000008 // Drive pwmA Low
#define PWM_1_GENA_ACTLOAD_ONE_V    0x0000000C // Drive pwmA High
#define PWM_1_GENA_ACTZERO_M        0x00000003 // Action for Counter=0
#define PWM_1_GENA_ACTZERO_S        0          // Action for Counter=0
#define PWM_1_GENA_ACTZERO_NONE_V   0x00000000 // Do nothing
#define PWM_1_GENA_ACTZERO_INV_V    0x00000001 // Invert pwmA
#define PWM_1_GENA_ACTZERO_ZERO_V   0x00000002 // Drive pwmA Low
#define PWM_1_GENA_ACTZERO_ONE_V    0x00000003 // Drive pwmA High
#define PWM_1_GENB_ACTCMPBD_M       0x00000C00 // Action for Comparator B Down
#define PWM_1_GENB_ACTCMPBD_S       10         // Action for Comparator B Down
#define PWM_1_GENB_ACTCMPBD_INV_V   0x00000400 // Invert pwmB
#define PWM_1_GENB_ACTCMPBD_ONE_V   0x00000C00 // Drive pwmB High
#define PWM_1_GENB_ACTCMPBU_M       0x00000300 // Action for Comparator B Up
#define PWM_1_GENB_ACTCMPBU_S       8          // Action for Comparator B Up
#define PWM_1_GENB_ACTCMPBU_INV_V   0x00000100 // Invert pwmB
#define PWM_1_GENB_ACTCMPBU_ONE_V   0x00000300 // Drive pwmB High
#define PWM_1_GENB_ACTCMPAD_M       0x000000C0 // Action for Comparator A Down
#define PWM_1_GENB_ACTCMPAD_S       6          // Action for Comparator A Down
#define PWM_1_GENB_ACTCMPAD_INV_V   0x00000040 // Invert pwmB
#define PWM_1_GENB_ACTCMPAD_ONE_V   0x000000C0 // Drive pwmB High
#define PWM_1_GENB_ACTCMPAU_M       0x00000030 // Action for Comparator A Up
#define PWM_1_GENB_ACTCMPAU_S       4          // Action for Comparator A Up
#define PWM_1_GENB_ACTCMPAU_INV_V   0x00000010 // Invert pwmB
#define PWM_1_GENB_ACTCMPAU_ONE_V   0x00000030 // Drive pwmB High
#define PWM_1_GENB_ACTLOAD_M        0x0000000C // Action for Counter=LOAD
#define PWM_1_GENB_ACTLOAD_S        2          // Action for Counter=LOAD
#define PWM_1_GENB_ACTLOAD_NONE_V   0x00000000 // Do nothing
#define PWM_1_GENB_ACTLOAD_INV_V    0x00000004 // Invert pwmB
#define PWM_1_GENB_ACTLOAD_ZERO_V   0x00000008 // Drive pwmB Low
#define PWM_1_GENB_ACTLOAD_ONE_V    0x0000000C // Drive pwmB High
#define PWM_1_GENB_ACTZERO_M        0x00000003 // Action for Counter=0
#define PWM_1_GENB_ACTZERO_S        0          // Action for Counter=0
#define PWM_1_GENB_ACTZERO_NONE_V   0x00000000 // Do nothing
#define PWM_1_GENB_ACTZERO_INV_V    0x00000001 // Invert pwmB
#define PWM_1_GENB_ACTZERO_ZERO_V   0x00000002 // Drive pwmB Low
#define PWM_1_GENB_ACTZERO_ONE_V    0x00000003 // Drive pwmB High
#define PWM_1_DBCTL_ENABLE_V        0x00000001 // Dead-Band Generator Enable
#define PWM_1_FLTSRC0_FAULT1_V      0x00000002 // Fault1 Input
#define PWM_1_FLTSRC0_FAULT0_V      0x00000001 // Fault0 Input
#define PWM_1_FLTSRC1_DCMP7_B       0x00000080 // Digital Comparator 7
#define PWM_1_FLTSRC1_DCMP6_B       0x00000040 // Digital Comparator 6
#define PWM_1_FLTSRC1_DCMP5_B       0x00000020 // Digital Comparator 5
#define PWM_1_FLTSRC1_DCMP4_B       0x00000010 // Digital Comparator 4
#define PWM_1_FLTSRC1_DCMP3_B       0x00000008 // Digital Comparator 3
#define PWM_1_FLTSRC1_DCMP2_B       0x00000004 // Digital Comparator 2
#define PWM_1_FLTSRC1_DCMP1_B       0x00000002 // Digital Comparator 1
#define PWM_1_FLTSRC1_DCMP0_B       0x00000001 // Digital Comparator 0
#define PWM_1_MINFLTPER_MFP_M       0x0000FFFF // Minimum Fault Period
#define PWM_1_MINFLTPER_MFP_S       0          // Minimum Fault Period

/* Bit/Fields in Register 2 of Module PWM                                     */
#define PWM_2_CTL_LATCH_B           0x00040000 // Latch Fault Input
#define PWM_2_CTL_MINFLTPER_B       0x00020000 // Minimum Fault Period
#define PWM_2_CTL_FLTSRC_B          0x00010000 // Fault Condition Source
#define PWM_2_CTL_DBFALLUPD_M       0x0000C000 // PWMnDBFALL Update Mode
#define PWM_2_CTL_DBFALLUPD_S       14         // PWMnDBFALL Update Mode
#define PWM_2_CTL_DBFALLUPD_I_V     0x00000000 // Immediate
#define PWM_2_CTL_DBFALLUPD_LS_V    0x00008000 // Locally Synchronized
#define PWM_2_CTL_DBFALLUPD_GS_V    0x0000C000 // Globally Synchronized
#define PWM_2_CTL_DBRISEUPD_M       0x00003000 // PWMnDBRISE Update Mode
#define PWM_2_CTL_DBRISEUPD_S       12         // PWMnDBRISE Update Mode
#define PWM_2_CTL_DBRISEUPD_I_V     0x00000000 // Immediate
#define PWM_2_CTL_DBRISEUPD_LS_V    0x00002000 // Locally Synchronized
#define PWM_2_CTL_DBRISEUPD_GS_V    0x00003000 // Globally Synchronized
#define PWM_2_CTL_DBCTLUPD_M        0x00000C00 // PWMnDBCTL Update Mode
#define PWM_2_CTL_DBCTLUPD_S        10         // PWMnDBCTL Update Mode
#define PWM_2_CTL_DBCTLUPD_I_V      0x00000000 // Immediate
#define PWM_2_CTL_DBCTLUPD_LS_V     0x00000800 // Locally Synchronized
#define PWM_2_CTL_DBCTLUPD_GS_V     0x00000C00 // Globally Synchronized
#define PWM_2_CTL_GENBUPD_M         0x00000300 // PWMnGENB Update Mode
#define PWM_2_CTL_GENBUPD_S         8          // PWMnGENB Update Mode
#define PWM_2_CTL_GENBUPD_I_V       0x00000000 // Immediate
#define PWM_2_CTL_GENBUPD_LS_V      0x00000200 // Locally Synchronized
#define PWM_2_CTL_GENBUPD_GS_V      0x00000300 // Globally Synchronized
#define PWM_2_CTL_GENAUPD_M         0x000000C0 // PWMnGENA Update Mode
#define PWM_2_CTL_GENAUPD_S         6          // PWMnGENA Update Mode
#define PWM_2_CTL_GENAUPD_I_V       0x00000000 // Immediate
#define PWM_2_CTL_GENAUPD_LS_V      0x00000080 // Locally Synchronized
#define PWM_2_CTL_GENAUPD_GS_V      0x000000C0 // Globally Synchronized
#define PWM_2_CTL_CMPBUPD_B         0x00000020 // Comparator B Update Mode
#define PWM_2_CTL_CMPAUPD_B         0x00000010 // Comparator A Update Mode
#define PWM_2_CTL_LOADUPD_B         0x00000008 // Load Register Update Mode
#define PWM_2_CTL_DEBUG_B           0x00000004 // Debug Mode
#define PWM_2_CTL_MODE_B            0x00000002 // Counter Mode
#define PWM_2_CTL_ENABLE_B          0x00000001 // PWM Block Enable
#define PWM_2_INTEN_TRCMPBD_B       0x00002000 // Trigger for Counter=PWMnCMPB
#define PWM_2_INTEN_TRCMPBU_B       0x00001000 // Trigger for Counter=PWMnCMPB Up
#define PWM_2_INTEN_TRCMPAD_B       0x00000800 // Trigger for Counter=PWMnCMPA
#define PWM_2_INTEN_TRCMPAU_B       0x00000400 // Trigger for Counter=PWMnCMPA Up
#define PWM_2_INTEN_TRCNTLOAD_B     0x00000200 // Trigger for Counter=PWMnLOAD
#define PWM_2_INTEN_TRCNTZERO_B     0x00000100 // Trigger for Counter=0
#define PWM_2_INTEN_INTCMPBD_B      0x00000020 // Interrupt for Counter=PWMnCMPB
#define PWM_2_INTEN_INTCMPBU_B      0x00000010 // Interrupt for Counter=PWMnCMPB
#define PWM_2_INTEN_INTCMPAD_B      0x00000008 // Interrupt for Counter=PWMnCMPA
#define PWM_2_INTEN_INTCMPAU_B      0x00000004 // Interrupt for Counter=PWMnCMPA
#define PWM_2_INTEN_INTCNTLOAD_B    0x00000002 // Interrupt for Counter=PWMnLOAD
#define PWM_2_INTEN_INTCNTZERO_B    0x00000001 // Interrupt for Counter=0
#define PWM_2_RIS_INTCMPBD_B        0x00000020 // Comparator B Down Interrupt
#define PWM_2_RIS_INTCMPBU_B        0x00000010 // Comparator B Up Interrupt Status
#define PWM_2_RIS_INTCMPAD_B        0x00000008 // Comparator A Down Interrupt
#define PWM_2_RIS_INTCMPAU_B        0x00000004 // Comparator A Up Interrupt Status
#define PWM_2_RIS_INTCNTLOAD_B      0x00000002 // Counter=Load Interrupt Status
#define PWM_2_RIS_INTCNTZERO_B      0x00000001 // Counter=0 Interrupt Status
#define PWM_2_ISC_INTCMPBD_B        0x00000020 // Comparator B Down Interrupt
#define PWM_2_ISC_INTCMPBU_B        0x00000010 // Comparator B Up Interrupt
#define PWM_2_ISC_INTCMPAD_B        0x00000008 // Comparator A Down Interrupt
#define PWM_2_ISC_INTCMPAU_B        0x00000004 // Comparator A Up Interrupt
#define PWM_2_ISC_INTCNTLOAD_B      0x00000002 // Counter=Load Interrupt
#define PWM_2_ISC_INTCNTZERO_B      0x00000001 // Counter=0 Interrupt
#define PWM_2_LOAD_LOAD_M           0x0000FFFF // Counter Load Value
#define PWM_2_LOAD_LOAD_S           0          // Counter Load Value
#define PWM_2_COUNT_COUNT_M         0x0000FFFF // Counter Value
#define PWM_2_COUNT_COUNT_S         0          // Counter Value
#define PWM_2_CMPA_COMPA_M          0x0000FFFF // Comparator A Value
#define PWM_2_CMPA_COMPA_S          0          // Comparator A Value
#define PWM_2_CMPB_COMPB_M          0x0000FFFF // Comparator B Value
#define PWM_2_CMPB_COMPB_S          0          // Comparator B Value
#define PWM_2_GENA_ACTCMPBD_M       0x00000C00 // Action for Comparator B Down
#define PWM_2_GENA_ACTCMPBD_S       10         // Action for Comparator B Down
#define PWM_2_GENA_ACTCMPBD_INV_V   0x00000400 // Invert pwmA
#define PWM_2_GENA_ACTCMPBD_ONE_V   0x00000C00 // Drive pwmA High
#define PWM_2_GENA_ACTCMPBU_M       0x00000300 // Action for Comparator B Up
#define PWM_2_GENA_ACTCMPBU_S       8          // Action for Comparator B Up
#define PWM_2_GENA_ACTCMPBU_INV_V   0x00000100 // Invert pwmA
#define PWM_2_GENA_ACTCMPBU_ONE_V   0x00000300 // Drive pwmA High
#define PWM_2_GENA_ACTCMPAD_M       0x000000C0 // Action for Comparator A Down
#define PWM_2_GENA_ACTCMPAD_S       6          // Action for Comparator A Down
#define PWM_2_GENA_ACTCMPAD_INV_V   0x00000040 // Invert pwmA
#define PWM_2_GENA_ACTCMPAD_ONE_V   0x000000C0 // Drive pwmA High
#define PWM_2_GENA_ACTCMPAU_M       0x00000030 // Action for Comparator A Up
#define PWM_2_GENA_ACTCMPAU_S       4          // Action for Comparator A Up
#define PWM_2_GENA_ACTCMPAU_INV_V   0x00000010 // Invert pwmA
#define PWM_2_GENA_ACTCMPAU_ONE_V   0x00000030 // Drive pwmA High
#define PWM_2_GENA_ACTLOAD_M        0x0000000C // Action for Counter=LOAD
#define PWM_2_GENA_ACTLOAD_S        2          // Action for Counter=LOAD
#define PWM_2_GENA_ACTLOAD_NONE_V   0x00000000 // Do nothing
#define PWM_2_GENA_ACTLOAD_INV_V    0x00000004 // Invert pwmA
#define PWM_2_GENA_ACTLOAD_ZERO_V   0x00000008 // Drive pwmA Low
#define PWM_2_GENA_ACTLOAD_ONE_V    0x0000000C // Drive pwmA High
#define PWM_2_GENA_ACTZERO_M        0x00000003 // Action for Counter=0
#define PWM_2_GENA_ACTZERO_S        0          // Action for Counter=0
#define PWM_2_GENA_ACTZERO_NONE_V   0x00000000 // Do nothing
#define PWM_2_GENA_ACTZERO_INV_V    0x00000001 // Invert pwmA
#define PWM_2_GENA_ACTZERO_ZERO_V   0x00000002 // Drive pwmA Low
#define PWM_2_GENA_ACTZERO_ONE_V    0x00000003 // Drive pwmA High
#define PWM_2_GENB_ACTCMPBD_M       0x00000C00 // Action for Comparator B Down
#define PWM_2_GENB_ACTCMPBD_S       10         // Action for Comparator B Down
#define PWM_2_GENB_ACTCMPBD_INV_V   0x00000400 // Invert pwmB
#define PWM_2_GENB_ACTCMPBD_ONE_V   0x00000C00 // Drive pwmB High
#define PWM_2_GENB_ACTCMPBU_M       0x00000300 // Action for Comparator B Up
#define PWM_2_GENB_ACTCMPBU_S       8          // Action for Comparator B Up
#define PWM_2_GENB_ACTCMPBU_INV_V   0x00000100 // Invert pwmB
#define PWM_2_GENB_ACTCMPBU_ONE_V   0x00000300 // Drive pwmB High
#define PWM_2_GENB_ACTCMPAD_M       0x000000C0 // Action for Comparator A Down
#define PWM_2_GENB_ACTCMPAD_S       6          // Action for Comparator A Down
#define PWM_2_GENB_ACTCMPAD_INV_V   0x00000040 // Invert pwmB
#define PWM_2_GENB_ACTCMPAD_ONE_V   0x000000C0 // Drive pwmB High
#define PWM_2_GENB_ACTCMPAU_M       0x00000030 // Action for Comparator A Up
#define PWM_2_GENB_ACTCMPAU_S       4          // Action for Comparator A Up
#define PWM_2_GENB_ACTCMPAU_INV_V   0x00000010 // Invert pwmB
#define PWM_2_GENB_ACTCMPAU_ONE_V   0x00000030 // Drive pwmB High
#define PWM_2_GENB_ACTLOAD_M        0x0000000C // Action for Counter=LOAD
#define PWM_2_GENB_ACTLOAD_S        2          // Action for Counter=LOAD
#define PWM_2_GENB_ACTLOAD_NONE_V   0x00000000 // Do nothing
#define PWM_2_GENB_ACTLOAD_INV_V    0x00000004 // Invert pwmB
#define PWM_2_GENB_ACTLOAD_ZERO_V   0x00000008 // Drive pwmB Low
#define PWM_2_GENB_ACTLOAD_ONE_V    0x0000000C // Drive pwmB High
#define PWM_2_GENB_ACTZERO_M        0x00000003 // Action for Counter=0
#define PWM_2_GENB_ACTZERO_S        0          // Action for Counter=0
#define PWM_2_GENB_ACTZERO_NONE_V   0x00000000 // Do nothing
#define PWM_2_GENB_ACTZERO_INV_V    0x00000001 // Invert pwmB
#define PWM_2_GENB_ACTZERO_ZERO_V   0x00000002 // Drive pwmB Low
#define PWM_2_GENB_ACTZERO_ONE_V    0x00000003 // Drive pwmB High
#define PWM_2_DBCTL_ENABLE_V        0x00000001 // Dead-Band Generator Enable
#define PWM_2_FLTSRC0_FAULT1_V      0x00000002 // Fault1 Input
#define PWM_2_FLTSRC0_FAULT0_V      0x00000001 // Fault0 Input
#define PWM_2_FLTSRC1_DCMP7_B       0x00000080 // Digital Comparator 7
#define PWM_2_FLTSRC1_DCMP6_B       0x00000040 // Digital Comparator 6
#define PWM_2_FLTSRC1_DCMP5_B       0x00000020 // Digital Comparator 5
#define PWM_2_FLTSRC1_DCMP4_B       0x00000010 // Digital Comparator 4
#define PWM_2_FLTSRC1_DCMP3_B       0x00000008 // Digital Comparator 3
#define PWM_2_FLTSRC1_DCMP2_B       0x00000004 // Digital Comparator 2
#define PWM_2_FLTSRC1_DCMP1_B       0x00000002 // Digital Comparator 1
#define PWM_2_FLTSRC1_DCMP0_B       0x00000001 // Digital Comparator 0
#define PWM_2_MINFLTPER_MFP_M       0x0000FFFF // Minimum Fault Period
#define PWM_2_MINFLTPER_MFP_S       0          // Minimum Fault Period

/* Bit/Fields in Register 3 of Module PWM                                     */
#define PWM_3_CTL_LATCH_B           0x00040000 // Latch Fault Input
#define PWM_3_CTL_MINFLTPER_B       0x00020000 // Minimum Fault Period
#define PWM_3_CTL_FLTSRC_B          0x00010000 // Fault Condition Source
#define PWM_3_CTL_DBFALLUPD_M       0x0000C000 // PWMnDBFALL Update Mode
#define PWM_3_CTL_DBFALLUPD_S       14         // PWMnDBFALL Update Mode
#define PWM_3_CTL_DBFALLUPD_I_V     0x00000000 // Immediate
#define PWM_3_CTL_DBFALLUPD_LS_V    0x00008000 // Locally Synchronized
#define PWM_3_CTL_DBFALLUPD_GS_V    0x0000C000 // Globally Synchronized
#define PWM_3_CTL_DBRISEUPD_M       0x00003000 // PWMnDBRISE Update Mode
#define PWM_3_CTL_DBRISEUPD_S       12         // PWMnDBRISE Update Mode
#define PWM_3_CTL_DBRISEUPD_I_V     0x00000000 // Immediate
#define PWM_3_CTL_DBRISEUPD_LS_V    0x00002000 // Locally Synchronized
#define PWM_3_CTL_DBRISEUPD_GS_V    0x00003000 // Globally Synchronized
#define PWM_3_CTL_DBCTLUPD_M        0x00000C00 // PWMnDBCTL Update Mode
#define PWM_3_CTL_DBCTLUPD_S        10         // PWMnDBCTL Update Mode
#define PWM_3_CTL_DBCTLUPD_I_V      0x00000000 // Immediate
#define PWM_3_CTL_DBCTLUPD_LS_V     0x00000800 // Locally Synchronized
#define PWM_3_CTL_DBCTLUPD_GS_V     0x00000C00 // Globally Synchronized
#define PWM_3_CTL_GENBUPD_M         0x00000300 // PWMnGENB Update Mode
#define PWM_3_CTL_GENBUPD_S         8          // PWMnGENB Update Mode
#define PWM_3_CTL_GENBUPD_I_V       0x00000000 // Immediate
#define PWM_3_CTL_GENBUPD_LS_V      0x00000200 // Locally Synchronized
#define PWM_3_CTL_GENBUPD_GS_V      0x00000300 // Globally Synchronized
#define PWM_3_CTL_GENAUPD_M         0x000000C0 // PWMnGENA Update Mode
#define PWM_3_CTL_GENAUPD_S         6          // PWMnGENA Update Mode
#define PWM_3_CTL_GENAUPD_I_V       0x00000000 // Immediate
#define PWM_3_CTL_GENAUPD_LS_V      0x00000080 // Locally Synchronized
#define PWM_3_CTL_GENAUPD_GS_V      0x000000C0 // Globally Synchronized
#define PWM_3_CTL_CMPBUPD_B         0x00000020 // Comparator B Update Mode
#define PWM_3_CTL_CMPAUPD_B         0x00000010 // Comparator A Update Mode
#define PWM_3_CTL_LOADUPD_B         0x00000008 // Load Register Update Mode
#define PWM_3_CTL_DEBUG_B           0x00000004 // Debug Mode
#define PWM_3_CTL_MODE_B            0x00000002 // Counter Mode
#define PWM_3_CTL_ENABLE_B          0x00000001 // PWM Block Enable
#define PWM_3_INTEN_TRCMPBD_B       0x00002000 // Trigger for Counter=PWMnCMPB
#define PWM_3_INTEN_TRCMPBU_B       0x00001000 // Trigger for Counter=PWMnCMPB Up
#define PWM_3_INTEN_TRCMPAD_B       0x00000800 // Trigger for Counter=PWMnCMPA
#define PWM_3_INTEN_TRCMPAU_B       0x00000400 // Trigger for Counter=PWMnCMPA Up
#define PWM_3_INTEN_TRCNTLOAD_B     0x00000200 // Trigger for Counter=PWMnLOAD
#define PWM_3_INTEN_TRCNTZERO_B     0x00000100 // Trigger for Counter=0
#define PWM_3_INTEN_INTCMPBD_B      0x00000020 // Interrupt for Counter=PWMnCMPB
#define PWM_3_INTEN_INTCMPBU_B      0x00000010 // Interrupt for Counter=PWMnCMPB
#define PWM_3_INTEN_INTCMPAD_B      0x00000008 // Interrupt for Counter=PWMnCMPA
#define PWM_3_INTEN_INTCMPAU_B      0x00000004 // Interrupt for Counter=PWMnCMPA
#define PWM_3_INTEN_INTCNTLOAD_B    0x00000002 // Interrupt for Counter=PWMnLOAD
#define PWM_3_INTEN_INTCNTZERO_B    0x00000001 // Interrupt for Counter=0
#define PWM_3_RIS_INTCMPBD_B        0x00000020 // Comparator B Down Interrupt
#define PWM_3_RIS_INTCMPBU_B        0x00000010 // Comparator B Up Interrupt Status
#define PWM_3_RIS_INTCMPAD_B        0x00000008 // Comparator A Down Interrupt
#define PWM_3_RIS_INTCMPAU_B        0x00000004 // Comparator A Up Interrupt Status
#define PWM_3_RIS_INTCNTLOAD_B      0x00000002 // Counter=Load Interrupt Status
#define PWM_3_RIS_INTCNTZERO_B      0x00000001 // Counter=0 Interrupt Status
#define PWM_3_ISC_INTCMPBD_B        0x00000020 // Comparator B Down Interrupt
#define PWM_3_ISC_INTCMPBU_B        0x00000010 // Comparator B Up Interrupt
#define PWM_3_ISC_INTCMPAD_B        0x00000008 // Comparator A Down Interrupt
#define PWM_3_ISC_INTCMPAU_B        0x00000004 // Comparator A Up Interrupt
#define PWM_3_ISC_INTCNTLOAD_B      0x00000002 // Counter=Load Interrupt
#define PWM_3_ISC_INTCNTZERO_B      0x00000001 // Counter=0 Interrupt
#define PWM_3_LOAD_LOAD_M           0x0000FFFF // Counter Load Value
#define PWM_3_LOAD_LOAD_S           0          // Counter Load Value
#define PWM_3_COUNT_COUNT_M         0x0000FFFF // Counter Value
#define PWM_3_COUNT_COUNT_S         0          // Counter Value
#define PWM_3_CMPA_COMPA_M          0x0000FFFF // Comparator A Value
#define PWM_3_CMPA_COMPA_S          0          // Comparator A Value
#define PWM_3_CMPB_COMPB_M          0x0000FFFF // Comparator B Value
#define PWM_3_CMPB_COMPB_S          0          // Comparator B Value
#define PWM_3_GENA_ACTCMPBD_M       0x00000C00 // Action for Comparator B Down
#define PWM_3_GENA_ACTCMPBD_S       10         // Action for Comparator B Down
#define PWM_3_GENA_ACTCMPBD_INV_V   0x00000400 // Invert pwmA
#define PWM_3_GENA_ACTCMPBD_ONE_V   0x00000C00 // Drive pwmA High
#define PWM_3_GENA_ACTCMPBU_M       0x00000300 // Action for Comparator B Up
#define PWM_3_GENA_ACTCMPBU_S       8          // Action for Comparator B Up
#define PWM_3_GENA_ACTCMPBU_INV_V   0x00000100 // Invert pwmA
#define PWM_3_GENA_ACTCMPBU_ONE_V   0x00000300 // Drive pwmA High
#define PWM_3_GENA_ACTCMPAD_M       0x000000C0 // Action for Comparator A Down
#define PWM_3_GENA_ACTCMPAD_S       6          // Action for Comparator A Down
#define PWM_3_GENA_ACTCMPAD_INV_V   0x00000040 // Invert pwmA
#define PWM_3_GENA_ACTCMPAD_ONE_V   0x000000C0 // Drive pwmA High
#define PWM_3_GENA_ACTCMPAU_M       0x00000030 // Action for Comparator A Up
#define PWM_3_GENA_ACTCMPAU_S       4          // Action for Comparator A Up
#define PWM_3_GENA_ACTCMPAU_INV_V   0x00000010 // Invert pwmA
#define PWM_3_GENA_ACTCMPAU_ONE_V   0x00000030 // Drive pwmA High
#define PWM_3_GENA_ACTLOAD_M        0x0000000C // Action for Counter=LOAD
#define PWM_3_GENA_ACTLOAD_S        2          // Action for Counter=LOAD
#define PWM_3_GENA_ACTLOAD_NONE_V   0x00000000 // Do nothing
#define PWM_3_GENA_ACTLOAD_INV_V    0x00000004 // Invert pwmA
#define PWM_3_GENA_ACTLOAD_ZERO_V   0x00000008 // Drive pwmA Low
#define PWM_3_GENA_ACTLOAD_ONE_V    0x0000000C // Drive pwmA High
#define PWM_3_GENA_ACTZERO_M        0x00000003 // Action for Counter=0
#define PWM_3_GENA_ACTZERO_S        0          // Action for Counter=0
#define PWM_3_GENA_ACTZERO_NONE_V   0x00000000 // Do nothing
#define PWM_3_GENA_ACTZERO_INV_V    0x00000001 // Invert pwmA
#define PWM_3_GENA_ACTZERO_ZERO_V   0x00000002 // Drive pwmA Low
#define PWM_3_GENA_ACTZERO_ONE_V    0x00000003 // Drive pwmA High
#define PWM_3_GENB_ACTCMPBD_M       0x00000C00 // Action for Comparator B Down
#define PWM_3_GENB_ACTCMPBD_S       10         // Action for Comparator B Down
#define PWM_3_GENB_ACTCMPBD_INV_V   0x00000400 // Invert pwmB
#define PWM_3_GENB_ACTCMPBD_ONE_V   0x00000C00 // Drive pwmB High
#define PWM_3_GENB_ACTCMPBU_M       0x00000300 // Action for Comparator B Up
#define PWM_3_GENB_ACTCMPBU_S       8          // Action for Comparator B Up
#define PWM_3_GENB_ACTCMPBU_INV_V   0x00000100 // Invert pwmB
#define PWM_3_GENB_ACTCMPBU_ONE_V   0x00000300 // Drive pwmB High
#define PWM_3_GENB_ACTCMPAD_M       0x000000C0 // Action for Comparator A Down
#define PWM_3_GENB_ACTCMPAD_S       6          // Action for Comparator A Down
#define PWM_3_GENB_ACTCMPAD_INV_V   0x00000040 // Invert pwmB
#define PWM_3_GENB_ACTCMPAD_ONE_V   0x000000C0 // Drive pwmB High
#define PWM_3_GENB_ACTCMPAU_M       0x00000030 // Action for Comparator A Up
#define PWM_3_GENB_ACTCMPAU_S       4          // Action for Comparator A Up
#define PWM_3_GENB_ACTCMPAU_INV_V   0x00000010 // Invert pwmB
#define PWM_3_GENB_ACTCMPAU_ONE_V   0x00000030 // Drive pwmB High
#define PWM_3_GENB_ACTLOAD_M        0x0000000C // Action for Counter=LOAD
#define PWM_3_GENB_ACTLOAD_S        2          // Action for Counter=LOAD
#define PWM_3_GENB_ACTLOAD_NONE_V   0x00000000 // Do nothing
#define PWM_3_GENB_ACTLOAD_INV_V    0x00000004 // Invert pwmB
#define PWM_3_GENB_ACTLOAD_ZERO_V   0x00000008 // Drive pwmB Low
#define PWM_3_GENB_ACTLOAD_ONE_V    0x0000000C // Drive pwmB High
#define PWM_3_GENB_ACTZERO_M        0x00000003 // Action for Counter=0
#define PWM_3_GENB_ACTZERO_S        0          // Action for Counter=0
#define PWM_3_GENB_ACTZERO_NONE_V   0x00000000 // Do nothing
#define PWM_3_GENB_ACTZERO_INV_V    0x00000001 // Invert pwmB
#define PWM_3_GENB_ACTZERO_ZERO_V   0x00000002 // Drive pwmB Low
#define PWM_3_GENB_ACTZERO_ONE_V    0x00000003 // Drive pwmB High
#define PWM_3_DBCTL_ENABLE_V        0x00000001 // Dead-Band Generator Enable
#define PWM_3_FLTSRC0_FAULT1_V      0x00000002 // Fault1 Input
#define PWM_3_FLTSRC0_FAULT0_V      0x00000001 // Fault0 Input
#define PWM_3_FLTSRC1_DCMP7_B       0x00000080 // Digital Comparator 7
#define PWM_3_FLTSRC1_DCMP6_B       0x00000040 // Digital Comparator 6
#define PWM_3_FLTSRC1_DCMP5_B       0x00000020 // Digital Comparator 5
#define PWM_3_FLTSRC1_DCMP4_B       0x00000010 // Digital Comparator 4
#define PWM_3_FLTSRC1_DCMP3_B       0x00000008 // Digital Comparator 3
#define PWM_3_FLTSRC1_DCMP2_B       0x00000004 // Digital Comparator 2
#define PWM_3_FLTSRC1_DCMP1_B       0x00000002 // Digital Comparator 1
#define PWM_3_FLTSRC1_DCMP0_B       0x00000001 // Digital Comparator 0
#define PWM_3_MINFLTPER_MFP_M       0x0000FFFF // Minimum Fault Period
#define PWM_3_MINFLTPER_MFP_S       0          // Minimum Fault Period

/* Bit/Fields in Register 0 of Module PWM                                     */
#define PWM_0_FLTSEN_FAULT1_B       0x00000002 // Fault1 Sense
#define PWM_0_FLTSEN_FAULT0_B       0x00000001 // Fault0 Sense
#define PWM_0_FLTSTAT0_FAULT1_B     0x00000002 // Fault Input 1
#define PWM_0_FLTSTAT0_FAULT0_B     0x00000001 // Fault Input 0
#define PWM_0_FLTSTAT1_DCMP7_B      0x00000080 // Digital Comparator 7 Trigger
#define PWM_0_FLTSTAT1_DCMP6_B      0x00000040 // Digital Comparator 6 Trigger
#define PWM_0_FLTSTAT1_DCMP5_B      0x00000020 // Digital Comparator 5 Trigger
#define PWM_0_FLTSTAT1_DCMP4_B      0x00000010 // Digital Comparator 4 Trigger
#define PWM_0_FLTSTAT1_DCMP3_B      0x00000008 // Digital Comparator 3 Trigger
#define PWM_0_FLTSTAT1_DCMP2_B      0x00000004 // Digital Comparator 2 Trigger
#define PWM_0_FLTSTAT1_DCMP1_B      0x00000002 // Digital Comparator 1 Trigger
#define PWM_0_FLTSTAT1_DCMP0_B      0x00000001 // Digital Comparator 0 Trigger

/* Bit/Fields in Register 1 of Module PWM                                     */
#define PWM_1_FLTSEN_FAULT1_B       0x00000002 // Fault1 Sense
#define PWM_1_FLTSEN_FAULT0_B       0x00000001 // Fault0 Sense
#define PWM_1_FLTSTAT0_FAULT1_B     0x00000002 // Fault Input 1
#define PWM_1_FLTSTAT0_FAULT0_B     0x00000001 // Fault Input 0
#define PWM_1_FLTSTAT1_DCMP7_B      0x00000080 // Digital Comparator 7 Trigger
#define PWM_1_FLTSTAT1_DCMP6_B      0x00000040 // Digital Comparator 6 Trigger
#define PWM_1_FLTSTAT1_DCMP5_B      0x00000020 // Digital Comparator 5 Trigger
#define PWM_1_FLTSTAT1_DCMP4_B      0x00000010 // Digital Comparator 4 Trigger
#define PWM_1_FLTSTAT1_DCMP3_B      0x00000008 // Digital Comparator 3 Trigger
#define PWM_1_FLTSTAT1_DCMP2_B      0x00000004 // Digital Comparator 2 Trigger
#define PWM_1_FLTSTAT1_DCMP1_B      0x00000002 // Digital Comparator 1 Trigger
#define PWM_1_FLTSTAT1_DCMP0_B      0x00000001 // Digital Comparator 0 Trigger

/* Bit/Fields in Register 2 of Module PWM                                     */
#define PWM_2_FLTSTAT0_FAULT1_B     0x00000002 // Fault Input 1
#define PWM_2_FLTSTAT0_FAULT0_B     0x00000001 // Fault Input 0
#define PWM_2_FLTSTAT1_DCMP7_B      0x00000080 // Digital Comparator 7 Trigger
#define PWM_2_FLTSTAT1_DCMP6_B      0x00000040 // Digital Comparator 6 Trigger
#define PWM_2_FLTSTAT1_DCMP5_B      0x00000020 // Digital Comparator 5 Trigger
#define PWM_2_FLTSTAT1_DCMP4_B      0x00000010 // Digital Comparator 4 Trigger
#define PWM_2_FLTSTAT1_DCMP3_B      0x00000008 // Digital Comparator 3 Trigger
#define PWM_2_FLTSTAT1_DCMP2_B      0x00000004 // Digital Comparator 2 Trigger
#define PWM_2_FLTSTAT1_DCMP1_B      0x00000002 // Digital Comparator 1 Trigger
#define PWM_2_FLTSTAT1_DCMP0_B      0x00000001 // Digital Comparator 0 Trigger

/* Bit/Fields in Register 3 of Module PWM                                     */
#define PWM_3_FLTSTAT0_FAULT1_B     0x00000002 // Fault Input 1
#define PWM_3_FLTSTAT0_FAULT0_B     0x00000001 // Fault Input 0
#define PWM_3_FLTSTAT1_DCMP7_B      0x00000080 // Digital Comparator 7 Trigger
#define PWM_3_FLTSTAT1_DCMP6_B      0x00000040 // Digital Comparator 6 Trigger
#define PWM_3_FLTSTAT1_DCMP5_B      0x00000020 // Digital Comparator 5 Trigger
#define PWM_3_FLTSTAT1_DCMP4_B      0x00000010 // Digital Comparator 4 Trigger
#define PWM_3_FLTSTAT1_DCMP3_B      0x00000008 // Digital Comparator 3 Trigger
#define PWM_3_FLTSTAT1_DCMP2_B      0x00000004 // Digital Comparator 2 Trigger
#define PWM_3_FLTSTAT1_DCMP1_B      0x00000002 // Digital Comparator 1 Trigger
#define PWM_3_FLTSTAT1_DCMP0_B      0x00000001 // Digital Comparator 0 Trigger

/* Bit/Fields in Register PP of Module PWM                                    */
#define PWM_PP_ONE_B                0x00000400 // One-Shot Mode
#define PWM_PP_EFAULT_B             0x00000200 // Extended Fault
#define PWM_PP_ESYNC_B              0x00000100 // Extended Synchronization
#define PWM_PP_FCNT_M               0x000000F0 // Fault Inputs (per PWM unit)
#define PWM_PP_FCNT_S               4          // Fault Inputs (per PWM unit)
#define PWM_PP_GCNT_M               0x0000000F // Generators
#define PWM_PP_GCNT_S               0          // Generators


/******************************************************************************/
/*                                                                            */
/*                      QEI                                                   */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register CTL of Module QEI                                   */
#define QEI_CTL_FILTCNT_M           0x000F0000 // Input Filter Prescale Count
#define QEI_CTL_FILTCNT_S           16         // Input Filter Prescale Count
#define QEI_CTL_FILTEN_B            0x00002000 // Enable Input Filter
#define QEI_CTL_STALLEN_B           0x00001000 // Stall QEI
#define QEI_CTL_INVI_B              0x00000800 // Invert Index Pulse
#define QEI_CTL_INVB_B              0x00000400 // Invert PhB
#define QEI_CTL_INVA_B              0x00000200 // Invert PhA
#define QEI_CTL_VELDIV_M            0x000001C0 // Predivide Velocity
#define QEI_CTL_VELDIV_S            6          // Predivide Velocity
#define QEI_CTL_VELDIV_1_V          0x00000000 // QEI clock /1
#define QEI_CTL_VELDIV_2_V          0x00000040 // QEI clock /2
#define QEI_CTL_VELDIV_4_V          0x00000080 // QEI clock /4
#define QEI_CTL_VELDIV_8_V          0x000000C0 // QEI clock /8
#define QEI_CTL_VELDIV_16_V         0x00000100 // QEI clock /16
#define QEI_CTL_VELDIV_32_V         0x00000140 // QEI clock /32
#define QEI_CTL_VELDIV_64_V         0x00000180 // QEI clock /64
#define QEI_CTL_VELDIV_128_V        0x000001C0 // QEI clock /128
#define QEI_CTL_VELEN_B             0x00000020 // Capture Velocity
#define QEI_CTL_RESMODE_B           0x00000010 // Reset Mode
#define QEI_CTL_CAPMODE_B           0x00000008 // Capture Mode
#define QEI_CTL_SIGMODE_B           0x00000004 // Signal Mode
#define QEI_CTL_SWAP_B              0x00000002 // Swap Signals
#define QEI_CTL_ENABLE_B            0x00000001 // Enable QEI

/* Bit/Fields in Register STAT of Module QEI                                  */
#define QEI_STAT_DIRECTION_B        0x00000002 // Direction of Rotation
#define QEI_STAT_ERROR_B            0x00000001 // Error Detected

/* Bit/Fields in Register POS of Module QEI                                   */
#define QEI_POS_M                   0xFFFFFFFF // Current Position Integrator
#define QEI_POS_S                   0          // Current Position Integrator

/* Bit/Fields in Register MAXPOS of Module QEI                                */
#define QEI_MAXPOS_M                0xFFFFFFFF // Maximum Position Integrator
#define QEI_MAXPOS_S                0          // Maximum Position Integrator

/* Bit/Fields in Register LOAD of Module QEI                                  */
#define QEI_LOAD_M                  0xFFFFFFFF // Velocity Timer Load Value
#define QEI_LOAD_S                  0          // Velocity Timer Load Value

/* Bit/Fields in Register TIME of Module QEI                                  */
#define QEI_TIME_M                  0xFFFFFFFF // Velocity Timer Current Value
#define QEI_TIME_S                  0          // Velocity Timer Current Value

/* Bit/Fields in Register COUNT of Module QEI                                 */
#define QEI_COUNT_M                 0xFFFFFFFF // Velocity Pulse Count
#define QEI_COUNT_S                 0          // Velocity Pulse Count

/* Bit/Fields in Register SPEED of Module QEI                                 */
#define QEI_SPEED_M                 0xFFFFFFFF // Velocity
#define QEI_SPEED_S                 0          // Velocity

/* Bit/Fields in Register INTEN of Module QEI                                 */
#define QEI_INTEN_ERROR_B           0x00000008 // Phase Error Interrupt Enable
#define QEI_INTEN_DIR_B             0x00000004 // Direction Change Interrupt
#define QEI_INTEN_TIMER_B           0x00000002 // Timer Expires Interrupt Enable
#define QEI_INTEN_INDEX_B           0x00000001 // Index Pulse Detected Interrupt

/* Bit/Fields in Register RIS of Module QEI                                   */
#define QEI_RIS_ERROR_B             0x00000008 // Phase Error Detected
#define QEI_RIS_DIR_B               0x00000004 // Direction Change Detected
#define QEI_RIS_TIMER_B             0x00000002 // Velocity Timer Expired
#define QEI_RIS_INDEX_B             0x00000001 // Index Pulse Asserted

/* Bit/Fields in Register ISC of Module QEI                                   */
#define QEI_ISC_ERROR_B             0x00000008 // Phase Error Interrupt
#define QEI_ISC_DIR_B               0x00000004 // Direction Change Interrupt
#define QEI_ISC_TIMER_B             0x00000002 // Velocity Timer Expired Interrupt
#define QEI_ISC_INDEX_B             0x00000001 // Index Pulse Interrupt


/******************************************************************************/
/*                                                                            */
/*                      TIMER                                                 */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register CFG of Module TIMER                                 */
#define TIMER_CFG_M                 0x00000007 // GPTM Configuration
#define TIMER_CFG_S                 0          // GPTM Configuration
#define TIMER_CFG_32_BIT_TIMER_V    0x00000000 // For a 16/32-bit timer, this
#define TIMER_CFG_32_BIT_RTC_V      0x00000001 // For a 16/32-bit timer, this
#define TIMER_CFG_16_BIT_V          0x00000004 // For a 16/32-bit timer, this

/* Bit/Fields in Register TAMR of Module TIMER                                */
#define TIMER_TAMR_TAPLO_B          0x00000800 // GPTM Timer A PWM Legacy
#define TIMER_TAMR_TAMRSU_B         0x00000400 // GPTM Timer A Match Register
#define TIMER_TAMR_TAPWMIE_B        0x00000200 // GPTM Timer A PWM Interrupt
#define TIMER_TAMR_TAILD_B          0x00000100 // GPTM Timer A Interval Load Write
#define TIMER_TAMR_TASNAPS_B        0x00000080 // GPTM Timer A Snap-Shot Mode
#define TIMER_TAMR_TAWOT_B          0x00000040 // GPTM Timer A Wait-on-Trigger
#define TIMER_TAMR_TAMIE_B          0x00000020 // GPTM Timer A Match Interrupt
#define TIMER_TAMR_TACDIR_B         0x00000010 // GPTM Timer A Count Direction
#define TIMER_TAMR_TAAMS_B          0x00000008 // GPTM Timer A Alternate Mode
#define TIMER_TAMR_TACMR_B          0x00000004 // GPTM Timer A Capture Mode
#define TIMER_TAMR_TAMR_M           0x00000003 // GPTM Timer A Mode
#define TIMER_TAMR_TAMR_S           0          // GPTM Timer A Mode
#define TIMER_TAMR_TAMR_1_SHOT_V    0x00000001 // One-Shot Timer mode
#define TIMER_TAMR_TAMR_PERIOD_V    0x00000002 // Periodic Timer mode
#define TIMER_TAMR_TAMR_CAP_V       0x00000003 // Capture mode

/* Bit/Fields in Register TBMR of Module TIMER                                */
#define TIMER_TBMR_TBPLO_B          0x00000800 // GPTM Timer B PWM Legacy
#define TIMER_TBMR_TBMRSU_B         0x00000400 // GPTM Timer B Match Register
#define TIMER_TBMR_TBPWMIE_B        0x00000200 // GPTM Timer B PWM Interrupt
#define TIMER_TBMR_TBILD_B          0x00000100 // GPTM Timer B Interval Load Write
#define TIMER_TBMR_TBSNAPS_B        0x00000080 // GPTM Timer B Snap-Shot Mode
#define TIMER_TBMR_TBWOT_B          0x00000040 // GPTM Timer B Wait-on-Trigger
#define TIMER_TBMR_TBMIE_B          0x00000020 // GPTM Timer B Match Interrupt
#define TIMER_TBMR_TBCDIR_B         0x00000010 // GPTM Timer B Count Direction
#define TIMER_TBMR_TBAMS_B          0x00000008 // GPTM Timer B Alternate Mode
#define TIMER_TBMR_TBCMR_B          0x00000004 // GPTM Timer B Capture Mode
#define TIMER_TBMR_TBMR_M           0x00000003 // GPTM Timer B Mode
#define TIMER_TBMR_TBMR_S           0          // GPTM Timer B Mode
#define TIMER_TBMR_TBMR_1_SHOT_V    0x00000001 // One-Shot Timer mode
#define TIMER_TBMR_TBMR_PERIOD_V    0x00000002 // Periodic Timer mode
#define TIMER_TBMR_TBMR_CAP_V       0x00000003 // Capture mode

/* Bit/Fields in Register CTL of Module TIMER                                 */
#define TIMER_CTL_TBPWML_B          0x00004000 // GPTM Timer B PWM Output Level
#define TIMER_CTL_TBOTE_B           0x00002000 // GPTM Timer B Output Trigger
#define TIMER_CTL_TBEVENT_M         0x00000C00 // GPTM Timer B Event Mode
#define TIMER_CTL_TBEVENT_S         10         // GPTM Timer B Event Mode
#define TIMER_CTL_TBEVENT_POS_V     0x00000000 // Positive edge
#define TIMER_CTL_TBEVENT_NEG_V     0x00000400 // Negative edge
#define TIMER_CTL_TBEVENT_BOTH_V    0x00000C00 // Both edges
#define TIMER_CTL_TBSTALL_B         0x00000200 // GPTM Timer B Stall Enable
#define TIMER_CTL_TBEN_B            0x00000100 // GPTM Timer B Enable
#define TIMER_CTL_TAPWML_B          0x00000040 // GPTM Timer A PWM Output Level
#define TIMER_CTL_TAOTE_B           0x00000020 // GPTM Timer A Output Trigger
#define TIMER_CTL_RTCEN_B           0x00000010 // GPTM RTC Stall Enable
#define TIMER_CTL_TAEVENT_M         0x0000000C // GPTM Timer A Event Mode
#define TIMER_CTL_TAEVENT_S         2          // GPTM Timer A Event Mode
#define TIMER_CTL_TAEVENT_POS_V     0x00000000 // Positive edge
#define TIMER_CTL_TAEVENT_NEG_V     0x00000004 // Negative edge
#define TIMER_CTL_TAEVENT_BOTH_V    0x0000000C // Both edges
#define TIMER_CTL_TASTALL_B         0x00000002 // GPTM Timer A Stall Enable
#define TIMER_CTL_TAEN_B            0x00000001 // GPTM Timer A Enable

/* Bit/Fields in Register SYNC of Module TIMER                                */
#define TIMER_SYNC_SYNCWT5_M        0x00C00000 // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT5_S        22         // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT5_NONE_V   0x00000000 // GPTM 32/64-Bit Timer 5 is not
#define TIMER_SYNC_SYNCWT5_TA_V     0x00400000 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCWT5_TB_V     0x00800000 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCWT5_TATB_V   0x00C00000 // A timeout event for both Timer A
#define TIMER_SYNC_SYNCWT4_M        0x00300000 // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT4_S        20         // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT4_NONE_V   0x00000000 // GPTM 32/64-Bit Timer 4 is not
#define TIMER_SYNC_SYNCWT4_TA_V     0x00100000 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCWT4_TB_V     0x00200000 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCWT4_TATB_V   0x00300000 // A timeout event for both Timer A
#define TIMER_SYNC_SYNCWT3_M        0x000C0000 // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT3_S        18         // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT3_NONE_V   0x00000000 // GPTM 32/64-Bit Timer 3 is not
#define TIMER_SYNC_SYNCWT3_TA_V     0x00040000 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCWT3_TB_V     0x00080000 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCWT3_TATB_V   0x000C0000 // A timeout event for both Timer A
#define TIMER_SYNC_SYNCWT2_M        0x00030000 // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT2_S        16         // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT2_NONE_V   0x00000000 // GPTM 32/64-Bit Timer 2 is not
#define TIMER_SYNC_SYNCWT2_TA_V     0x00010000 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCWT2_TB_V     0x00020000 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCWT2_TATB_V   0x00030000 // A timeout event for both Timer A
#define TIMER_SYNC_SYNCWT1_M        0x0000C000 // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT1_S        14         // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT1_NONE_V   0x00000000 // GPTM 32/64-Bit Timer 1 is not
#define TIMER_SYNC_SYNCWT1_TA_V     0x00004000 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCWT1_TB_V     0x00008000 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCWT1_TATB_V   0x0000C000 // A timeout event for both Timer A
#define TIMER_SYNC_SYNCWT0_M        0x00003000 // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT0_S        12         // Synchronize GPTM 32/64-Bit Timer
#define TIMER_SYNC_SYNCWT0_NONE_V   0x00000000 // GPTM 32/64-Bit Timer 0 is not
#define TIMER_SYNC_SYNCWT0_TA_V     0x00001000 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCWT0_TB_V     0x00002000 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCWT0_TATB_V   0x00003000 // A timeout event for both Timer A
#define TIMER_SYNC_SYNCT5_M         0x00000C00 // Synchronize GPTM Timer 5
#define TIMER_SYNC_SYNCT5_S         10         // Synchronize GPTM Timer 5
#define TIMER_SYNC_SYNCT5_NONE_V    0x00000000 // GPTM5 is not affected
#define TIMER_SYNC_SYNCT5_TA_V      0x00000400 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCT5_TB_V      0x00000800 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCT5_TATB_V    0x00000C00 // A timeout event for both Timer A
#define TIMER_SYNC_SYNCT4_M         0x00000300 // Synchronize GPTM Timer 4
#define TIMER_SYNC_SYNCT4_S         8          // Synchronize GPTM Timer 4
#define TIMER_SYNC_SYNCT4_NONE_V    0x00000000 // GPTM4 is not affected
#define TIMER_SYNC_SYNCT4_TA_V      0x00000100 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCT4_TB_V      0x00000200 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCT4_TATB_V    0x00000300 // A timeout event for both Timer A
#define TIMER_SYNC_SYNCT3_M         0x000000C0 // Synchronize GPTM Timer 3
#define TIMER_SYNC_SYNCT3_S         6          // Synchronize GPTM Timer 3
#define TIMER_SYNC_SYNCT3_NONE_V    0x00000000 // GPTM3 is not affected
#define TIMER_SYNC_SYNCT3_TA_V      0x00000040 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCT3_TB_V      0x00000080 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCT3_TATB_V    0x000000C0 // A timeout event for both Timer A
#define TIMER_SYNC_SYNCT2_M         0x00000030 // Synchronize GPTM Timer 2
#define TIMER_SYNC_SYNCT2_S         4          // Synchronize GPTM Timer 2
#define TIMER_SYNC_SYNCT2_NONE_V    0x00000000 // GPTM2 is not affected
#define TIMER_SYNC_SYNCT2_TA_V      0x00000010 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCT2_TB_V      0x00000020 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCT2_TATB_V    0x00000030 // A timeout event for both Timer A
#define TIMER_SYNC_SYNCT1_M         0x0000000C // Synchronize GPTM Timer 1
#define TIMER_SYNC_SYNCT1_S         2          // Synchronize GPTM Timer 1
#define TIMER_SYNC_SYNCT1_NONE_V    0x00000000 // GPTM1 is not affected
#define TIMER_SYNC_SYNCT1_TA_V      0x00000004 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCT1_TB_V      0x00000008 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCT1_TATB_V    0x0000000C // A timeout event for both Timer A
#define TIMER_SYNC_SYNCT0_M         0x00000003 // Synchronize GPTM Timer 0
#define TIMER_SYNC_SYNCT0_S         0          // Synchronize GPTM Timer 0
#define TIMER_SYNC_SYNCT0_NONE_V    0x00000000 // GPTM0 is not affected
#define TIMER_SYNC_SYNCT0_TA_V      0x00000001 // A timeout event for Timer A of
#define TIMER_SYNC_SYNCT0_TB_V      0x00000002 // A timeout event for Timer B of
#define TIMER_SYNC_SYNCT0_TATB_V    0x00000003 // A timeout event for both Timer A

/* Bit/Fields in Register IMR of Module TIMER                                 */
#define TIMER_IMR_WUEIM_B           0x00010000 // 32/64-Bit Wide GPTM Write Update
#define TIMER_IMR_TBMIM_B           0x00000800 // GPTM Timer B Match Interrupt
#define TIMER_IMR_CBEIM_B           0x00000400 // GPTM Timer B Capture Mode Event
#define TIMER_IMR_CBMIM_B           0x00000200 // GPTM Timer B Capture Mode Match
#define TIMER_IMR_TBTOIM_B          0x00000100 // GPTM Timer B Time-Out Interrupt
#define TIMER_IMR_TAMIM_B           0x00000010 // GPTM Timer A Match Interrupt
#define TIMER_IMR_RTCIM_B           0x00000008 // GPTM RTC Interrupt Mask
#define TIMER_IMR_CAEIM_B           0x00000004 // GPTM Timer A Capture Mode Event
#define TIMER_IMR_CAMIM_B           0x00000002 // GPTM Timer A Capture Mode Match
#define TIMER_IMR_TATOIM_B          0x00000001 // GPTM Timer A Time-Out Interrupt

/* Bit/Fields in Register RIS of Module TIMER                                 */
#define TIMER_RIS_WUERIS_B          0x00010000 // 32/64-Bit Wide GPTM Write Update
#define TIMER_RIS_TBMRIS_B          0x00000800 // GPTM Timer B Match Raw Interrupt
#define TIMER_RIS_CBERIS_B          0x00000400 // GPTM Timer B Capture Mode Event
#define TIMER_RIS_CBMRIS_B          0x00000200 // GPTM Timer B Capture Mode Match
#define TIMER_RIS_TBTORIS_B         0x00000100 // GPTM Timer B Time-Out Raw
#define TIMER_RIS_TAMRIS_B          0x00000010 // GPTM Timer A Match Raw Interrupt
#define TIMER_RIS_RTCRIS_B          0x00000008 // GPTM RTC Raw Interrupt
#define TIMER_RIS_CAERIS_B          0x00000004 // GPTM Timer A Capture Mode Event
#define TIMER_RIS_CAMRIS_B          0x00000002 // GPTM Timer A Capture Mode Match
#define TIMER_RIS_TATORIS_B         0x00000001 // GPTM Timer A Time-Out Raw

/* Bit/Fields in Register MIS of Module TIMER                                 */
#define TIMER_MIS_WUEMIS_B          0x00010000 // 32/64-Bit Wide GPTM Write Update
#define TIMER_MIS_TBMMIS_B          0x00000800 // GPTM Timer B Match Masked
#define TIMER_MIS_CBEMIS_B          0x00000400 // GPTM Timer B Capture Mode Event
#define TIMER_MIS_CBMMIS_B          0x00000200 // GPTM Timer B Capture Mode Match
#define TIMER_MIS_TBTOMIS_B         0x00000100 // GPTM Timer B Time-Out Masked
#define TIMER_MIS_TAMMIS_B          0x00000010 // GPTM Timer A Match Masked
#define TIMER_MIS_RTCMIS_B          0x00000008 // GPTM RTC Masked Interrupt
#define TIMER_MIS_CAEMIS_B          0x00000004 // GPTM Timer A Capture Mode Event
#define TIMER_MIS_CAMMIS_B          0x00000002 // GPTM Timer A Capture Mode Match
#define TIMER_MIS_TATOMIS_B         0x00000001 // GPTM Timer A Time-Out Masked

/* Bit/Fields in Register ICR of Module TIMER                                 */
#define TIMER_ICR_WUECINT_B         0x00010000 // 32/64-Bit Wide GPTM Write Update
#define TIMER_ICR_TBMCINT_B         0x00000800 // GPTM Timer B Match Interrupt
#define TIMER_ICR_CBECINT_B         0x00000400 // GPTM Timer B Capture Mode Event
#define TIMER_ICR_CBMCINT_B         0x00000200 // GPTM Timer B Capture Mode Match
#define TIMER_ICR_TBTOCINT_B        0x00000100 // GPTM Timer B Time-Out Interrupt
#define TIMER_ICR_TAMCINT_B         0x00000010 // GPTM Timer A Match Interrupt
#define TIMER_ICR_RTCCINT_B         0x00000008 // GPTM RTC Interrupt Clear
#define TIMER_ICR_CAECINT_B         0x00000004 // GPTM Timer A Capture Mode Event
#define TIMER_ICR_CAMCINT_B         0x00000002 // GPTM Timer A Capture Mode Match
#define TIMER_ICR_TATOCINT_B        0x00000001 // GPTM Timer A Time-Out Raw

/* Bit/Fields in Register TAILR of Module TIMER                               */
#define TIMER_TAILR_M               0xFFFFFFFF // GPTM Timer A Interval Load
#define TIMER_TAILR_S               0          // GPTM Timer A Interval Load

/* Bit/Fields in Register TBILR of Module TIMER                               */
#define TIMER_TBILR_M               0xFFFFFFFF // GPTM Timer B Interval Load
#define TIMER_TBILR_S               0          // GPTM Timer B Interval Load

/* Bit/Fields in Register TAMATCHR of Module TIMER                            */
#define TIMER_TAMATCHR_TAMR_M       0xFFFFFFFF // GPTM Timer A Match Register
#define TIMER_TAMATCHR_TAMR_S       0          // GPTM Timer A Match Register

/* Bit/Fields in Register TBMATCHR of Module TIMER                            */
#define TIMER_TBMATCHR_TBMR_M       0xFFFFFFFF // GPTM Timer B Match Register
#define TIMER_TBMATCHR_TBMR_S       0          // GPTM Timer B Match Register

/* Bit/Fields in Register TAPR of Module TIMER                                */
#define TIMER_TAPR_TAPSRH_M         0x0000FF00 // GPTM Timer A Prescale High Byte
#define TIMER_TAPR_TAPSRH_S         8          // GPTM Timer A Prescale High Byte
#define TIMER_TAPR_TAPSR_M          0x000000FF // GPTM Timer A Prescale
#define TIMER_TAPR_TAPSR_S          0          // GPTM Timer A Prescale

/* Bit/Fields in Register TBPR of Module TIMER                                */
#define TIMER_TBPR_TBPSRH_M         0x0000FF00 // GPTM Timer B Prescale High Byte
#define TIMER_TBPR_TBPSRH_S         8          // GPTM Timer B Prescale High Byte
#define TIMER_TBPR_TBPSR_M          0x000000FF // GPTM Timer B Prescale
#define TIMER_TBPR_TBPSR_S          0          // GPTM Timer B Prescale

/* Bit/Fields in Register TAPMR of Module TIMER                               */
#define TIMER_TAPMR_TAPSMRH_M       0x0000FF00 // GPTM Timer A Prescale Match High
#define TIMER_TAPMR_TAPSMRH_S       8          // GPTM Timer A Prescale Match High
#define TIMER_TAPMR_TAPSMR_M        0x000000FF // GPTM TimerA Prescale Match
#define TIMER_TAPMR_TAPSMR_S        0          // GPTM TimerA Prescale Match

/* Bit/Fields in Register TBPMR of Module TIMER                               */
#define TIMER_TBPMR_TBPSMRH_M       0x0000FF00 // GPTM Timer B Prescale Match High
#define TIMER_TBPMR_TBPSMRH_S       8          // GPTM Timer B Prescale Match High
#define TIMER_TBPMR_TBPSMR_M        0x000000FF // GPTM TimerB Prescale Match
#define TIMER_TBPMR_TBPSMR_S        0          // GPTM TimerB Prescale Match

/* Bit/Fields in Register TAR of Module TIMER                                 */
#define TIMER_TAR_M                 0xFFFFFFFF // GPTM Timer A Register
#define TIMER_TAR_S                 0          // GPTM Timer A Register

/* Bit/Fields in Register TBR of Module TIMER                                 */
#define TIMER_TBR_M                 0xFFFFFFFF // GPTM Timer B Register
#define TIMER_TBR_S                 0          // GPTM Timer B Register

/* Bit/Fields in Register TAV of Module TIMER                                 */
#define TIMER_TAV_M                 0xFFFFFFFF // GPTM Timer A Value
#define TIMER_TAV_S                 0          // GPTM Timer A Value

/* Bit/Fields in Register TBV of Module TIMER                                 */
#define TIMER_TBV_M                 0xFFFFFFFF // GPTM Timer B Value
#define TIMER_TBV_S                 0          // GPTM Timer B Value

/* Bit/Fields in Register RTCPD of Module TIMER                               */
#define TIMER_RTCPD_RTCPD_M         0x0000FFFF // RTC Predivide Counter Value
#define TIMER_RTCPD_RTCPD_S         0          // RTC Predivide Counter Value

/* Bit/Fields in Register TAPS of Module TIMER                                */
#define TIMER_TAPS_PSS_M            0x0000FFFF // GPTM Timer A Prescaler Snapshot
#define TIMER_TAPS_PSS_S            0          // GPTM Timer A Prescaler Snapshot

/* Bit/Fields in Register TBPS of Module TIMER                                */
#define TIMER_TBPS_PSS_M            0x0000FFFF // GPTM Timer A Prescaler Value
#define TIMER_TBPS_PSS_S            0          // GPTM Timer A Prescaler Value

/* Bit/Fields in Register TAPV of Module TIMER                                */
#define TIMER_TAPV_PSV_M            0x0000FFFF // GPTM Timer A Prescaler Value
#define TIMER_TAPV_PSV_S            0          // GPTM Timer A Prescaler Value

/* Bit/Fields in Register TBPV of Module TIMER                                */
#define TIMER_TBPV_PSV_M            0x0000FFFF // GPTM Timer B Prescaler Value
#define TIMER_TBPV_PSV_S            0          // GPTM Timer B Prescaler Value

/* Bit/Fields in Register PP of Module TIMER                                  */
#define TIMER_PP_SIZE_M             0x0000000F // Count Size
#define TIMER_PP_SIZE_S             0          // Count Size
#define TIMER_PP_SIZE_16_V          0x00000000 // Timer A and Timer B counters are
#define TIMER_PP_SIZE_32_V          0x00000001 // Timer A and Timer B counters are


/******************************************************************************/
/*                                                                            */
/*                      ADC                                                   */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register ACTSS of Module ADC                                 */
#define ADC_ACTSS_BUSY_B            0x00010000 // ADC Busy
#define ADC_ACTSS_ASEN3_B           0x00000008 // ADC SS3 Enable
#define ADC_ACTSS_ASEN2_B           0x00000004 // ADC SS2 Enable
#define ADC_ACTSS_ASEN1_B           0x00000002 // ADC SS1 Enable
#define ADC_ACTSS_ASEN0_B           0x00000001 // ADC SS0 Enable

/* Bit/Fields in Register RIS of Module ADC                                   */
#define ADC_RIS_INRDC_B             0x00010000 // Digital Comparator Raw Interrupt
#define ADC_RIS_INR3_B              0x00000008 // SS3 Raw Interrupt Status
#define ADC_RIS_INR2_B              0x00000004 // SS2 Raw Interrupt Status
#define ADC_RIS_INR1_B              0x00000002 // SS1 Raw Interrupt Status
#define ADC_RIS_INR0_B              0x00000001 // SS0 Raw Interrupt Status

/* Bit/Fields in Register IM of Module ADC                                    */
#define ADC_IM_DCONSS3_B            0x00080000 // Digital Comparator Interrupt on
#define ADC_IM_DCONSS2_B            0x00040000 // Digital Comparator Interrupt on
#define ADC_IM_DCONSS1_B            0x00020000 // Digital Comparator Interrupt on
#define ADC_IM_DCONSS0_B            0x00010000 // Digital Comparator Interrupt on
#define ADC_IM_MASK3_B              0x00000008 // SS3 Interrupt Mask
#define ADC_IM_MASK2_B              0x00000004 // SS2 Interrupt Mask
#define ADC_IM_MASK1_B              0x00000002 // SS1 Interrupt Mask
#define ADC_IM_MASK0_B              0x00000001 // SS0 Interrupt Mask

/* Bit/Fields in Register ISC of Module ADC                                   */
#define ADC_ISC_DCINSS3_B           0x00080000 // Digital Comparator Interrupt
#define ADC_ISC_DCINSS2_B           0x00040000 // Digital Comparator Interrupt
#define ADC_ISC_DCINSS1_B           0x00020000 // Digital Comparator Interrupt
#define ADC_ISC_DCINSS0_B           0x00010000 // Digital Comparator Interrupt
#define ADC_ISC_IN3_B               0x00000008 // SS3 Interrupt Status and Clear
#define ADC_ISC_IN2_B               0x00000004 // SS2 Interrupt Status and Clear
#define ADC_ISC_IN1_B               0x00000002 // SS1 Interrupt Status and Clear
#define ADC_ISC_IN0_B               0x00000001 // SS0 Interrupt Status and Clear

/* Bit/Fields in Register OSTAT of Module ADC                                 */
#define ADC_OSTAT_OV3_B             0x00000008 // SS3 FIFO Overflow
#define ADC_OSTAT_OV2_B             0x00000004 // SS2 FIFO Overflow
#define ADC_OSTAT_OV1_B             0x00000002 // SS1 FIFO Overflow
#define ADC_OSTAT_OV0_B             0x00000001 // SS0 FIFO Overflow

/* Bit/Fields in Register EMUX of Module ADC                                  */
#define ADC_EMUX_EM3_M              0x0000F000 // SS3 Trigger Select
#define ADC_EMUX_EM3_S              12         // SS3 Trigger Select
#define ADC_EMUX_EM3_PROCESSOR_V    0x00000000 // Processor (default)
#define ADC_EMUX_EM3_COMP0_V        0x00001000 // Analog Comparator 0
#define ADC_EMUX_EM3_COMP1_V        0x00002000 // Analog Comparator 1
#define ADC_EMUX_EM3_EXTERNAL_V     0x00004000 // External (GPIO Pins)
#define ADC_EMUX_EM3_TIMER_V        0x00005000 // Timer
#define ADC_EMUX_EM3_PWM0_V         0x00006000 // PWM generator 0
#define ADC_EMUX_EM3_PWM1_V         0x00007000 // PWM generator 1
#define ADC_EMUX_EM3_PWM2_V         0x00008000 // PWM generator 2
#define ADC_EMUX_EM3_PWM3_V         0x00009000 // PWM generator 3
#define ADC_EMUX_EM3_ALWAYS_V       0x0000F000 // Always (continuously sample)
#define ADC_EMUX_EM2_M              0x00000F00 // SS2 Trigger Select
#define ADC_EMUX_EM2_S              8          // SS2 Trigger Select
#define ADC_EMUX_EM2_PROCESSOR_V    0x00000000 // Processor (default)
#define ADC_EMUX_EM2_COMP0_V        0x00000100 // Analog Comparator 0
#define ADC_EMUX_EM2_COMP1_V        0x00000200 // Analog Comparator 1
#define ADC_EMUX_EM2_EXTERNAL_V     0x00000400 // External (GPIO Pins)
#define ADC_EMUX_EM2_TIMER_V        0x00000500 // Timer
#define ADC_EMUX_EM2_PWM0_V         0x00000600 // PWM generator 0
#define ADC_EMUX_EM2_PWM1_V         0x00000700 // PWM generator 1
#define ADC_EMUX_EM2_PWM2_V         0x00000800 // PWM generator 2
#define ADC_EMUX_EM2_PWM3_V         0x00000900 // PWM generator 3
#define ADC_EMUX_EM2_ALWAYS_V       0x00000F00 // Always (continuously sample)
#define ADC_EMUX_EM1_M              0x000000F0 // SS1 Trigger Select
#define ADC_EMUX_EM1_S              4          // SS1 Trigger Select
#define ADC_EMUX_EM1_PROCESSOR_V    0x00000000 // Processor (default)
#define ADC_EMUX_EM1_COMP0_V        0x00000010 // Analog Comparator 0
#define ADC_EMUX_EM1_COMP1_V        0x00000020 // Analog Comparator 1
#define ADC_EMUX_EM1_EXTERNAL_V     0x00000040 // External (GPIO Pins)
#define ADC_EMUX_EM1_TIMER_V        0x00000050 // Timer
#define ADC_EMUX_EM1_PWM0_V         0x00000060 // PWM generator 0
#define ADC_EMUX_EM1_PWM1_V         0x00000070 // PWM generator 1
#define ADC_EMUX_EM1_PWM2_V         0x00000080 // PWM generator 2
#define ADC_EMUX_EM1_PWM3_V         0x00000090 // PWM generator 3
#define ADC_EMUX_EM1_ALWAYS_V       0x000000F0 // Always (continuously sample)
#define ADC_EMUX_EM0_M              0x0000000F // SS0 Trigger Select
#define ADC_EMUX_EM0_S              0          // SS0 Trigger Select
#define ADC_EMUX_EM0_PROCESSOR_V    0x00000000 // Processor (default)
#define ADC_EMUX_EM0_COMP0_V        0x00000001 // Analog Comparator 0
#define ADC_EMUX_EM0_COMP1_V        0x00000002 // Analog Comparator 1
#define ADC_EMUX_EM0_EXTERNAL_V     0x00000004 // External (GPIO Pins)
#define ADC_EMUX_EM0_TIMER_V        0x00000005 // Timer
#define ADC_EMUX_EM0_PWM0_V         0x00000006 // PWM generator 0
#define ADC_EMUX_EM0_PWM1_V         0x00000007 // PWM generator 1
#define ADC_EMUX_EM0_PWM2_V         0x00000008 // PWM generator 2
#define ADC_EMUX_EM0_PWM3_V         0x00000009 // PWM generator 3
#define ADC_EMUX_EM0_ALWAYS_V       0x0000000F // Always (continuously sample)

/* Bit/Fields in Register USTAT of Module ADC                                 */
#define ADC_USTAT_UV3_B             0x00000008 // SS3 FIFO Underflow
#define ADC_USTAT_UV2_B             0x00000004 // SS2 FIFO Underflow
#define ADC_USTAT_UV1_B             0x00000002 // SS1 FIFO Underflow
#define ADC_USTAT_UV0_B             0x00000001 // SS0 FIFO Underflow

/* Bit/Fields in Register TSSEL of Module ADC                                 */
#define ADC_TSSEL_PS3_M             0x30000000 // Generator 3 PWM Module Trigger
#define ADC_TSSEL_PS3_S             28         // Generator 3 PWM Module Trigger
#define ADC_TSSEL_PS3_0_V           0x00000000 // Use Generator 3 (and its
#define ADC_TSSEL_PS3_1_V           0x10000000 // Use Generator 3 (and its
#define ADC_TSSEL_PS2_M             0x00300000 // Generator 2 PWM Module Trigger
#define ADC_TSSEL_PS2_S             20         // Generator 2 PWM Module Trigger
#define ADC_TSSEL_PS2_0_V           0x00000000 // Use Generator 2 (and its
#define ADC_TSSEL_PS2_1_V           0x00100000 // Use Generator 2 (and its
#define ADC_TSSEL_PS1_M             0x00003000 // Generator 1 PWM Module Trigger
#define ADC_TSSEL_PS1_S             12         // Generator 1 PWM Module Trigger
#define ADC_TSSEL_PS1_0_V           0x00000000 // Use Generator 1 (and its
#define ADC_TSSEL_PS1_1_V           0x00001000 // Use Generator 1 (and its
#define ADC_TSSEL_PS0_M             0x00000030 // Generator 0 PWM Module Trigger
#define ADC_TSSEL_PS0_S             4          // Generator 0 PWM Module Trigger
#define ADC_TSSEL_PS0_0_V           0x00000000 // Use Generator 0 (and its
#define ADC_TSSEL_PS0_1_V           0x00000010 // Use Generator 0 (and its

/* Bit/Fields in Register SSPRI of Module ADC                                 */
#define ADC_SSPRI_SS3_M             0x00003000 // SS3 Priority
#define ADC_SSPRI_SS3_S             12         // SS3 Priority
#define ADC_SSPRI_SS2_M             0x00000300 // SS2 Priority
#define ADC_SSPRI_SS2_S             8          // SS2 Priority
#define ADC_SSPRI_SS1_M             0x00000030 // SS1 Priority
#define ADC_SSPRI_SS1_S             4          // SS1 Priority
#define ADC_SSPRI_SS0_M             0x00000003 // SS0 Priority
#define ADC_SSPRI_SS0_S             0          // SS0 Priority

/* Bit/Fields in Register SPC of Module ADC                                   */
#define ADC_SPC_PHASE_M             0x0000000F // Phase Difference
#define ADC_SPC_PHASE_S             0          // Phase Difference
#define ADC_SPC_PHASE_0_V           0x00000000 // ADC sample lags by 0.0
#define ADC_SPC_PHASE_22_5_V        0x00000001 // ADC sample lags by 22.5
#define ADC_SPC_PHASE_45_V          0x00000002 // ADC sample lags by 45.0
#define ADC_SPC_PHASE_67_5_V        0x00000003 // ADC sample lags by 67.5
#define ADC_SPC_PHASE_90_V          0x00000004 // ADC sample lags by 90.0
#define ADC_SPC_PHASE_112_5_V       0x00000005 // ADC sample lags by 112.5
#define ADC_SPC_PHASE_135_V         0x00000006 // ADC sample lags by 135.0
#define ADC_SPC_PHASE_157_5_V       0x00000007 // ADC sample lags by 157.5
#define ADC_SPC_PHASE_180_V         0x00000008 // ADC sample lags by 180.0
#define ADC_SPC_PHASE_202_5_V       0x00000009 // ADC sample lags by 202.5
#define ADC_SPC_PHASE_225_V         0x0000000A // ADC sample lags by 225.0
#define ADC_SPC_PHASE_247_5_V       0x0000000B // ADC sample lags by 247.5
#define ADC_SPC_PHASE_270_V         0x0000000C // ADC sample lags by 270.0
#define ADC_SPC_PHASE_292_5_V       0x0000000D // ADC sample lags by 292.5
#define ADC_SPC_PHASE_315_V         0x0000000E // ADC sample lags by 315.0
#define ADC_SPC_PHASE_337_5_V       0x0000000F // ADC sample lags by 337.5

/* Bit/Fields in Register PSSI of Module ADC                                  */
#define ADC_PSSI_GSYNC_B            0x80000000 // Global Synchronize
#define ADC_PSSI_SYNCWAIT_B         0x08000000 // Synchronize Wait
#define ADC_PSSI_SS3_B              0x00000008 // SS3 Initiate
#define ADC_PSSI_SS2_B              0x00000004 // SS2 Initiate
#define ADC_PSSI_SS1_B              0x00000002 // SS1 Initiate
#define ADC_PSSI_SS0_B              0x00000001 // SS0 Initiate

/* Bit/Fields in Register SAC of Module ADC                                   */
#define ADC_SAC_AVG_M               0x00000007 // Hardware Averaging Control
#define ADC_SAC_AVG_S               0          // Hardware Averaging Control
#define ADC_SAC_AVG_OFF_V           0x00000000 // No hardware oversampling
#define ADC_SAC_AVG_2X_V            0x00000001 // 2x hardware oversampling
#define ADC_SAC_AVG_4X_V            0x00000002 // 4x hardware oversampling
#define ADC_SAC_AVG_8X_V            0x00000003 // 8x hardware oversampling
#define ADC_SAC_AVG_16X_V           0x00000004 // 16x hardware oversampling
#define ADC_SAC_AVG_32X_V           0x00000005 // 32x hardware oversampling
#define ADC_SAC_AVG_64X_V           0x00000006 // 64x hardware oversampling

/* Bit/Fields in Register DCISC of Module ADC                                 */
#define ADC_DCISC_DCINT7_B          0x00000080 // Digital Comparator 7 Interrupt
#define ADC_DCISC_DCINT6_B          0x00000040 // Digital Comparator 6 Interrupt
#define ADC_DCISC_DCINT5_B          0x00000020 // Digital Comparator 5 Interrupt
#define ADC_DCISC_DCINT4_B          0x00000010 // Digital Comparator 4 Interrupt
#define ADC_DCISC_DCINT3_B          0x00000008 // Digital Comparator 3 Interrupt
#define ADC_DCISC_DCINT2_B          0x00000004 // Digital Comparator 2 Interrupt
#define ADC_DCISC_DCINT1_B          0x00000002 // Digital Comparator 1 Interrupt
#define ADC_DCISC_DCINT0_B          0x00000001 // Digital Comparator 0 Interrupt

/* Bit/Fields in Register CTL of Module ADC                                   */
#define ADC_CTL_DITHER_B            0x00000040 // Dither Mode Enable
#define ADC_CTL_VREF_M              0x00000001 // Voltage Reference Select
#define ADC_CTL_VREF_S              0          // Voltage Reference Select
#define ADC_CTL_VREF_INTERNAL_V     0x00000000 // VDDA and GNDA are the voltage

/* Bit/Fields in Register SSMUX0 of Module ADC                                */
#define ADC_SSMUX0_MUX7_M           0xF0000000 // 8th Sample Input Select
#define ADC_SSMUX0_MUX7_S           28         // 8th Sample Input Select
#define ADC_SSMUX0_MUX6_M           0x0F000000 // 7th Sample Input Select
#define ADC_SSMUX0_MUX6_S           24         // 7th Sample Input Select
#define ADC_SSMUX0_MUX5_M           0x00F00000 // 6th Sample Input Select
#define ADC_SSMUX0_MUX5_S           20         // 6th Sample Input Select
#define ADC_SSMUX0_MUX4_M           0x000F0000 // 5th Sample Input Select
#define ADC_SSMUX0_MUX4_S           16         // 5th Sample Input Select
#define ADC_SSMUX0_MUX3_M           0x0000F000 // 4th Sample Input Select
#define ADC_SSMUX0_MUX3_S           12         // 4th Sample Input Select
#define ADC_SSMUX0_MUX2_M           0x00000F00 // 3rd Sample Input Select
#define ADC_SSMUX0_MUX2_S           8          // 3rd Sample Input Select
#define ADC_SSMUX0_MUX1_M           0x000000F0 // 2nd Sample Input Select
#define ADC_SSMUX0_MUX1_S           4          // 2nd Sample Input Select
#define ADC_SSMUX0_MUX0_M           0x0000000F // 1st Sample Input Select
#define ADC_SSMUX0_MUX0_S           0          // 1st Sample Input Select

/* Bit/Fields in Register SSCTL0 of Module ADC                                */
#define ADC_SSCTL0_TS7_B            0x80000000 // 8th Sample Temp Sensor Select
#define ADC_SSCTL0_IE7_B            0x40000000 // 8th Sample Interrupt Enable
#define ADC_SSCTL0_END7_B           0x20000000 // 8th Sample is End of Sequence
#define ADC_SSCTL0_D7_B             0x10000000 // 8th Sample Differential Input
#define ADC_SSCTL0_TS6_B            0x08000000 // 7th Sample Temp Sensor Select
#define ADC_SSCTL0_IE6_B            0x04000000 // 7th Sample Interrupt Enable
#define ADC_SSCTL0_END6_B           0x02000000 // 7th Sample is End of Sequence
#define ADC_SSCTL0_D6_B             0x01000000 // 7th Sample Differential Input
#define ADC_SSCTL0_TS5_B            0x00800000 // 6th Sample Temp Sensor Select
#define ADC_SSCTL0_IE5_B            0x00400000 // 6th Sample Interrupt Enable
#define ADC_SSCTL0_END5_B           0x00200000 // 6th Sample is End of Sequence
#define ADC_SSCTL0_D5_B             0x00100000 // 6th Sample Differential Input
#define ADC_SSCTL0_TS4_B            0x00080000 // 5th Sample Temp Sensor Select
#define ADC_SSCTL0_IE4_B            0x00040000 // 5th Sample Interrupt Enable
#define ADC_SSCTL0_END4_B           0x00020000 // 5th Sample is End of Sequence
#define ADC_SSCTL0_D4_B             0x00010000 // 5th Sample Differential Input
#define ADC_SSCTL0_TS3_B            0x00008000 // 4th Sample Temp Sensor Select
#define ADC_SSCTL0_IE3_B            0x00004000 // 4th Sample Interrupt Enable
#define ADC_SSCTL0_END3_B           0x00002000 // 4th Sample is End of Sequence
#define ADC_SSCTL0_D3_B             0x00001000 // 4th Sample Differential Input
#define ADC_SSCTL0_TS2_B            0x00000800 // 3rd Sample Temp Sensor Select
#define ADC_SSCTL0_IE2_B            0x00000400 // 3rd Sample Interrupt Enable
#define ADC_SSCTL0_END2_B           0x00000200 // 3rd Sample is End of Sequence
#define ADC_SSCTL0_D2_B             0x00000100 // 3rd Sample Differential Input
#define ADC_SSCTL0_TS1_B            0x00000080 // 2nd Sample Temp Sensor Select
#define ADC_SSCTL0_IE1_B            0x00000040 // 2nd Sample Interrupt Enable
#define ADC_SSCTL0_END1_B           0x00000020 // 2nd Sample is End of Sequence
#define ADC_SSCTL0_D1_B             0x00000010 // 2nd Sample Differential Input
#define ADC_SSCTL0_TS0_B            0x00000008 // 1st Sample Temp Sensor Select
#define ADC_SSCTL0_IE0_B            0x00000004 // 1st Sample Interrupt Enable
#define ADC_SSCTL0_END0_B           0x00000002 // 1st Sample is End of Sequence
#define ADC_SSCTL0_D0_B             0x00000001 // 1st Sample Differential Input

/* Bit/Fields in Register SSFIFO0 of Module ADC                               */
#define ADC_SSFIFO0_DATA_M          0x00000FFF // Conversion Result Data
#define ADC_SSFIFO0_DATA_S          0          // Conversion Result Data

/* Bit/Fields in Register SSFSTAT0 of Module ADC                              */
#define ADC_SSFSTAT0_FULL_B         0x00001000 // FIFO Full
#define ADC_SSFSTAT0_EMPTY_B        0x00000100 // FIFO Empty
#define ADC_SSFSTAT0_HPTR_M         0x000000F0 // FIFO Head Pointer
#define ADC_SSFSTAT0_HPTR_S         4          // FIFO Head Pointer
#define ADC_SSFSTAT0_TPTR_M         0x0000000F // FIFO Tail Pointer
#define ADC_SSFSTAT0_TPTR_S         0          // FIFO Tail Pointer

/* Bit/Fields in Register SSOP0 of Module ADC                                 */
#define ADC_SSOP0_S7DCOP_B          0x10000000 // Sample 7 Digital Comparator
#define ADC_SSOP0_S6DCOP_B          0x01000000 // Sample 6 Digital Comparator
#define ADC_SSOP0_S5DCOP_B          0x00100000 // Sample 5 Digital Comparator
#define ADC_SSOP0_S4DCOP_B          0x00010000 // Sample 4 Digital Comparator
#define ADC_SSOP0_S3DCOP_B          0x00001000 // Sample 3 Digital Comparator
#define ADC_SSOP0_S2DCOP_B          0x00000100 // Sample 2 Digital Comparator
#define ADC_SSOP0_S1DCOP_B          0x00000010 // Sample 1 Digital Comparator
#define ADC_SSOP0_S0DCOP_B          0x00000001 // Sample 0 Digital Comparator

/* Bit/Fields in Register SSDC0 of Module ADC                                 */
#define ADC_SSDC0_S7DCSEL_M         0xF0000000 // Sample 7 Digital Comparator
#define ADC_SSDC0_S7DCSEL_S         28         // Sample 7 Digital Comparator
#define ADC_SSDC0_S6DCSEL_M         0x0F000000 // Sample 6 Digital Comparator
#define ADC_SSDC0_S6DCSEL_S         24         // Sample 6 Digital Comparator
#define ADC_SSDC0_S5DCSEL_M         0x00F00000 // Sample 5 Digital Comparator
#define ADC_SSDC0_S5DCSEL_S         20         // Sample 5 Digital Comparator
#define ADC_SSDC0_S4DCSEL_M         0x000F0000 // Sample 4 Digital Comparator
#define ADC_SSDC0_S4DCSEL_S         16         // Sample 4 Digital Comparator
#define ADC_SSDC0_S3DCSEL_M         0x0000F000 // Sample 3 Digital Comparator
#define ADC_SSDC0_S3DCSEL_S         12         // Sample 3 Digital Comparator
#define ADC_SSDC0_S2DCSEL_M         0x00000F00 // Sample 2 Digital Comparator
#define ADC_SSDC0_S2DCSEL_S         8          // Sample 2 Digital Comparator
#define ADC_SSDC0_S1DCSEL_M         0x000000F0 // Sample 1 Digital Comparator
#define ADC_SSDC0_S1DCSEL_S         4          // Sample 1 Digital Comparator
#define ADC_SSDC0_S0DCSEL_M         0x0000000F // Sample 0 Digital Comparator
#define ADC_SSDC0_S0DCSEL_S         0          // Sample 0 Digital Comparator

/* Bit/Fields in Register SSMUX1 of Module ADC                                */
#define ADC_SSMUX1_MUX3_M           0x0000F000 // 4th Sample Input Select
#define ADC_SSMUX1_MUX3_S           12         // 4th Sample Input Select
#define ADC_SSMUX1_MUX2_M           0x00000F00 // 3rd Sample Input Select
#define ADC_SSMUX1_MUX2_S           8          // 3rd Sample Input Select
#define ADC_SSMUX1_MUX1_M           0x000000F0 // 2nd Sample Input Select
#define ADC_SSMUX1_MUX1_S           4          // 2nd Sample Input Select
#define ADC_SSMUX1_MUX0_M           0x0000000F // 1st Sample Input Select
#define ADC_SSMUX1_MUX0_S           0          // 1st Sample Input Select

/* Bit/Fields in Register SSCTL1 of Module ADC                                */
#define ADC_SSCTL1_TS3_B            0x00008000 // 4th Sample Temp Sensor Select
#define ADC_SSCTL1_IE3_B            0x00004000 // 4th Sample Interrupt Enable
#define ADC_SSCTL1_END3_B           0x00002000 // 4th Sample is End of Sequence
#define ADC_SSCTL1_D3_B             0x00001000 // 4th Sample Differential Input
#define ADC_SSCTL1_TS2_B            0x00000800 // 3rd Sample Temp Sensor Select
#define ADC_SSCTL1_IE2_B            0x00000400 // 3rd Sample Interrupt Enable
#define ADC_SSCTL1_END2_B           0x00000200 // 3rd Sample is End of Sequence
#define ADC_SSCTL1_D2_B             0x00000100 // 3rd Sample Differential Input
#define ADC_SSCTL1_TS1_B            0x00000080 // 2nd Sample Temp Sensor Select
#define ADC_SSCTL1_IE1_B            0x00000040 // 2nd Sample Interrupt Enable
#define ADC_SSCTL1_END1_B           0x00000020 // 2nd Sample is End of Sequence
#define ADC_SSCTL1_D1_B             0x00000010 // 2nd Sample Differential Input
#define ADC_SSCTL1_TS0_B            0x00000008 // 1st Sample Temp Sensor Select
#define ADC_SSCTL1_IE0_B            0x00000004 // 1st Sample Interrupt Enable
#define ADC_SSCTL1_END0_B           0x00000002 // 1st Sample is End of Sequence
#define ADC_SSCTL1_D0_B             0x00000001 // 1st Sample Differential Input

/* Bit/Fields in Register SSFIFO1 of Module ADC                               */
#define ADC_SSFIFO1_DATA_M          0x00000FFF // Conversion Result Data
#define ADC_SSFIFO1_DATA_S          0          // Conversion Result Data

/* Bit/Fields in Register SSFSTAT1 of Module ADC                              */
#define ADC_SSFSTAT1_FULL_B         0x00001000 // FIFO Full
#define ADC_SSFSTAT1_EMPTY_B        0x00000100 // FIFO Empty
#define ADC_SSFSTAT1_HPTR_M         0x000000F0 // FIFO Head Pointer
#define ADC_SSFSTAT1_HPTR_S         4          // FIFO Head Pointer
#define ADC_SSFSTAT1_TPTR_M         0x0000000F // FIFO Tail Pointer
#define ADC_SSFSTAT1_TPTR_S         0          // FIFO Tail Pointer

/* Bit/Fields in Register SSOP1 of Module ADC                                 */
#define ADC_SSOP1_S3DCOP_B          0x00001000 // Sample 3 Digital Comparator
#define ADC_SSOP1_S2DCOP_B          0x00000100 // Sample 2 Digital Comparator
#define ADC_SSOP1_S1DCOP_B          0x00000010 // Sample 1 Digital Comparator
#define ADC_SSOP1_S0DCOP_B          0x00000001 // Sample 0 Digital Comparator

/* Bit/Fields in Register SSDC1 of Module ADC                                 */
#define ADC_SSDC1_S3DCSEL_M         0x0000F000 // Sample 3 Digital Comparator
#define ADC_SSDC1_S3DCSEL_S         12         // Sample 3 Digital Comparator
#define ADC_SSDC1_S2DCSEL_M         0x00000F00 // Sample 2 Digital Comparator
#define ADC_SSDC1_S2DCSEL_S         8          // Sample 2 Digital Comparator
#define ADC_SSDC1_S1DCSEL_M         0x000000F0 // Sample 1 Digital Comparator
#define ADC_SSDC1_S1DCSEL_S         4          // Sample 1 Digital Comparator
#define ADC_SSDC1_S0DCSEL_M         0x0000000F // Sample 0 Digital Comparator
#define ADC_SSDC1_S0DCSEL_S         0          // Sample 0 Digital Comparator

/* Bit/Fields in Register SSMUX2 of Module ADC                                */
#define ADC_SSMUX2_MUX3_M           0x0000F000 // 4th Sample Input Select
#define ADC_SSMUX2_MUX3_S           12         // 4th Sample Input Select
#define ADC_SSMUX2_MUX2_M           0x00000F00 // 3rd Sample Input Select
#define ADC_SSMUX2_MUX2_S           8          // 3rd Sample Input Select
#define ADC_SSMUX2_MUX1_M           0x000000F0 // 2nd Sample Input Select
#define ADC_SSMUX2_MUX1_S           4          // 2nd Sample Input Select
#define ADC_SSMUX2_MUX0_M           0x0000000F // 1st Sample Input Select
#define ADC_SSMUX2_MUX0_S           0          // 1st Sample Input Select

/* Bit/Fields in Register SSCTL2 of Module ADC                                */
#define ADC_SSCTL2_TS3_B            0x00008000 // 4th Sample Temp Sensor Select
#define ADC_SSCTL2_IE3_B            0x00004000 // 4th Sample Interrupt Enable
#define ADC_SSCTL2_END3_B           0x00002000 // 4th Sample is End of Sequence
#define ADC_SSCTL2_D3_B             0x00001000 // 4th Sample Differential Input
#define ADC_SSCTL2_TS2_B            0x00000800 // 3rd Sample Temp Sensor Select
#define ADC_SSCTL2_IE2_B            0x00000400 // 3rd Sample Interrupt Enable
#define ADC_SSCTL2_END2_B           0x00000200 // 3rd Sample is End of Sequence
#define ADC_SSCTL2_D2_B             0x00000100 // 3rd Sample Differential Input
#define ADC_SSCTL2_TS1_B            0x00000080 // 2nd Sample Temp Sensor Select
#define ADC_SSCTL2_IE1_B            0x00000040 // 2nd Sample Interrupt Enable
#define ADC_SSCTL2_END1_B           0x00000020 // 2nd Sample is End of Sequence
#define ADC_SSCTL2_D1_B             0x00000010 // 2nd Sample Differential Input
#define ADC_SSCTL2_TS0_B            0x00000008 // 1st Sample Temp Sensor Select
#define ADC_SSCTL2_IE0_B            0x00000004 // 1st Sample Interrupt Enable
#define ADC_SSCTL2_END0_B           0x00000002 // 1st Sample is End of Sequence
#define ADC_SSCTL2_D0_B             0x00000001 // 1st Sample Differential Input

/* Bit/Fields in Register SSFIFO2 of Module ADC                               */
#define ADC_SSFIFO2_DATA_M          0x00000FFF // Conversion Result Data
#define ADC_SSFIFO2_DATA_S          0          // Conversion Result Data

/* Bit/Fields in Register SSFSTAT2 of Module ADC                              */
#define ADC_SSFSTAT2_FULL_B         0x00001000 // FIFO Full
#define ADC_SSFSTAT2_EMPTY_B        0x00000100 // FIFO Empty
#define ADC_SSFSTAT2_HPTR_M         0x000000F0 // FIFO Head Pointer
#define ADC_SSFSTAT2_HPTR_S         4          // FIFO Head Pointer
#define ADC_SSFSTAT2_TPTR_M         0x0000000F // FIFO Tail Pointer
#define ADC_SSFSTAT2_TPTR_S         0          // FIFO Tail Pointer

/* Bit/Fields in Register SSOP2 of Module ADC                                 */
#define ADC_SSOP2_S3DCOP_B          0x00001000 // Sample 3 Digital Comparator
#define ADC_SSOP2_S2DCOP_B          0x00000100 // Sample 2 Digital Comparator
#define ADC_SSOP2_S1DCOP_B          0x00000010 // Sample 1 Digital Comparator
#define ADC_SSOP2_S0DCOP_B          0x00000001 // Sample 0 Digital Comparator

/* Bit/Fields in Register SSDC2 of Module ADC                                 */
#define ADC_SSDC2_S3DCSEL_M         0x0000F000 // Sample 3 Digital Comparator
#define ADC_SSDC2_S3DCSEL_S         12         // Sample 3 Digital Comparator
#define ADC_SSDC2_S2DCSEL_M         0x00000F00 // Sample 2 Digital Comparator
#define ADC_SSDC2_S2DCSEL_S         8          // Sample 2 Digital Comparator
#define ADC_SSDC2_S1DCSEL_M         0x000000F0 // Sample 1 Digital Comparator
#define ADC_SSDC2_S1DCSEL_S         4          // Sample 1 Digital Comparator
#define ADC_SSDC2_S0DCSEL_M         0x0000000F // Sample 0 Digital Comparator
#define ADC_SSDC2_S0DCSEL_S         0          // Sample 0 Digital Comparator

/* Bit/Fields in Register SSMUX3 of Module ADC                                */
#define ADC_SSMUX3_MUX0_M           0x0000000F // 1st Sample Input Select
#define ADC_SSMUX3_MUX0_S           0          // 1st Sample Input Select

/* Bit/Fields in Register SSCTL3 of Module ADC                                */
#define ADC_SSCTL3_TS0_B            0x00000008 // 1st Sample Temp Sensor Select
#define ADC_SSCTL3_IE0_B            0x00000004 // Sample Interrupt Enable
#define ADC_SSCTL3_END0_B           0x00000002 // End of Sequence
#define ADC_SSCTL3_D0_B             0x00000001 // Sample Differential Input Select

/* Bit/Fields in Register SSFIFO3 of Module ADC                               */
#define ADC_SSFIFO3_DATA_M          0x00000FFF // Conversion Result Data
#define ADC_SSFIFO3_DATA_S          0          // Conversion Result Data

/* Bit/Fields in Register SSFSTAT3 of Module ADC                              */
#define ADC_SSFSTAT3_FULL_B         0x00001000 // FIFO Full
#define ADC_SSFSTAT3_EMPTY_B        0x00000100 // FIFO Empty
#define ADC_SSFSTAT3_HPTR_M         0x000000F0 // FIFO Head Pointer
#define ADC_SSFSTAT3_HPTR_S         4          // FIFO Head Pointer
#define ADC_SSFSTAT3_TPTR_M         0x0000000F // FIFO Tail Pointer
#define ADC_SSFSTAT3_TPTR_S         0          // FIFO Tail Pointer

/* Bit/Fields in Register SSOP3 of Module ADC                                 */
#define ADC_SSOP3_S0DCOP_B          0x00000001 // Sample 0 Digital Comparator

/* Bit/Fields in Register SSDC3 of Module ADC                                 */
#define ADC_SSDC3_S0DCSEL_M         0x0000000F // Sample 0 Digital Comparator
#define ADC_SSDC3_S0DCSEL_S         0          // Sample 0 Digital Comparator

/* Bit/Fields in Register DCRIC of Module ADC                                 */
#define ADC_DCRIC_DCTRIG7_B         0x00800000 // Digital Comparator Trigger 7
#define ADC_DCRIC_DCTRIG6_B         0x00400000 // Digital Comparator Trigger 6
#define ADC_DCRIC_DCTRIG5_B         0x00200000 // Digital Comparator Trigger 5
#define ADC_DCRIC_DCTRIG4_B         0x00100000 // Digital Comparator Trigger 4
#define ADC_DCRIC_DCTRIG3_B         0x00080000 // Digital Comparator Trigger 3
#define ADC_DCRIC_DCTRIG2_B         0x00040000 // Digital Comparator Trigger 2
#define ADC_DCRIC_DCTRIG1_B         0x00020000 // Digital Comparator Trigger 1
#define ADC_DCRIC_DCTRIG0_B         0x00010000 // Digital Comparator Trigger 0
#define ADC_DCRIC_DCINT7_B          0x00000080 // Digital Comparator Interrupt 7
#define ADC_DCRIC_DCINT6_B          0x00000040 // Digital Comparator Interrupt 6
#define ADC_DCRIC_DCINT5_B          0x00000020 // Digital Comparator Interrupt 5
#define ADC_DCRIC_DCINT4_B          0x00000010 // Digital Comparator Interrupt 4
#define ADC_DCRIC_DCINT3_B          0x00000008 // Digital Comparator Interrupt 3
#define ADC_DCRIC_DCINT2_B          0x00000004 // Digital Comparator Interrupt 2
#define ADC_DCRIC_DCINT1_B          0x00000002 // Digital Comparator Interrupt 1
#define ADC_DCRIC_DCINT0_B          0x00000001 // Digital Comparator Interrupt 0

/* Bit/Fields in Register DCCTL0 of Module ADC                                */
#define ADC_DCCTL0_CTE_B            0x00001000 // Comparison Trigger Enable
#define ADC_DCCTL0_CTC_M            0x00000C00 // Comparison Trigger Condition
#define ADC_DCCTL0_CTC_S            10         // Comparison Trigger Condition
#define ADC_DCCTL0_CTC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL0_CTC_MID_V        0x00000400 // Mid Band
#define ADC_DCCTL0_CTC_HIGH_V       0x00000C00 // High Band
#define ADC_DCCTL0_CTM_M            0x00000300 // Comparison Trigger Mode
#define ADC_DCCTL0_CTM_S            8          // Comparison Trigger Mode
#define ADC_DCCTL0_CTM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL0_CTM_ONCE_V       0x00000100 // Once
#define ADC_DCCTL0_CTM_HALWAYS_V    0x00000200 // Hysteresis Always
#define ADC_DCCTL0_CTM_HONCE_V      0x00000300 // Hysteresis Once
#define ADC_DCCTL0_CIE_B            0x00000010 // Comparison Interrupt Enable
#define ADC_DCCTL0_CIC_M            0x0000000C // Comparison Interrupt Condition
#define ADC_DCCTL0_CIC_S            2          // Comparison Interrupt Condition
#define ADC_DCCTL0_CIC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL0_CIC_MID_V        0x00000004 // Mid Band
#define ADC_DCCTL0_CIC_HIGH_V       0x0000000C // High Band
#define ADC_DCCTL0_CIM_M            0x00000003 // Comparison Interrupt Mode
#define ADC_DCCTL0_CIM_S            0          // Comparison Interrupt Mode
#define ADC_DCCTL0_CIM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL0_CIM_ONCE_V       0x00000001 // Once
#define ADC_DCCTL0_CIM_HALWAYS_V    0x00000002 // Hysteresis Always
#define ADC_DCCTL0_CIM_HONCE_V      0x00000003 // Hysteresis Once

/* Bit/Fields in Register DCCTL1 of Module ADC                                */
#define ADC_DCCTL1_CTE_B            0x00001000 // Comparison Trigger Enable
#define ADC_DCCTL1_CTC_M            0x00000C00 // Comparison Trigger Condition
#define ADC_DCCTL1_CTC_S            10         // Comparison Trigger Condition
#define ADC_DCCTL1_CTC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL1_CTC_MID_V        0x00000400 // Mid Band
#define ADC_DCCTL1_CTC_HIGH_V       0x00000C00 // High Band
#define ADC_DCCTL1_CTM_M            0x00000300 // Comparison Trigger Mode
#define ADC_DCCTL1_CTM_S            8          // Comparison Trigger Mode
#define ADC_DCCTL1_CTM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL1_CTM_ONCE_V       0x00000100 // Once
#define ADC_DCCTL1_CTM_HALWAYS_V    0x00000200 // Hysteresis Always
#define ADC_DCCTL1_CTM_HONCE_V      0x00000300 // Hysteresis Once
#define ADC_DCCTL1_CIE_B            0x00000010 // Comparison Interrupt Enable
#define ADC_DCCTL1_CIC_M            0x0000000C // Comparison Interrupt Condition
#define ADC_DCCTL1_CIC_S            2          // Comparison Interrupt Condition
#define ADC_DCCTL1_CIC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL1_CIC_MID_V        0x00000004 // Mid Band
#define ADC_DCCTL1_CIC_HIGH_V       0x0000000C // High Band
#define ADC_DCCTL1_CIM_M            0x00000003 // Comparison Interrupt Mode
#define ADC_DCCTL1_CIM_S            0          // Comparison Interrupt Mode
#define ADC_DCCTL1_CIM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL1_CIM_ONCE_V       0x00000001 // Once
#define ADC_DCCTL1_CIM_HALWAYS_V    0x00000002 // Hysteresis Always
#define ADC_DCCTL1_CIM_HONCE_V      0x00000003 // Hysteresis Once

/* Bit/Fields in Register DCCTL2 of Module ADC                                */
#define ADC_DCCTL2_CTE_B            0x00001000 // Comparison Trigger Enable
#define ADC_DCCTL2_CTC_M            0x00000C00 // Comparison Trigger Condition
#define ADC_DCCTL2_CTC_S            10         // Comparison Trigger Condition
#define ADC_DCCTL2_CTC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL2_CTC_MID_V        0x00000400 // Mid Band
#define ADC_DCCTL2_CTC_HIGH_V       0x00000C00 // High Band
#define ADC_DCCTL2_CTM_M            0x00000300 // Comparison Trigger Mode
#define ADC_DCCTL2_CTM_S            8          // Comparison Trigger Mode
#define ADC_DCCTL2_CTM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL2_CTM_ONCE_V       0x00000100 // Once
#define ADC_DCCTL2_CTM_HALWAYS_V    0x00000200 // Hysteresis Always
#define ADC_DCCTL2_CTM_HONCE_V      0x00000300 // Hysteresis Once
#define ADC_DCCTL2_CIE_B            0x00000010 // Comparison Interrupt Enable
#define ADC_DCCTL2_CIC_M            0x0000000C // Comparison Interrupt Condition
#define ADC_DCCTL2_CIC_S            2          // Comparison Interrupt Condition
#define ADC_DCCTL2_CIC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL2_CIC_MID_V        0x00000004 // Mid Band
#define ADC_DCCTL2_CIC_HIGH_V       0x0000000C // High Band
#define ADC_DCCTL2_CIM_M            0x00000003 // Comparison Interrupt Mode
#define ADC_DCCTL2_CIM_S            0          // Comparison Interrupt Mode
#define ADC_DCCTL2_CIM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL2_CIM_ONCE_V       0x00000001 // Once
#define ADC_DCCTL2_CIM_HALWAYS_V    0x00000002 // Hysteresis Always
#define ADC_DCCTL2_CIM_HONCE_V      0x00000003 // Hysteresis Once

/* Bit/Fields in Register DCCTL3 of Module ADC                                */
#define ADC_DCCTL3_CTE_B            0x00001000 // Comparison Trigger Enable
#define ADC_DCCTL3_CTC_M            0x00000C00 // Comparison Trigger Condition
#define ADC_DCCTL3_CTC_S            10         // Comparison Trigger Condition
#define ADC_DCCTL3_CTC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL3_CTC_MID_V        0x00000400 // Mid Band
#define ADC_DCCTL3_CTC_HIGH_V       0x00000C00 // High Band
#define ADC_DCCTL3_CTM_M            0x00000300 // Comparison Trigger Mode
#define ADC_DCCTL3_CTM_S            8          // Comparison Trigger Mode
#define ADC_DCCTL3_CTM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL3_CTM_ONCE_V       0x00000100 // Once
#define ADC_DCCTL3_CTM_HALWAYS_V    0x00000200 // Hysteresis Always
#define ADC_DCCTL3_CTM_HONCE_V      0x00000300 // Hysteresis Once
#define ADC_DCCTL3_CIE_B            0x00000010 // Comparison Interrupt Enable
#define ADC_DCCTL3_CIC_M            0x0000000C // Comparison Interrupt Condition
#define ADC_DCCTL3_CIC_S            2          // Comparison Interrupt Condition
#define ADC_DCCTL3_CIC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL3_CIC_MID_V        0x00000004 // Mid Band
#define ADC_DCCTL3_CIC_HIGH_V       0x0000000C // High Band
#define ADC_DCCTL3_CIM_M            0x00000003 // Comparison Interrupt Mode
#define ADC_DCCTL3_CIM_S            0          // Comparison Interrupt Mode
#define ADC_DCCTL3_CIM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL3_CIM_ONCE_V       0x00000001 // Once
#define ADC_DCCTL3_CIM_HALWAYS_V    0x00000002 // Hysteresis Always
#define ADC_DCCTL3_CIM_HONCE_V      0x00000003 // Hysteresis Once

/* Bit/Fields in Register DCCTL4 of Module ADC                                */
#define ADC_DCCTL4_CTE_B            0x00001000 // Comparison Trigger Enable
#define ADC_DCCTL4_CTC_M            0x00000C00 // Comparison Trigger Condition
#define ADC_DCCTL4_CTC_S            10         // Comparison Trigger Condition
#define ADC_DCCTL4_CTC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL4_CTC_MID_V        0x00000400 // Mid Band
#define ADC_DCCTL4_CTC_HIGH_V       0x00000C00 // High Band
#define ADC_DCCTL4_CTM_M            0x00000300 // Comparison Trigger Mode
#define ADC_DCCTL4_CTM_S            8          // Comparison Trigger Mode
#define ADC_DCCTL4_CTM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL4_CTM_ONCE_V       0x00000100 // Once
#define ADC_DCCTL4_CTM_HALWAYS_V    0x00000200 // Hysteresis Always
#define ADC_DCCTL4_CTM_HONCE_V      0x00000300 // Hysteresis Once
#define ADC_DCCTL4_CIE_B            0x00000010 // Comparison Interrupt Enable
#define ADC_DCCTL4_CIC_M            0x0000000C // Comparison Interrupt Condition
#define ADC_DCCTL4_CIC_S            2          // Comparison Interrupt Condition
#define ADC_DCCTL4_CIC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL4_CIC_MID_V        0x00000004 // Mid Band
#define ADC_DCCTL4_CIC_HIGH_V       0x0000000C // High Band
#define ADC_DCCTL4_CIM_M            0x00000003 // Comparison Interrupt Mode
#define ADC_DCCTL4_CIM_S            0          // Comparison Interrupt Mode
#define ADC_DCCTL4_CIM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL4_CIM_ONCE_V       0x00000001 // Once
#define ADC_DCCTL4_CIM_HALWAYS_V    0x00000002 // Hysteresis Always
#define ADC_DCCTL4_CIM_HONCE_V      0x00000003 // Hysteresis Once

/* Bit/Fields in Register DCCTL5 of Module ADC                                */
#define ADC_DCCTL5_CTE_B            0x00001000 // Comparison Trigger Enable
#define ADC_DCCTL5_CTC_M            0x00000C00 // Comparison Trigger Condition
#define ADC_DCCTL5_CTC_S            10         // Comparison Trigger Condition
#define ADC_DCCTL5_CTC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL5_CTC_MID_V        0x00000400 // Mid Band
#define ADC_DCCTL5_CTC_HIGH_V       0x00000C00 // High Band
#define ADC_DCCTL5_CTM_M            0x00000300 // Comparison Trigger Mode
#define ADC_DCCTL5_CTM_S            8          // Comparison Trigger Mode
#define ADC_DCCTL5_CTM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL5_CTM_ONCE_V       0x00000100 // Once
#define ADC_DCCTL5_CTM_HALWAYS_V    0x00000200 // Hysteresis Always
#define ADC_DCCTL5_CTM_HONCE_V      0x00000300 // Hysteresis Once
#define ADC_DCCTL5_CIE_B            0x00000010 // Comparison Interrupt Enable
#define ADC_DCCTL5_CIC_M            0x0000000C // Comparison Interrupt Condition
#define ADC_DCCTL5_CIC_S            2          // Comparison Interrupt Condition
#define ADC_DCCTL5_CIC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL5_CIC_MID_V        0x00000004 // Mid Band
#define ADC_DCCTL5_CIC_HIGH_V       0x0000000C // High Band
#define ADC_DCCTL5_CIM_M            0x00000003 // Comparison Interrupt Mode
#define ADC_DCCTL5_CIM_S            0          // Comparison Interrupt Mode
#define ADC_DCCTL5_CIM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL5_CIM_ONCE_V       0x00000001 // Once
#define ADC_DCCTL5_CIM_HALWAYS_V    0x00000002 // Hysteresis Always
#define ADC_DCCTL5_CIM_HONCE_V      0x00000003 // Hysteresis Once

/* Bit/Fields in Register DCCTL6 of Module ADC                                */
#define ADC_DCCTL6_CTE_B            0x00001000 // Comparison Trigger Enable
#define ADC_DCCTL6_CTC_M            0x00000C00 // Comparison Trigger Condition
#define ADC_DCCTL6_CTC_S            10         // Comparison Trigger Condition
#define ADC_DCCTL6_CTC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL6_CTC_MID_V        0x00000400 // Mid Band
#define ADC_DCCTL6_CTC_HIGH_V       0x00000C00 // High Band
#define ADC_DCCTL6_CTM_M            0x00000300 // Comparison Trigger Mode
#define ADC_DCCTL6_CTM_S            8          // Comparison Trigger Mode
#define ADC_DCCTL6_CTM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL6_CTM_ONCE_V       0x00000100 // Once
#define ADC_DCCTL6_CTM_HALWAYS_V    0x00000200 // Hysteresis Always
#define ADC_DCCTL6_CTM_HONCE_V      0x00000300 // Hysteresis Once
#define ADC_DCCTL6_CIE_B            0x00000010 // Comparison Interrupt Enable
#define ADC_DCCTL6_CIC_M            0x0000000C // Comparison Interrupt Condition
#define ADC_DCCTL6_CIC_S            2          // Comparison Interrupt Condition
#define ADC_DCCTL6_CIC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL6_CIC_MID_V        0x00000004 // Mid Band
#define ADC_DCCTL6_CIC_HIGH_V       0x0000000C // High Band
#define ADC_DCCTL6_CIM_M            0x00000003 // Comparison Interrupt Mode
#define ADC_DCCTL6_CIM_S            0          // Comparison Interrupt Mode
#define ADC_DCCTL6_CIM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL6_CIM_ONCE_V       0x00000001 // Once
#define ADC_DCCTL6_CIM_HALWAYS_V    0x00000002 // Hysteresis Always
#define ADC_DCCTL6_CIM_HONCE_V      0x00000003 // Hysteresis Once

/* Bit/Fields in Register DCCTL7 of Module ADC                                */
#define ADC_DCCTL7_CTE_B            0x00001000 // Comparison Trigger Enable
#define ADC_DCCTL7_CTC_M            0x00000C00 // Comparison Trigger Condition
#define ADC_DCCTL7_CTC_S            10         // Comparison Trigger Condition
#define ADC_DCCTL7_CTC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL7_CTC_MID_V        0x00000400 // Mid Band
#define ADC_DCCTL7_CTC_HIGH_V       0x00000C00 // High Band
#define ADC_DCCTL7_CTM_M            0x00000300 // Comparison Trigger Mode
#define ADC_DCCTL7_CTM_S            8          // Comparison Trigger Mode
#define ADC_DCCTL7_CTM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL7_CTM_ONCE_V       0x00000100 // Once
#define ADC_DCCTL7_CTM_HALWAYS_V    0x00000200 // Hysteresis Always
#define ADC_DCCTL7_CTM_HONCE_V      0x00000300 // Hysteresis Once
#define ADC_DCCTL7_CIE_B            0x00000010 // Comparison Interrupt Enable
#define ADC_DCCTL7_CIC_M            0x0000000C // Comparison Interrupt Condition
#define ADC_DCCTL7_CIC_S            2          // Comparison Interrupt Condition
#define ADC_DCCTL7_CIC_LOW_V        0x00000000 // Low Band
#define ADC_DCCTL7_CIC_MID_V        0x00000004 // Mid Band
#define ADC_DCCTL7_CIC_HIGH_V       0x0000000C // High Band
#define ADC_DCCTL7_CIM_M            0x00000003 // Comparison Interrupt Mode
#define ADC_DCCTL7_CIM_S            0          // Comparison Interrupt Mode
#define ADC_DCCTL7_CIM_ALWAYS_V     0x00000000 // Always
#define ADC_DCCTL7_CIM_ONCE_V       0x00000001 // Once
#define ADC_DCCTL7_CIM_HALWAYS_V    0x00000002 // Hysteresis Always
#define ADC_DCCTL7_CIM_HONCE_V      0x00000003 // Hysteresis Once

/* Bit/Fields in Register DCCMP0 of Module ADC                                */
#define ADC_DCCMP0_COMP1_M          0x0FFF0000 // Compare 1
#define ADC_DCCMP0_COMP1_S          16         // Compare 1
#define ADC_DCCMP0_COMP0_M          0x00000FFF // Compare 0
#define ADC_DCCMP0_COMP0_S          0          // Compare 0

/* Bit/Fields in Register DCCMP1 of Module ADC                                */
#define ADC_DCCMP1_COMP1_M          0x0FFF0000 // Compare 1
#define ADC_DCCMP1_COMP1_S          16         // Compare 1
#define ADC_DCCMP1_COMP0_M          0x00000FFF // Compare 0
#define ADC_DCCMP1_COMP0_S          0          // Compare 0

/* Bit/Fields in Register DCCMP2 of Module ADC                                */
#define ADC_DCCMP2_COMP1_M          0x0FFF0000 // Compare 1
#define ADC_DCCMP2_COMP1_S          16         // Compare 1
#define ADC_DCCMP2_COMP0_M          0x00000FFF // Compare 0
#define ADC_DCCMP2_COMP0_S          0          // Compare 0

/* Bit/Fields in Register DCCMP3 of Module ADC                                */
#define ADC_DCCMP3_COMP1_M          0x0FFF0000 // Compare 1
#define ADC_DCCMP3_COMP1_S          16         // Compare 1
#define ADC_DCCMP3_COMP0_M          0x00000FFF // Compare 0
#define ADC_DCCMP3_COMP0_S          0          // Compare 0

/* Bit/Fields in Register DCCMP4 of Module ADC                                */
#define ADC_DCCMP4_COMP1_M          0x0FFF0000 // Compare 1
#define ADC_DCCMP4_COMP1_S          16         // Compare 1
#define ADC_DCCMP4_COMP0_M          0x00000FFF // Compare 0
#define ADC_DCCMP4_COMP0_S          0          // Compare 0

/* Bit/Fields in Register DCCMP5 of Module ADC                                */
#define ADC_DCCMP5_COMP1_M          0x0FFF0000 // Compare 1
#define ADC_DCCMP5_COMP1_S          16         // Compare 1
#define ADC_DCCMP5_COMP0_M          0x00000FFF // Compare 0
#define ADC_DCCMP5_COMP0_S          0          // Compare 0

/* Bit/Fields in Register DCCMP6 of Module ADC                                */
#define ADC_DCCMP6_COMP1_M          0x0FFF0000 // Compare 1
#define ADC_DCCMP6_COMP1_S          16         // Compare 1
#define ADC_DCCMP6_COMP0_M          0x00000FFF // Compare 0
#define ADC_DCCMP6_COMP0_S          0          // Compare 0

/* Bit/Fields in Register DCCMP7 of Module ADC                                */
#define ADC_DCCMP7_COMP1_M          0x0FFF0000 // Compare 1
#define ADC_DCCMP7_COMP1_S          16         // Compare 1
#define ADC_DCCMP7_COMP0_M          0x00000FFF // Compare 0
#define ADC_DCCMP7_COMP0_S          0          // Compare 0

/* Bit/Fields in Register PP of Module ADC                                    */
#define ADC_PP_TS_B                 0x00800000 // Temperature Sensor
#define ADC_PP_RSL_M                0x007C0000 // Resolution
#define ADC_PP_RSL_S                18         // Resolution
#define ADC_PP_TYPE_M               0x00030000 // ADC Architecture
#define ADC_PP_TYPE_S               16         // ADC Architecture
#define ADC_PP_TYPE_SAR_V           0x00000000 // SAR
#define ADC_PP_DC_M                 0x0000FC00 // Digital Comparator Count
#define ADC_PP_DC_S                 10         // Digital Comparator Count
#define ADC_PP_CH_M                 0x000003F0 // ADC Channel Count
#define ADC_PP_CH_S                 4          // ADC Channel Count
#define ADC_PP_MSR_M                0x0000000F // Maximum ADC Sample Rate
#define ADC_PP_MSR_S                0          // Maximum ADC Sample Rate
#define ADC_PP_MSR_125K_V           0x00000001 // 125 ksps
#define ADC_PP_MSR_250K_V           0x00000003 // 250 ksps
#define ADC_PP_MSR_500K_V           0x00000005 // 500 ksps
#define ADC_PP_MSR_1M_V             0x00000007 // 1 Msps

/* Bit/Fields in Register PC of Module ADC                                    */
#define ADC_PC_SR_M                 0x0000000F // ADC Sample Rate
#define ADC_PC_SR_S                 0          // ADC Sample Rate
#define ADC_PC_SR_125K_V            0x00000001 // 125 ksps
#define ADC_PC_SR_250K_V            0x00000003 // 250 ksps
#define ADC_PC_SR_500K_V            0x00000005 // 500 ksps
#define ADC_PC_SR_1M_V              0x00000007 // 1 Msps

/* Bit/Fields in Register CC of Module ADC                                    */
#define ADC_CC_CS_M                 0x0000000F // ADC Clock Source
#define ADC_CC_CS_S                 0          // ADC Clock Source
#define ADC_CC_CS_SYSPLL_V          0x00000000 // PLL VCO divided by CLKDIV
#define ADC_CC_CS_PIOSC_V           0x00000001 // PIOSC


/******************************************************************************/
/*                                                                            */
/*                      COMP                                                  */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register ACMIS of Module COMP                                */
#define COMP_ACMIS_IN1_B            0x00000002 // Comparator 1 Masked Interrupt
#define COMP_ACMIS_IN0_B            0x00000001 // Comparator 0 Masked Interrupt

/* Bit/Fields in Register ACRIS of Module COMP                                */
#define COMP_ACRIS_IN1_B            0x00000002 // Comparator 1 Interrupt Status
#define COMP_ACRIS_IN0_B            0x00000001 // Comparator 0 Interrupt Status

/* Bit/Fields in Register ACINTEN of Module COMP                              */
#define COMP_ACINTEN_IN1_B          0x00000002 // Comparator 1 Interrupt Enable
#define COMP_ACINTEN_IN0_B          0x00000001 // Comparator 0 Interrupt Enable

/* Bit/Fields in Register ACREFCTL of Module COMP                             */
#define COMP_ACREFCTL_EN_B          0x00000200 // Resistor Ladder Enable
#define COMP_ACREFCTL_RNG_B         0x00000100 // Resistor Ladder Range
#define COMP_ACREFCTL_VREF_M        0x0000000F // Resistor Ladder Voltage Ref
#define COMP_ACREFCTL_VREF_S        0          // Resistor Ladder Voltage Ref

/* Bit/Fields in Register ACSTAT0 of Module COMP                              */
#define COMP_ACSTAT0_OVAL_B         0x00000002 // Comparator Output Value

/* Bit/Fields in Register ACCTL0 of Module COMP                               */
#define COMP_ACCTL0_TOEN_B          0x00000800 // Trigger Output Enable
#define COMP_ACCTL0_ASRCP_M         0x00000600 // Analog Source Positive
#define COMP_ACCTL0_ASRCP_S         9          // Analog Source Positive
#define COMP_ACCTL0_ASRCP_PIN_V     0x00000000 // Pin value of Cn+
#define COMP_ACCTL0_ASRCP_PIN0_V    0x00000200 // Pin value of C0+
#define COMP_ACCTL0_ASRCP_REF_V     0x00000400 // Internal voltage reference
#define COMP_ACCTL0_TSLVAL_B        0x00000080 // Trigger Sense Level Value
#define COMP_ACCTL0_TSEN_M          0x00000060 // Trigger Sense
#define COMP_ACCTL0_TSEN_S          5          // Trigger Sense
#define COMP_ACCTL0_TSEN_LEVEL_V    0x00000000 // Level sense, see TSLVAL
#define COMP_ACCTL0_TSEN_FALL_V     0x00000020 // Falling edge
#define COMP_ACCTL0_TSEN_RISE_V     0x00000040 // Rising edge
#define COMP_ACCTL0_TSEN_BOTH_V     0x00000060 // Either edge
#define COMP_ACCTL0_ISLVAL_B        0x00000010 // Interrupt Sense Level Value
#define COMP_ACCTL0_ISEN_M          0x0000000C // Interrupt Sense
#define COMP_ACCTL0_ISEN_S          2          // Interrupt Sense
#define COMP_ACCTL0_ISEN_LEVEL_V    0x00000000 // Level sense, see ISLVAL
#define COMP_ACCTL0_ISEN_FALL_V     0x00000004 // Falling edge
#define COMP_ACCTL0_ISEN_RISE_V     0x00000008 // Rising edge
#define COMP_ACCTL0_ISEN_BOTH_V     0x0000000C // Either edge
#define COMP_ACCTL0_CINV_B          0x00000002 // Comparator Output Invert

/* Bit/Fields in Register ACSTAT1 of Module COMP                              */
#define COMP_ACSTAT1_OVAL_B         0x00000002 // Comparator Output Value

/* Bit/Fields in Register ACCTL1 of Module COMP                               */
#define COMP_ACCTL1_TOEN_B          0x00000800 // Trigger Output Enable
#define COMP_ACCTL1_ASRCP_M         0x00000600 // Analog Source Positive
#define COMP_ACCTL1_ASRCP_S         9          // Analog Source Positive
#define COMP_ACCTL1_ASRCP_PIN_V     0x00000000 // Pin value of Cn+
#define COMP_ACCTL1_ASRCP_PIN0_V    0x00000200 // Pin value of C0+
#define COMP_ACCTL1_ASRCP_REF_V     0x00000400 // Internal voltage reference
#define COMP_ACCTL1_TSLVAL_B        0x00000080 // Trigger Sense Level Value
#define COMP_ACCTL1_TSEN_M          0x00000060 // Trigger Sense
#define COMP_ACCTL1_TSEN_S          5          // Trigger Sense
#define COMP_ACCTL1_TSEN_LEVEL_V    0x00000000 // Level sense, see TSLVAL
#define COMP_ACCTL1_TSEN_FALL_V     0x00000020 // Falling edge
#define COMP_ACCTL1_TSEN_RISE_V     0x00000040 // Rising edge
#define COMP_ACCTL1_TSEN_BOTH_V     0x00000060 // Either edge
#define COMP_ACCTL1_ISLVAL_B        0x00000010 // Interrupt Sense Level Value
#define COMP_ACCTL1_ISEN_M          0x0000000C // Interrupt Sense
#define COMP_ACCTL1_ISEN_S          2          // Interrupt Sense
#define COMP_ACCTL1_ISEN_LEVEL_V    0x00000000 // Level sense, see ISLVAL
#define COMP_ACCTL1_ISEN_FALL_V     0x00000004 // Falling edge
#define COMP_ACCTL1_ISEN_RISE_V     0x00000008 // Rising edge
#define COMP_ACCTL1_ISEN_BOTH_V     0x0000000C // Either edge
#define COMP_ACCTL1_CINV_B          0x00000002 // Comparator Output Invert

/* Bit/Fields in Register PP of Module COMP                                   */
#define COMP_PP_C1O_B               0x00020000 // Comparator Output 1 Present
#define COMP_PP_C0O_B               0x00010000 // Comparator Output 0 Present
#define COMP_PP_CMP1_B              0x00000002 // Comparator 1 Present
#define COMP_PP_CMP0_B              0x00000001 // Comparator 0 Present


/******************************************************************************/
/*                                                                            */
/*                      CAN                                                   */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register CTL of Module CAN                                   */
#define CAN_CTL_TEST_B              0x00000080 // Test Mode Enable
#define CAN_CTL_CCE_B               0x00000040 // Configuration Change Enable
#define CAN_CTL_DAR_B               0x00000020 // Disable Automatic-Retransmission
#define CAN_CTL_EIE_B               0x00000008 // Error Interrupt Enable
#define CAN_CTL_SIE_B               0x00000004 // Status Interrupt Enable
#define CAN_CTL_IE_B                0x00000002 // CAN Interrupt Enable
#define CAN_CTL_INIT_B              0x00000001 // Initialization

/* Bit/Fields in Register STS of Module CAN                                   */
#define CAN_STS_BOFF_B              0x00000080 // Bus-Off Status
#define CAN_STS_EWARN_B             0x00000040 // Warning Status
#define CAN_STS_EPASS_B             0x00000020 // Error Passive
#define CAN_STS_RXOK_B              0x00000010 // Received a Message Successfully
#define CAN_STS_TXOK_B              0x00000008 // Transmitted a Message
#define CAN_STS_LEC_M               0x00000007 // Last Error Code
#define CAN_STS_LEC_S               0          // Last Error Code
#define CAN_STS_LEC_NONE_V          0x00000000 // No Error
#define CAN_STS_LEC_STUFF_V         0x00000001 // Stuff Error
#define CAN_STS_LEC_FORM_V          0x00000002 // Format Error
#define CAN_STS_LEC_ACK_V           0x00000003 // ACK Error
#define CAN_STS_LEC_BIT1_V          0x00000004 // Bit 1 Error
#define CAN_STS_LEC_BIT0_V          0x00000005 // Bit 0 Error
#define CAN_STS_LEC_CRC_V           0x00000006 // CRC Error
#define CAN_STS_LEC_NOEVENT_V       0x00000007 // No Event

/* Bit/Fields in Register ERR of Module CAN                                   */
#define CAN_ERR_RP_B                0x00008000 // Received Error Passive
#define CAN_ERR_REC_M               0x00007F00 // Receive Error Counter
#define CAN_ERR_REC_S               8          // Receive Error Counter
#define CAN_ERR_TEC_M               0x000000FF // Transmit Error Counter
#define CAN_ERR_TEC_S               0          // Transmit Error Counter

/* Bit/Fields in Register BIT of Module CAN                                   */
#define CAN_BIT_TSEG2_M             0x00007000 // Time Segment after Sample Point
#define CAN_BIT_TSEG2_S             12         // Time Segment after Sample Point
#define CAN_BIT_TSEG1_M             0x00000F00 // Time Segment Before Sample Point
#define CAN_BIT_TSEG1_S             8          // Time Segment Before Sample Point
#define CAN_BIT_SJW_M               0x000000C0 // (Re)Synchronization Jump Width
#define CAN_BIT_SJW_S               6          // (Re)Synchronization Jump Width
#define CAN_BIT_BRP_M               0x0000003F // Baud Rate Prescaler
#define CAN_BIT_BRP_S               0          // Baud Rate Prescaler

/* Bit/Fields in Register INT of Module CAN                                   */
#define CAN_INT_INTID_M             0x0000FFFF // Interrupt Identifier
#define CAN_INT_INTID_S             0          // Interrupt Identifier
#define CAN_INT_INTID_NONE_V        0x00000000 // No interrupt pending
#define CAN_INT_INTID_STATUS_V      0x00008000 // Status Interrupt

/* Bit/Fields in Register TST of Module CAN                                   */
#define CAN_TST_RX_B                0x00000080 // Receive Observation
#define CAN_TST_TX_M                0x00000060 // Transmit Control
#define CAN_TST_TX_S                5          // Transmit Control
#define CAN_TST_TX_CANCTL_V         0x00000000 // CAN Module Control
#define CAN_TST_TX_SAMPLE_V         0x00000020 // Sample Point
#define CAN_TST_TX_DOMINANT_V       0x00000040 // Driven Low
#define CAN_TST_TX_RECESSIVE_V      0x00000060 // Driven High
#define CAN_TST_LBACK_B             0x00000010 // Loopback Mode
#define CAN_TST_SILENT_B            0x00000008 // Silent Mode
#define CAN_TST_BASIC_B             0x00000004 // Basic Mode

/* Bit/Fields in Register BRPE of Module CAN                                  */
#define CAN_BRPE_BRPE_M             0x0000000F // Baud Rate Prescaler Extension
#define CAN_BRPE_BRPE_S             0          // Baud Rate Prescaler Extension

/* Bit/Fields in Register IF1CRQ of Module CAN                                */
#define CAN_IF1CRQ_BUSY_B           0x00008000 // Busy Flag
#define CAN_IF1CRQ_MNUM_M           0x0000003F // Message Number
#define CAN_IF1CRQ_MNUM_S           0          // Message Number

/* Bit/Fields in Register IF1CMSK of Module CAN                               */
#define CAN_IF1CMSK_WRNRD_B         0x00000080 // Write, Not Read
#define CAN_IF1CMSK_MASK_B          0x00000040 // Access Mask Bits
#define CAN_IF1CMSK_ARB_B           0x00000020 // Access Arbitration Bits
#define CAN_IF1CMSK_CONTROL_B       0x00000010 // Access Control Bits
#define CAN_IF1CMSK_CLRINTPND_B     0x00000008 // Clear Interrupt Pending Bit
#define CAN_IF1CMSK_NEWDAT_B        0x00000004 // Access New Data
#define CAN_IF1CMSK_TXRQST_B        0x00000004 // Access Transmission Request
#define CAN_IF1CMSK_DATAA_B         0x00000002 // Access Data Byte 0 to 3
#define CAN_IF1CMSK_DATAB_B         0x00000001 // Access Data Byte 4 to 7

/* Bit/Fields in Register IF1MSK1 of Module CAN                               */
#define CAN_IF1MSK1_IDMSK_M         0x0000FFFF // Identifier Mask
#define CAN_IF1MSK1_IDMSK_S         0          // Identifier Mask

/* Bit/Fields in Register IF1MSK2 of Module CAN                               */
#define CAN_IF1MSK2_MXTD_B          0x00008000 // Mask Extended Identifier
#define CAN_IF1MSK2_MDIR_B          0x00004000 // Mask Message Direction
#define CAN_IF1MSK2_IDMSK_M         0x00001FFF // Identifier Mask
#define CAN_IF1MSK2_IDMSK_S         0          // Identifier Mask

/* Bit/Fields in Register IF1ARB1 of Module CAN                               */
#define CAN_IF1ARB1_ID_M            0x0000FFFF // Message Identifier
#define CAN_IF1ARB1_ID_S            0          // Message Identifier

/* Bit/Fields in Register IF1ARB2 of Module CAN                               */
#define CAN_IF1ARB2_MSGVAL_B        0x00008000 // Message Valid
#define CAN_IF1ARB2_XTD_B           0x00004000 // Extended Identifier
#define CAN_IF1ARB2_DIR_B           0x00002000 // Message Direction
#define CAN_IF1ARB2_ID_M            0x00001FFF // Message Identifier
#define CAN_IF1ARB2_ID_S            0          // Message Identifier

/* Bit/Fields in Register IF1MCTL of Module CAN                               */
#define CAN_IF1MCTL_NEWDAT_B        0x00008000 // New Data
#define CAN_IF1MCTL_MSGLST_B        0x00004000 // Message Lost
#define CAN_IF1MCTL_INTPND_B        0x00002000 // Interrupt Pending
#define CAN_IF1MCTL_UMASK_B         0x00001000 // Use Acceptance Mask
#define CAN_IF1MCTL_TXIE_B          0x00000800 // Transmit Interrupt Enable
#define CAN_IF1MCTL_RXIE_B          0x00000400 // Receive Interrupt Enable
#define CAN_IF1MCTL_RMTEN_B         0x00000200 // Remote Enable
#define CAN_IF1MCTL_TXRQST_B        0x00000100 // Transmit Request
#define CAN_IF1MCTL_EOB_B           0x00000080 // End of Buffer
#define CAN_IF1MCTL_DLC_M           0x0000000F // Data Length Code
#define CAN_IF1MCTL_DLC_S           0          // Data Length Code

/* Bit/Fields in Register IF1DA1 of Module CAN                                */
#define CAN_IF1DA1_DATA_M           0x0000FFFF // Data
#define CAN_IF1DA1_DATA_S           0          // Data

/* Bit/Fields in Register IF1DA2 of Module CAN                                */
#define CAN_IF1DA2_DATA_M           0x0000FFFF // Data
#define CAN_IF1DA2_DATA_S           0          // Data

/* Bit/Fields in Register IF1DB1 of Module CAN                                */
#define CAN_IF1DB1_DATA_M           0x0000FFFF // Data
#define CAN_IF1DB1_DATA_S           0          // Data

/* Bit/Fields in Register IF1DB2 of Module CAN                                */
#define CAN_IF1DB2_DATA_M           0x0000FFFF // Data
#define CAN_IF1DB2_DATA_S           0          // Data

/* Bit/Fields in Register IF2CRQ of Module CAN                                */
#define CAN_IF2CRQ_BUSY_B           0x00008000 // Busy Flag
#define CAN_IF2CRQ_MNUM_M           0x0000003F // Message Number
#define CAN_IF2CRQ_MNUM_S           0          // Message Number

/* Bit/Fields in Register IF2CMSK of Module CAN                               */
#define CAN_IF2CMSK_WRNRD_B         0x00000080 // Write, Not Read
#define CAN_IF2CMSK_MASK_B          0x00000040 // Access Mask Bits
#define CAN_IF2CMSK_ARB_B           0x00000020 // Access Arbitration Bits
#define CAN_IF2CMSK_CONTROL_B       0x00000010 // Access Control Bits
#define CAN_IF2CMSK_CLRINTPND_B     0x00000008 // Clear Interrupt Pending Bit
#define CAN_IF2CMSK_NEWDAT_B        0x00000004 // Access New Data
#define CAN_IF2CMSK_TXRQST_B        0x00000004 // Access Transmission Request
#define CAN_IF2CMSK_DATAA_B         0x00000002 // Access Data Byte 0 to 3
#define CAN_IF2CMSK_DATAB_B         0x00000001 // Access Data Byte 4 to 7

/* Bit/Fields in Register IF2MSK1 of Module CAN                               */
#define CAN_IF2MSK1_IDMSK_M         0x0000FFFF // Identifier Mask
#define CAN_IF2MSK1_IDMSK_S         0          // Identifier Mask

/* Bit/Fields in Register IF2MSK2 of Module CAN                               */
#define CAN_IF2MSK2_MXTD_B          0x00008000 // Mask Extended Identifier
#define CAN_IF2MSK2_MDIR_B          0x00004000 // Mask Message Direction
#define CAN_IF2MSK2_IDMSK_M         0x00001FFF // Identifier Mask
#define CAN_IF2MSK2_IDMSK_S         0          // Identifier Mask

/* Bit/Fields in Register IF2ARB1 of Module CAN                               */
#define CAN_IF2ARB1_ID_M            0x0000FFFF // Message Identifier
#define CAN_IF2ARB1_ID_S            0          // Message Identifier

/* Bit/Fields in Register IF2ARB2 of Module CAN                               */
#define CAN_IF2ARB2_MSGVAL_B        0x00008000 // Message Valid
#define CAN_IF2ARB2_XTD_B           0x00004000 // Extended Identifier
#define CAN_IF2ARB2_DIR_B           0x00002000 // Message Direction
#define CAN_IF2ARB2_ID_M            0x00001FFF // Message Identifier
#define CAN_IF2ARB2_ID_S            0          // Message Identifier

/* Bit/Fields in Register IF2MCTL of Module CAN                               */
#define CAN_IF2MCTL_NEWDAT_B        0x00008000 // New Data
#define CAN_IF2MCTL_MSGLST_B        0x00004000 // Message Lost
#define CAN_IF2MCTL_INTPND_B        0x00002000 // Interrupt Pending
#define CAN_IF2MCTL_UMASK_B         0x00001000 // Use Acceptance Mask
#define CAN_IF2MCTL_TXIE_B          0x00000800 // Transmit Interrupt Enable
#define CAN_IF2MCTL_RXIE_B          0x00000400 // Receive Interrupt Enable
#define CAN_IF2MCTL_RMTEN_B         0x00000200 // Remote Enable
#define CAN_IF2MCTL_TXRQST_B        0x00000100 // Transmit Request
#define CAN_IF2MCTL_EOB_B           0x00000080 // End of Buffer
#define CAN_IF2MCTL_DLC_M           0x0000000F // Data Length Code
#define CAN_IF2MCTL_DLC_S           0          // Data Length Code

/* Bit/Fields in Register IF2DA1 of Module CAN                                */
#define CAN_IF2DA1_DATA_M           0x0000FFFF // Data
#define CAN_IF2DA1_DATA_S           0          // Data

/* Bit/Fields in Register IF2DA2 of Module CAN                                */
#define CAN_IF2DA2_DATA_M           0x0000FFFF // Data
#define CAN_IF2DA2_DATA_S           0          // Data

/* Bit/Fields in Register IF2DB1 of Module CAN                                */
#define CAN_IF2DB1_DATA_M           0x0000FFFF // Data
#define CAN_IF2DB1_DATA_S           0          // Data

/* Bit/Fields in Register IF2DB2 of Module CAN                                */
#define CAN_IF2DB2_DATA_M           0x0000FFFF // Data
#define CAN_IF2DB2_DATA_S           0          // Data

/* Bit/Fields in Register TXRQ1 of Module CAN                                 */
#define CAN_TXRQ1_TXRQST_M          0x0000FFFF // Transmission Request Bits
#define CAN_TXRQ1_TXRQST_S          0          // Transmission Request Bits

/* Bit/Fields in Register TXRQ2 of Module CAN                                 */
#define CAN_TXRQ2_TXRQST_M          0x0000FFFF // Transmission Request Bits
#define CAN_TXRQ2_TXRQST_S          0          // Transmission Request Bits

/* Bit/Fields in Register NWDA1 of Module CAN                                 */
#define CAN_NWDA1_NEWDAT_M          0x0000FFFF // New Data Bits
#define CAN_NWDA1_NEWDAT_S          0          // New Data Bits

/* Bit/Fields in Register NWDA2 of Module CAN                                 */
#define CAN_NWDA2_NEWDAT_M          0x0000FFFF // New Data Bits
#define CAN_NWDA2_NEWDAT_S          0          // New Data Bits

/* Bit/Fields in Register MSG1INT of Module CAN                               */
#define CAN_MSG1INT_INTPND_M        0x0000FFFF // Interrupt Pending Bits
#define CAN_MSG1INT_INTPND_S        0          // Interrupt Pending Bits

/* Bit/Fields in Register MSG2INT of Module CAN                               */
#define CAN_MSG2INT_INTPND_M        0x0000FFFF // Interrupt Pending Bits
#define CAN_MSG2INT_INTPND_S        0          // Interrupt Pending Bits

/* Bit/Fields in Register MSG1VAL of Module CAN                               */
#define CAN_MSG1VAL_MSGVAL_M        0x0000FFFF // Message Valid Bits
#define CAN_MSG1VAL_MSGVAL_S        0          // Message Valid Bits

/* Bit/Fields in Register MSG2VAL of Module CAN                               */
#define CAN_MSG2VAL_MSGVAL_M        0x0000FFFF // Message Valid Bits
#define CAN_MSG2VAL_MSGVAL_S        0          // Message Valid Bits


/******************************************************************************/
/*                                                                            */
/*                      USB                                                   */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register FADDR of Module USB                                 */
#define USB_FADDR_M                 0x0000007F // Function Address
#define USB_FADDR_S                 0          // Function Address

/* Bit/Fields in Register POWER of Module USB                                 */
#define USB_POWER_ISOUP_B           0x00000080 // Isochronous Update
#define USB_POWER_SOFTCONN_B        0x00000040 // Soft Connect/Disconnect
#define USB_POWER_RESET_B           0x00000008 // RESET Signaling
#define USB_POWER_RESUME_B          0x00000004 // RESUME Signaling
#define USB_POWER_SUSPEND_B         0x00000002 // SUSPEND Mode
#define USB_POWER_PWRDNPHY_B        0x00000001 // Power Down PHY

/* Bit/Fields in Register TXIS of Module USB                                  */
#define USB_TXIS_EP7_B              0x00000080 // TX Endpoint 7 Interrupt
#define USB_TXIS_EP6_B              0x00000040 // TX Endpoint 6 Interrupt
#define USB_TXIS_EP5_B              0x00000020 // TX Endpoint 5 Interrupt
#define USB_TXIS_EP4_B              0x00000010 // TX Endpoint 4 Interrupt
#define USB_TXIS_EP3_B              0x00000008 // TX Endpoint 3 Interrupt
#define USB_TXIS_EP2_B              0x00000004 // TX Endpoint 2 Interrupt
#define USB_TXIS_EP1_B              0x00000002 // TX Endpoint 1 Interrupt
#define USB_TXIS_EP0_B              0x00000001 // TX and RX Endpoint 0 Interrupt

/* Bit/Fields in Register RXIS of Module USB                                  */
#define USB_RXIS_EP7_B              0x00000080 // RX Endpoint 7 Interrupt
#define USB_RXIS_EP6_B              0x00000040 // RX Endpoint 6 Interrupt
#define USB_RXIS_EP5_B              0x00000020 // RX Endpoint 5 Interrupt
#define USB_RXIS_EP4_B              0x00000010 // RX Endpoint 4 Interrupt
#define USB_RXIS_EP3_B              0x00000008 // RX Endpoint 3 Interrupt
#define USB_RXIS_EP2_B              0x00000004 // RX Endpoint 2 Interrupt
#define USB_RXIS_EP1_B              0x00000002 // RX Endpoint 1 Interrupt

/* Bit/Fields in Register TXIE of Module USB                                  */
#define USB_TXIE_EP7_B              0x00000080 // TX Endpoint 7 Interrupt Enable
#define USB_TXIE_EP6_B              0x00000040 // TX Endpoint 6 Interrupt Enable
#define USB_TXIE_EP5_B              0x00000020 // TX Endpoint 5 Interrupt Enable
#define USB_TXIE_EP4_B              0x00000010 // TX Endpoint 4 Interrupt Enable
#define USB_TXIE_EP3_B              0x00000008 // TX Endpoint 3 Interrupt Enable
#define USB_TXIE_EP2_B              0x00000004 // TX Endpoint 2 Interrupt Enable
#define USB_TXIE_EP1_B              0x00000002 // TX Endpoint 1 Interrupt Enable
#define USB_TXIE_EP0_B              0x00000001 // TX and RX Endpoint 0 Interrupt

/* Bit/Fields in Register RXIE of Module USB                                  */
#define USB_RXIE_EP7_B              0x00000080 // RX Endpoint 7 Interrupt Enable
#define USB_RXIE_EP6_B              0x00000040 // RX Endpoint 6 Interrupt Enable
#define USB_RXIE_EP5_B              0x00000020 // RX Endpoint 5 Interrupt Enable
#define USB_RXIE_EP4_B              0x00000010 // RX Endpoint 4 Interrupt Enable
#define USB_RXIE_EP3_B              0x00000008 // RX Endpoint 3 Interrupt Enable
#define USB_RXIE_EP2_B              0x00000004 // RX Endpoint 2 Interrupt Enable
#define USB_RXIE_EP1_B              0x00000002 // RX Endpoint 1 Interrupt Enable

/* Bit/Fields in Register IS of Module USB                                    */
#define USB_IS_VBUSERR_B            0x00000080 // VBUS Error (OTG only)
#define USB_IS_SESREQ_B             0x00000040 // SESSION REQUEST (OTG only)
#define USB_IS_DISCON_B             0x00000020 // Session Disconnect (OTG only)
#define USB_IS_CONN_B               0x00000010 // Session Connect
#define USB_IS_SOF_B                0x00000008 // Start of Frame
#define USB_IS_BABBLE_B             0x00000004 // Babble Detected
#define USB_IS_RESET_B              0x00000004 // RESET Signaling Detected
#define USB_IS_RESUME_B             0x00000002 // RESUME Signaling Detected
#define USB_IS_SUSPEND_B            0x00000001 // SUSPEND Signaling Detected

/* Bit/Fields in Register IE of Module USB                                    */
#define USB_IE_VBUSERR_B            0x00000080 // Enable VBUS Error Interrupt (OTG
#define USB_IE_SESREQ_B             0x00000040 // Enable Session Request (OTG
#define USB_IE_DISCON_B             0x00000020 // Enable Disconnect Interrupt
#define USB_IE_CONN_B               0x00000010 // Enable Connect Interrupt
#define USB_IE_SOF_B                0x00000008 // Enable Start-of-Frame Interrupt
#define USB_IE_BABBLE_B             0x00000004 // Enable Babble Interrupt
#define USB_IE_RESET_B              0x00000004 // Enable RESET Interrupt
#define USB_IE_RESUME_B             0x00000002 // Enable RESUME Interrupt
#define USB_IE_SUSPND_B             0x00000001 // Enable SUSPEND Interrupt

/* Bit/Fields in Register FRAME of Module USB                                 */
#define USB_FRAME_M                 0x000007FF // Frame Number
#define USB_FRAME_S                 0          // Frame Number

/* Bit/Fields in Register EPIDX of Module USB                                 */
#define USB_EPIDX_EPIDX_M           0x0000000F // Endpoint Index
#define USB_EPIDX_EPIDX_S           0          // Endpoint Index

/* Bit/Fields in Register TEST of Module USB                                  */
#define USB_TEST_FORCEH_B           0x00000080 // Force Host Mode
#define USB_TEST_FIFOACC_B          0x00000040 // FIFO Access
#define USB_TEST_FORCEFS_B          0x00000020 // Force Full-Speed Mode

/* Bit/Fields in Register FIFO0 of Module USB                                 */
#define USB_FIFO0_EPDATA_M          0xFFFFFFFF // Endpoint Data
#define USB_FIFO0_EPDATA_S          0          // Endpoint Data

/* Bit/Fields in Register FIFO1 of Module USB                                 */
#define USB_FIFO1_EPDATA_M          0xFFFFFFFF // Endpoint Data
#define USB_FIFO1_EPDATA_S          0          // Endpoint Data

/* Bit/Fields in Register FIFO2 of Module USB                                 */
#define USB_FIFO2_EPDATA_M          0xFFFFFFFF // Endpoint Data
#define USB_FIFO2_EPDATA_S          0          // Endpoint Data

/* Bit/Fields in Register FIFO3 of Module USB                                 */
#define USB_FIFO3_EPDATA_M          0xFFFFFFFF // Endpoint Data
#define USB_FIFO3_EPDATA_S          0          // Endpoint Data

/* Bit/Fields in Register FIFO4 of Module USB                                 */
#define USB_FIFO4_EPDATA_M          0xFFFFFFFF // Endpoint Data
#define USB_FIFO4_EPDATA_S          0          // Endpoint Data

/* Bit/Fields in Register FIFO5 of Module USB                                 */
#define USB_FIFO5_EPDATA_M          0xFFFFFFFF // Endpoint Data
#define USB_FIFO5_EPDATA_S          0          // Endpoint Data

/* Bit/Fields in Register FIFO6 of Module USB                                 */
#define USB_FIFO6_EPDATA_M          0xFFFFFFFF // Endpoint Data
#define USB_FIFO6_EPDATA_S          0          // Endpoint Data

/* Bit/Fields in Register FIFO7 of Module USB                                 */
#define USB_FIFO7_EPDATA_M          0xFFFFFFFF // Endpoint Data
#define USB_FIFO7_EPDATA_S          0          // Endpoint Data

/* Bit/Fields in Register DEVCTL of Module USB                                */
#define USB_DEVCTL_DEV_B            0x00000080 // Device Mode (OTG only)
#define USB_DEVCTL_FSDEV_B          0x00000040 // Full-Speed Device Detected
#define USB_DEVCTL_LSDEV_B          0x00000020 // Low-Speed Device Detected
#define USB_DEVCTL_VBUS_M           0x00000018 // VBUS Level (OTG only)
#define USB_DEVCTL_VBUS_S           3          // VBUS Level (OTG only)
#define USB_DEVCTL_VBUS_NONE_V      0x00000000 // Below SessionEnd
#define USB_DEVCTL_VBUS_SEND_V      0x00000008 // Above SessionEnd, below AValid
#define USB_DEVCTL_VBUS_AVALID_V    0x00000010 // Above AValid, below VBUSValid
#define USB_DEVCTL_VBUS_VALID_V     0x00000018 // Above VBUSValid
#define USB_DEVCTL_HOST_B           0x00000004 // Host Mode
#define USB_DEVCTL_HOSTREQ_B        0x00000002 // Host Request (OTG only)
#define USB_DEVCTL_SESSION_B        0x00000001 // Session Start/End (OTG only)

/* Bit/Fields in Register TXFIFOSZ of Module USB                              */
#define USB_TXFIFOSZ_DPB_B          0x00000010 // Double Packet Buffer Support
#define USB_TXFIFOSZ_SIZE_M         0x0000000F // Max Packet Size
#define USB_TXFIFOSZ_SIZE_S         0          // Max Packet Size
#define USB_TXFIFOSZ_SIZE_8_V       0x00000000 // 8
#define USB_TXFIFOSZ_SIZE_16_V      0x00000001 // 16
#define USB_TXFIFOSZ_SIZE_32_V      0x00000002 // 32
#define USB_TXFIFOSZ_SIZE_64_V      0x00000003 // 64
#define USB_TXFIFOSZ_SIZE_128_V     0x00000004 // 128
#define USB_TXFIFOSZ_SIZE_256_V     0x00000005 // 256
#define USB_TXFIFOSZ_SIZE_512_V     0x00000006 // 512
#define USB_TXFIFOSZ_SIZE_1024_V    0x00000007 // 1024
#define USB_TXFIFOSZ_SIZE_2048_V    0x00000008 // 2048

/* Bit/Fields in Register RXFIFOSZ of Module USB                              */
#define USB_RXFIFOSZ_DPB_B          0x00000010 // Double Packet Buffer Support
#define USB_RXFIFOSZ_SIZE_M         0x0000000F // Max Packet Size
#define USB_RXFIFOSZ_SIZE_S         0          // Max Packet Size
#define USB_RXFIFOSZ_SIZE_8_V       0x00000000 // 8
#define USB_RXFIFOSZ_SIZE_16_V      0x00000001 // 16
#define USB_RXFIFOSZ_SIZE_32_V      0x00000002 // 32
#define USB_RXFIFOSZ_SIZE_64_V      0x00000003 // 64
#define USB_RXFIFOSZ_SIZE_128_V     0x00000004 // 128
#define USB_RXFIFOSZ_SIZE_256_V     0x00000005 // 256
#define USB_RXFIFOSZ_SIZE_512_V     0x00000006 // 512
#define USB_RXFIFOSZ_SIZE_1024_V    0x00000007 // 1024
#define USB_RXFIFOSZ_SIZE_2048_V    0x00000008 // 2048

/* Bit/Fields in Register TXFIFOADD of Module USB                             */
#define USB_TXFIFOADD_ADDR_M        0x000001FF // Transmit/Receive Start Address
#define USB_TXFIFOADD_ADDR_S        0          // Transmit/Receive Start Address

/* Bit/Fields in Register RXFIFOADD of Module USB                             */
#define USB_RXFIFOADD_ADDR_M        0x000001FF // Transmit/Receive Start Address
#define USB_RXFIFOADD_ADDR_S        0          // Transmit/Receive Start Address

/* Bit/Fields in Register CONTIM of Module USB                                */
#define USB_CONTIM_WTCON_M          0x000000F0 // Connect Wait
#define USB_CONTIM_WTCON_S          4          // Connect Wait
#define USB_CONTIM_WTID_M           0x0000000F // Wait ID
#define USB_CONTIM_WTID_S           0          // Wait ID

/* Bit/Fields in Register VPLEN of Module USB                                 */
#define USB_VPLEN_VPLEN_M           0x000000FF // VBUS Pulse Length
#define USB_VPLEN_VPLEN_S           0          // VBUS Pulse Length

/* Bit/Fields in Register FSEOF of Module USB                                 */
#define USB_FSEOF_FSEOFG_M          0x000000FF // Full-Speed End-of-Frame Gap
#define USB_FSEOF_FSEOFG_S          0          // Full-Speed End-of-Frame Gap

/* Bit/Fields in Register LSEOF of Module USB                                 */
#define USB_LSEOF_LSEOFG_M          0x000000FF // Low-Speed End-of-Frame Gap
#define USB_LSEOF_LSEOFG_S          0          // Low-Speed End-of-Frame Gap

/* Bit/Fields in Register TXFUNCADDR0 of Module USB                           */
#define USB_TXFUNCADDR0_ADDR_M      0x0000007F // Device Address
#define USB_TXFUNCADDR0_ADDR_S      0          // Device Address

/* Bit/Fields in Register TXHUBADDR0 of Module USB                            */
#define USB_TXHUBADDR0_ADDR_M       0x0000007F // Hub Address
#define USB_TXHUBADDR0_ADDR_S       0          // Hub Address

/* Bit/Fields in Register TXHUBPORT0 of Module USB                            */
#define USB_TXHUBPORT0_PORT_M       0x0000007F // Hub Port
#define USB_TXHUBPORT0_PORT_S       0          // Hub Port

/* Bit/Fields in Register TXFUNCADDR1 of Module USB                           */
#define USB_TXFUNCADDR1_ADDR_M      0x0000007F // Device Address
#define USB_TXFUNCADDR1_ADDR_S      0          // Device Address

/* Bit/Fields in Register TXHUBADDR1 of Module USB                            */
#define USB_TXHUBADDR1_ADDR_M       0x0000007F // Hub Address
#define USB_TXHUBADDR1_ADDR_S       0          // Hub Address

/* Bit/Fields in Register TXHUBPORT1 of Module USB                            */
#define USB_TXHUBPORT1_PORT_M       0x0000007F // Hub Port
#define USB_TXHUBPORT1_PORT_S       0          // Hub Port

/* Bit/Fields in Register RXFUNCADDR1 of Module USB                           */
#define USB_RXFUNCADDR1_ADDR_M      0x0000007F // Device Address
#define USB_RXFUNCADDR1_ADDR_S      0          // Device Address

/* Bit/Fields in Register RXHUBADDR1 of Module USB                            */
#define USB_RXHUBADDR1_ADDR_M       0x0000007F // Hub Address
#define USB_RXHUBADDR1_ADDR_S       0          // Hub Address

/* Bit/Fields in Register RXHUBPORT1 of Module USB                            */
#define USB_RXHUBPORT1_PORT_M       0x0000007F // Hub Port
#define USB_RXHUBPORT1_PORT_S       0          // Hub Port

/* Bit/Fields in Register TXFUNCADDR2 of Module USB                           */
#define USB_TXFUNCADDR2_ADDR_M      0x0000007F // Device Address
#define USB_TXFUNCADDR2_ADDR_S      0          // Device Address

/* Bit/Fields in Register TXHUBADDR2 of Module USB                            */
#define USB_TXHUBADDR2_ADDR_M       0x0000007F // Hub Address
#define USB_TXHUBADDR2_ADDR_S       0          // Hub Address

/* Bit/Fields in Register TXHUBPORT2 of Module USB                            */
#define USB_TXHUBPORT2_PORT_M       0x0000007F // Hub Port
#define USB_TXHUBPORT2_PORT_S       0          // Hub Port

/* Bit/Fields in Register RXFUNCADDR2 of Module USB                           */
#define USB_RXFUNCADDR2_ADDR_M      0x0000007F // Device Address
#define USB_RXFUNCADDR2_ADDR_S      0          // Device Address

/* Bit/Fields in Register RXHUBADDR2 of Module USB                            */
#define USB_RXHUBADDR2_ADDR_M       0x0000007F // Hub Address
#define USB_RXHUBADDR2_ADDR_S       0          // Hub Address

/* Bit/Fields in Register RXHUBPORT2 of Module USB                            */
#define USB_RXHUBPORT2_PORT_M       0x0000007F // Hub Port
#define USB_RXHUBPORT2_PORT_S       0          // Hub Port

/* Bit/Fields in Register TXFUNCADDR3 of Module USB                           */
#define USB_TXFUNCADDR3_ADDR_M      0x0000007F // Device Address
#define USB_TXFUNCADDR3_ADDR_S      0          // Device Address

/* Bit/Fields in Register TXHUBADDR3 of Module USB                            */
#define USB_TXHUBADDR3_ADDR_M       0x0000007F // Hub Address
#define USB_TXHUBADDR3_ADDR_S       0          // Hub Address

/* Bit/Fields in Register TXHUBPORT3 of Module USB                            */
#define USB_TXHUBPORT3_PORT_M       0x0000007F // Hub Port
#define USB_TXHUBPORT3_PORT_S       0          // Hub Port

/* Bit/Fields in Register RXFUNCADDR3 of Module USB                           */
#define USB_RXFUNCADDR3_ADDR_M      0x0000007F // Device Address
#define USB_RXFUNCADDR3_ADDR_S      0          // Device Address

/* Bit/Fields in Register RXHUBADDR3 of Module USB                            */
#define USB_RXHUBADDR3_ADDR_M       0x0000007F // Hub Address
#define USB_RXHUBADDR3_ADDR_S       0          // Hub Address

/* Bit/Fields in Register RXHUBPORT3 of Module USB                            */
#define USB_RXHUBPORT3_PORT_M       0x0000007F // Hub Port
#define USB_RXHUBPORT3_PORT_S       0          // Hub Port

/* Bit/Fields in Register TXFUNCADDR4 of Module USB                           */
#define USB_TXFUNCADDR4_ADDR_M      0x0000007F // Device Address
#define USB_TXFUNCADDR4_ADDR_S      0          // Device Address

/* Bit/Fields in Register TXHUBADDR4 of Module USB                            */
#define USB_TXHUBADDR4_ADDR_M       0x0000007F // Hub Address
#define USB_TXHUBADDR4_ADDR_S       0          // Hub Address

/* Bit/Fields in Register TXHUBPORT4 of Module USB                            */
#define USB_TXHUBPORT4_PORT_M       0x0000007F // Hub Port
#define USB_TXHUBPORT4_PORT_S       0          // Hub Port

/* Bit/Fields in Register RXFUNCADDR4 of Module USB                           */
#define USB_RXFUNCADDR4_ADDR_M      0x0000007F // Device Address
#define USB_RXFUNCADDR4_ADDR_S      0          // Device Address

/* Bit/Fields in Register RXHUBADDR4 of Module USB                            */
#define USB_RXHUBADDR4_ADDR_M       0x0000007F // Hub Address
#define USB_RXHUBADDR4_ADDR_S       0          // Hub Address

/* Bit/Fields in Register RXHUBPORT4 of Module USB                            */
#define USB_RXHUBPORT4_PORT_M       0x0000007F // Hub Port
#define USB_RXHUBPORT4_PORT_S       0          // Hub Port

/* Bit/Fields in Register TXFUNCADDR5 of Module USB                           */
#define USB_TXFUNCADDR5_ADDR_M      0x0000007F // Device Address
#define USB_TXFUNCADDR5_ADDR_S      0          // Device Address

/* Bit/Fields in Register TXHUBADDR5 of Module USB                            */
#define USB_TXHUBADDR5_ADDR_M       0x0000007F // Hub Address
#define USB_TXHUBADDR5_ADDR_S       0          // Hub Address

/* Bit/Fields in Register TXHUBPORT5 of Module USB                            */
#define USB_TXHUBPORT5_PORT_M       0x0000007F // Hub Port
#define USB_TXHUBPORT5_PORT_S       0          // Hub Port

/* Bit/Fields in Register RXFUNCADDR5 of Module USB                           */
#define USB_RXFUNCADDR5_ADDR_M      0x0000007F // Device Address
#define USB_RXFUNCADDR5_ADDR_S      0          // Device Address

/* Bit/Fields in Register RXHUBADDR5 of Module USB                            */
#define USB_RXHUBADDR5_ADDR_M       0x0000007F // Hub Address
#define USB_RXHUBADDR5_ADDR_S       0          // Hub Address

/* Bit/Fields in Register RXHUBPORT5 of Module USB                            */
#define USB_RXHUBPORT5_PORT_M       0x0000007F // Hub Port
#define USB_RXHUBPORT5_PORT_S       0          // Hub Port

/* Bit/Fields in Register TXFUNCADDR6 of Module USB                           */
#define USB_TXFUNCADDR6_ADDR_M      0x0000007F // Device Address
#define USB_TXFUNCADDR6_ADDR_S      0          // Device Address

/* Bit/Fields in Register TXHUBADDR6 of Module USB                            */
#define USB_TXHUBADDR6_ADDR_M       0x0000007F // Hub Address
#define USB_TXHUBADDR6_ADDR_S       0          // Hub Address

/* Bit/Fields in Register TXHUBPORT6 of Module USB                            */
#define USB_TXHUBPORT6_PORT_M       0x0000007F // Hub Port
#define USB_TXHUBPORT6_PORT_S       0          // Hub Port

/* Bit/Fields in Register RXFUNCADDR6 of Module USB                           */
#define USB_RXFUNCADDR6_ADDR_M      0x0000007F // Device Address
#define USB_RXFUNCADDR6_ADDR_S      0          // Device Address

/* Bit/Fields in Register RXHUBADDR6 of Module USB                            */
#define USB_RXHUBADDR6_ADDR_M       0x0000007F // Hub Address
#define USB_RXHUBADDR6_ADDR_S       0          // Hub Address

/* Bit/Fields in Register RXHUBPORT6 of Module USB                            */
#define USB_RXHUBPORT6_PORT_M       0x0000007F // Hub Port
#define USB_RXHUBPORT6_PORT_S       0          // Hub Port

/* Bit/Fields in Register TXFUNCADDR7 of Module USB                           */
#define USB_TXFUNCADDR7_ADDR_M      0x0000007F // Device Address
#define USB_TXFUNCADDR7_ADDR_S      0          // Device Address

/* Bit/Fields in Register TXHUBADDR7 of Module USB                            */
#define USB_TXHUBADDR7_ADDR_M       0x0000007F // Hub Address
#define USB_TXHUBADDR7_ADDR_S       0          // Hub Address

/* Bit/Fields in Register TXHUBPORT7 of Module USB                            */
#define USB_TXHUBPORT7_PORT_M       0x0000007F // Hub Port
#define USB_TXHUBPORT7_PORT_S       0          // Hub Port

/* Bit/Fields in Register RXFUNCADDR7 of Module USB                           */
#define USB_RXFUNCADDR7_ADDR_M      0x0000007F // Device Address
#define USB_RXFUNCADDR7_ADDR_S      0          // Device Address

/* Bit/Fields in Register RXHUBADDR7 of Module USB                            */
#define USB_RXHUBADDR7_ADDR_M       0x0000007F // Hub Address
#define USB_RXHUBADDR7_ADDR_S       0          // Hub Address

/* Bit/Fields in Register RXHUBPORT7 of Module USB                            */
#define USB_RXHUBPORT7_PORT_M       0x0000007F // Hub Port
#define USB_RXHUBPORT7_PORT_S       0          // Hub Port

/* Bit/Fields in Register CSRL0 of Module USB                                 */
#define USB_CSRL0_NAKTO_B           0x00000080 // NAK Timeout
#define USB_CSRL0_SETENDC_B         0x00000080 // Setup End Clear
#define USB_CSRL0_STATUS_B          0x00000040 // STATUS Packet
#define USB_CSRL0_RXRDYC_B          0x00000040 // RXRDY Clear
#define USB_CSRL0_REQPKT_B          0x00000020 // Request Packet
#define USB_CSRL0_STALL_B           0x00000020 // Send Stall
#define USB_CSRL0_SETEND_B          0x00000010 // Setup End
#define USB_CSRL0_ERROR_B           0x00000010 // Error
#define USB_CSRL0_DATAEND_B         0x00000008 // Data End
#define USB_CSRL0_SETUP_B           0x00000008 // Setup Packet
#define USB_CSRL0_STALLED_B         0x00000004 // Endpoint Stalled
#define USB_CSRL0_TXRDY_B           0x00000002 // Transmit Packet Ready
#define USB_CSRL0_RXRDY_B           0x00000001 // Receive Packet Ready

/* Bit/Fields in Register CSRH0 of Module USB                                 */
#define USB_CSRH0_DTWE_B            0x00000004 // Data Toggle Write Enable
#define USB_CSRH0_DT_B              0x00000002 // Data Toggle
#define USB_CSRH0_FLUSH_B           0x00000001 // Flush FIFO

/* Bit/Fields in Register COUNT0 of Module USB                                */
#define USB_COUNT0_COUNT_M          0x0000007F // FIFO Count
#define USB_COUNT0_COUNT_S          0          // FIFO Count

/* Bit/Fields in Register TYPE0 of Module USB                                 */
#define USB_TYPE0_SPEED_M           0x000000C0 // Operating Speed
#define USB_TYPE0_SPEED_S           6          // Operating Speed
#define USB_TYPE0_SPEED_FULL_V      0x00000080 // Full
#define USB_TYPE0_SPEED_LOW_V       0x000000C0 // Low

/* Bit/Fields in Register NAKLMT of Module USB                                */
#define USB_NAKLMT_NAKLMT_M         0x0000001F // EP0 NAK Limit
#define USB_NAKLMT_NAKLMT_S         0          // EP0 NAK Limit

/* Bit/Fields in Register TXMAXP1 of Module USB                               */
#define USB_TXMAXP1_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_TXMAXP1_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register TXCSRL1 of Module USB                               */
#define USB_TXCSRL1_NAKTO_B         0x00000080 // NAK Timeout
#define USB_TXCSRL1_CLRDT_B         0x00000040 // Clear Data Toggle
#define USB_TXCSRL1_STALLED_B       0x00000020 // Endpoint Stalled
#define USB_TXCSRL1_STALL_B         0x00000010 // Send STALL
#define USB_TXCSRL1_SETUP_B         0x00000010 // Setup Packet
#define USB_TXCSRL1_FLUSH_B         0x00000008 // Flush FIFO
#define USB_TXCSRL1_ERROR_B         0x00000004 // Error
#define USB_TXCSRL1_UNDRN_B         0x00000004 // Underrun
#define USB_TXCSRL1_FIFONE_B        0x00000002 // FIFO Not Empty
#define USB_TXCSRL1_TXRDY_B         0x00000001 // Transmit Packet Ready

/* Bit/Fields in Register TXCSRH1 of Module USB                               */
#define USB_TXCSRH1_AUTOSET_B       0x00000080 // Auto Set
#define USB_TXCSRH1_ISO_B           0x00000040 // Isochronous Transfers
#define USB_TXCSRH1_MODE_B          0x00000020 // Mode
#define USB_TXCSRH1_DMAEN_B         0x00000010 // DMA Request Enable
#define USB_TXCSRH1_FDT_B           0x00000008 // Force Data Toggle
#define USB_TXCSRH1_DMAMOD_B        0x00000004 // DMA Request Mode
#define USB_TXCSRH1_DTWE_B          0x00000002 // Data Toggle Write Enable
#define USB_TXCSRH1_DT_B            0x00000001 // Data Toggle

/* Bit/Fields in Register RXMAXP1 of Module USB                               */
#define USB_RXMAXP1_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_RXMAXP1_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register RXCSRL1 of Module USB                               */
#define USB_RXCSRL1_CLRDT_B         0x00000080 // Clear Data Toggle
#define USB_RXCSRL1_STALLED_B       0x00000040 // Endpoint Stalled
#define USB_RXCSRL1_STALL_B         0x00000020 // Send STALL
#define USB_RXCSRL1_REQPKT_B        0x00000020 // Request Packet
#define USB_RXCSRL1_FLUSH_B         0x00000010 // Flush FIFO
#define USB_RXCSRL1_DATAERR_B       0x00000008 // Data Error
#define USB_RXCSRL1_NAKTO_B         0x00000008 // NAK Timeout
#define USB_RXCSRL1_OVER_B          0x00000004 // Overrun
#define USB_RXCSRL1_ERROR_B         0x00000004 // Error
#define USB_RXCSRL1_FULL_B          0x00000002 // FIFO Full
#define USB_RXCSRL1_RXRDY_B         0x00000001 // Receive Packet Ready

/* Bit/Fields in Register RXCSRH1 of Module USB                               */
#define USB_RXCSRH1_AUTOCL_B        0x00000080 // Auto Clear
#define USB_RXCSRH1_AUTORQ_B        0x00000040 // Auto Request
#define USB_RXCSRH1_ISO_B           0x00000040 // Isochronous Transfers
#define USB_RXCSRH1_DMAEN_B         0x00000020 // DMA Request Enable
#define USB_RXCSRH1_DISNYET_B       0x00000010 // Disable NYET
#define USB_RXCSRH1_PIDERR_B        0x00000010 // PID Error
#define USB_RXCSRH1_DMAMOD_B        0x00000008 // DMA Request Mode
#define USB_RXCSRH1_DTWE_B          0x00000004 // Data Toggle Write Enable
#define USB_RXCSRH1_DT_B            0x00000002 // Data Toggle

/* Bit/Fields in Register RXCOUNT1 of Module USB                              */
#define USB_RXCOUNT1_COUNT_M        0x00001FFF // Receive Packet Count
#define USB_RXCOUNT1_COUNT_S        0          // Receive Packet Count

/* Bit/Fields in Register TXTYPE1 of Module USB                               */
#define USB_TXTYPE1_SPEED_M         0x000000C0 // Operating Speed
#define USB_TXTYPE1_SPEED_S         6          // Operating Speed
#define USB_TXTYPE1_SPEED_DFLT_V    0x00000000 // Default
#define USB_TXTYPE1_SPEED_FULL_V    0x00000080 // Full
#define USB_TXTYPE1_SPEED_LOW_V     0x000000C0 // Low
#define USB_TXTYPE1_PROTO_M         0x00000030 // Protocol
#define USB_TXTYPE1_PROTO_S         4          // Protocol
#define USB_TXTYPE1_PROTO_CTRL_V    0x00000000 // Control
#define USB_TXTYPE1_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_TXTYPE1_PROTO_BULK_V    0x00000020 // Bulk
#define USB_TXTYPE1_PROTO_INT_V     0x00000030 // Interrupt
#define USB_TXTYPE1_TEP_M           0x0000000F // Target Endpoint Number
#define USB_TXTYPE1_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register RXTYPE1 of Module USB                               */
#define USB_RXTYPE1_SPEED_M         0x000000C0 // Operating Speed
#define USB_RXTYPE1_SPEED_S         6          // Operating Speed
#define USB_RXTYPE1_SPEED_DFLT_V    0x00000000 // Default
#define USB_RXTYPE1_SPEED_FULL_V    0x00000080 // Full
#define USB_RXTYPE1_SPEED_LOW_V     0x000000C0 // Low
#define USB_RXTYPE1_PROTO_M         0x00000030 // Protocol
#define USB_RXTYPE1_PROTO_S         4          // Protocol
#define USB_RXTYPE1_PROTO_CTRL_V    0x00000000 // Control
#define USB_RXTYPE1_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_RXTYPE1_PROTO_BULK_V    0x00000020 // Bulk
#define USB_RXTYPE1_PROTO_INT_V     0x00000030 // Interrupt
#define USB_RXTYPE1_TEP_M           0x0000000F // Target Endpoint Number
#define USB_RXTYPE1_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register TXMAXP2 of Module USB                               */
#define USB_TXMAXP2_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_TXMAXP2_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register TXCSRL2 of Module USB                               */
#define USB_TXCSRL2_NAKTO_B         0x00000080 // NAK Timeout
#define USB_TXCSRL2_CLRDT_B         0x00000040 // Clear Data Toggle
#define USB_TXCSRL2_STALLED_B       0x00000020 // Endpoint Stalled
#define USB_TXCSRL2_SETUP_B         0x00000010 // Setup Packet
#define USB_TXCSRL2_STALL_B         0x00000010 // Send STALL
#define USB_TXCSRL2_FLUSH_B         0x00000008 // Flush FIFO
#define USB_TXCSRL2_ERROR_B         0x00000004 // Error
#define USB_TXCSRL2_UNDRN_B         0x00000004 // Underrun
#define USB_TXCSRL2_FIFONE_B        0x00000002 // FIFO Not Empty
#define USB_TXCSRL2_TXRDY_B         0x00000001 // Transmit Packet Ready

/* Bit/Fields in Register TXCSRH2 of Module USB                               */
#define USB_TXCSRH2_AUTOSET_B       0x00000080 // Auto Set
#define USB_TXCSRH2_ISO_B           0x00000040 // Isochronous Transfers
#define USB_TXCSRH2_MODE_B          0x00000020 // Mode
#define USB_TXCSRH2_DMAEN_B         0x00000010 // DMA Request Enable
#define USB_TXCSRH2_FDT_B           0x00000008 // Force Data Toggle
#define USB_TXCSRH2_DMAMOD_B        0x00000004 // DMA Request Mode
#define USB_TXCSRH2_DTWE_B          0x00000002 // Data Toggle Write Enable
#define USB_TXCSRH2_DT_B            0x00000001 // Data Toggle

/* Bit/Fields in Register RXMAXP2 of Module USB                               */
#define USB_RXMAXP2_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_RXMAXP2_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register RXCSRL2 of Module USB                               */
#define USB_RXCSRL2_CLRDT_B         0x00000080 // Clear Data Toggle
#define USB_RXCSRL2_STALLED_B       0x00000040 // Endpoint Stalled
#define USB_RXCSRL2_REQPKT_B        0x00000020 // Request Packet
#define USB_RXCSRL2_STALL_B         0x00000020 // Send STALL
#define USB_RXCSRL2_FLUSH_B         0x00000010 // Flush FIFO
#define USB_RXCSRL2_DATAERR_B       0x00000008 // Data Error
#define USB_RXCSRL2_NAKTO_B         0x00000008 // NAK Timeout
#define USB_RXCSRL2_ERROR_B         0x00000004 // Error
#define USB_RXCSRL2_OVER_B          0x00000004 // Overrun
#define USB_RXCSRL2_FULL_B          0x00000002 // FIFO Full
#define USB_RXCSRL2_RXRDY_B         0x00000001 // Receive Packet Ready

/* Bit/Fields in Register RXCSRH2 of Module USB                               */
#define USB_RXCSRH2_AUTOCL_B        0x00000080 // Auto Clear
#define USB_RXCSRH2_AUTORQ_B        0x00000040 // Auto Request
#define USB_RXCSRH2_ISO_B           0x00000040 // Isochronous Transfers
#define USB_RXCSRH2_DMAEN_B         0x00000020 // DMA Request Enable
#define USB_RXCSRH2_DISNYET_B       0x00000010 // Disable NYET
#define USB_RXCSRH2_PIDERR_B        0x00000010 // PID Error
#define USB_RXCSRH2_DMAMOD_B        0x00000008 // DMA Request Mode
#define USB_RXCSRH2_DTWE_B          0x00000004 // Data Toggle Write Enable
#define USB_RXCSRH2_DT_B            0x00000002 // Data Toggle

/* Bit/Fields in Register RXCOUNT2 of Module USB                              */
#define USB_RXCOUNT2_COUNT_M        0x00001FFF // Receive Packet Count
#define USB_RXCOUNT2_COUNT_S        0          // Receive Packet Count

/* Bit/Fields in Register TXTYPE2 of Module USB                               */
#define USB_TXTYPE2_SPEED_M         0x000000C0 // Operating Speed
#define USB_TXTYPE2_SPEED_S         6          // Operating Speed
#define USB_TXTYPE2_SPEED_DFLT_V    0x00000000 // Default
#define USB_TXTYPE2_SPEED_FULL_V    0x00000080 // Full
#define USB_TXTYPE2_SPEED_LOW_V     0x000000C0 // Low
#define USB_TXTYPE2_PROTO_M         0x00000030 // Protocol
#define USB_TXTYPE2_PROTO_S         4          // Protocol
#define USB_TXTYPE2_PROTO_CTRL_V    0x00000000 // Control
#define USB_TXTYPE2_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_TXTYPE2_PROTO_BULK_V    0x00000020 // Bulk
#define USB_TXTYPE2_PROTO_INT_V     0x00000030 // Interrupt
#define USB_TXTYPE2_TEP_M           0x0000000F // Target Endpoint Number
#define USB_TXTYPE2_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register RXTYPE2 of Module USB                               */
#define USB_RXTYPE2_SPEED_M         0x000000C0 // Operating Speed
#define USB_RXTYPE2_SPEED_S         6          // Operating Speed
#define USB_RXTYPE2_SPEED_DFLT_V    0x00000000 // Default
#define USB_RXTYPE2_SPEED_FULL_V    0x00000080 // Full
#define USB_RXTYPE2_SPEED_LOW_V     0x000000C0 // Low
#define USB_RXTYPE2_PROTO_M         0x00000030 // Protocol
#define USB_RXTYPE2_PROTO_S         4          // Protocol
#define USB_RXTYPE2_PROTO_CTRL_V    0x00000000 // Control
#define USB_RXTYPE2_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_RXTYPE2_PROTO_BULK_V    0x00000020 // Bulk
#define USB_RXTYPE2_PROTO_INT_V     0x00000030 // Interrupt
#define USB_RXTYPE2_TEP_M           0x0000000F // Target Endpoint Number
#define USB_RXTYPE2_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register TXMAXP3 of Module USB                               */
#define USB_TXMAXP3_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_TXMAXP3_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register TXCSRL3 of Module USB                               */
#define USB_TXCSRL3_NAKTO_B         0x00000080 // NAK Timeout
#define USB_TXCSRL3_CLRDT_B         0x00000040 // Clear Data Toggle
#define USB_TXCSRL3_STALLED_B       0x00000020 // Endpoint Stalled
#define USB_TXCSRL3_SETUP_B         0x00000010 // Setup Packet
#define USB_TXCSRL3_STALL_B         0x00000010 // Send STALL
#define USB_TXCSRL3_FLUSH_B         0x00000008 // Flush FIFO
#define USB_TXCSRL3_ERROR_B         0x00000004 // Error
#define USB_TXCSRL3_UNDRN_B         0x00000004 // Underrun
#define USB_TXCSRL3_FIFONE_B        0x00000002 // FIFO Not Empty
#define USB_TXCSRL3_TXRDY_B         0x00000001 // Transmit Packet Ready

/* Bit/Fields in Register TXCSRH3 of Module USB                               */
#define USB_TXCSRH3_AUTOSET_B       0x00000080 // Auto Set
#define USB_TXCSRH3_ISO_B           0x00000040 // Isochronous Transfers
#define USB_TXCSRH3_MODE_B          0x00000020 // Mode
#define USB_TXCSRH3_DMAEN_B         0x00000010 // DMA Request Enable
#define USB_TXCSRH3_FDT_B           0x00000008 // Force Data Toggle
#define USB_TXCSRH3_DMAMOD_B        0x00000004 // DMA Request Mode
#define USB_TXCSRH3_DTWE_B          0x00000002 // Data Toggle Write Enable
#define USB_TXCSRH3_DT_B            0x00000001 // Data Toggle

/* Bit/Fields in Register RXMAXP3 of Module USB                               */
#define USB_RXMAXP3_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_RXMAXP3_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register RXCSRL3 of Module USB                               */
#define USB_RXCSRL3_CLRDT_B         0x00000080 // Clear Data Toggle
#define USB_RXCSRL3_STALLED_B       0x00000040 // Endpoint Stalled
#define USB_RXCSRL3_STALL_B         0x00000020 // Send STALL
#define USB_RXCSRL3_REQPKT_B        0x00000020 // Request Packet
#define USB_RXCSRL3_FLUSH_B         0x00000010 // Flush FIFO
#define USB_RXCSRL3_DATAERR_B       0x00000008 // Data Error
#define USB_RXCSRL3_NAKTO_B         0x00000008 // NAK Timeout
#define USB_RXCSRL3_ERROR_B         0x00000004 // Error
#define USB_RXCSRL3_OVER_B          0x00000004 // Overrun
#define USB_RXCSRL3_FULL_B          0x00000002 // FIFO Full
#define USB_RXCSRL3_RXRDY_B         0x00000001 // Receive Packet Ready

/* Bit/Fields in Register RXCSRH3 of Module USB                               */
#define USB_RXCSRH3_AUTOCL_B        0x00000080 // Auto Clear
#define USB_RXCSRH3_AUTORQ_B        0x00000040 // Auto Request
#define USB_RXCSRH3_ISO_B           0x00000040 // Isochronous Transfers
#define USB_RXCSRH3_DMAEN_B         0x00000020 // DMA Request Enable
#define USB_RXCSRH3_DISNYET_B       0x00000010 // Disable NYET
#define USB_RXCSRH3_PIDERR_B        0x00000010 // PID Error
#define USB_RXCSRH3_DMAMOD_B        0x00000008 // DMA Request Mode
#define USB_RXCSRH3_DTWE_B          0x00000004 // Data Toggle Write Enable
#define USB_RXCSRH3_DT_B            0x00000002 // Data Toggle

/* Bit/Fields in Register RXCOUNT3 of Module USB                              */
#define USB_RXCOUNT3_COUNT_M        0x00001FFF // Receive Packet Count
#define USB_RXCOUNT3_COUNT_S        0          // Receive Packet Count

/* Bit/Fields in Register TXTYPE3 of Module USB                               */
#define USB_TXTYPE3_SPEED_M         0x000000C0 // Operating Speed
#define USB_TXTYPE3_SPEED_S         6          // Operating Speed
#define USB_TXTYPE3_SPEED_DFLT_V    0x00000000 // Default
#define USB_TXTYPE3_SPEED_FULL_V    0x00000080 // Full
#define USB_TXTYPE3_SPEED_LOW_V     0x000000C0 // Low
#define USB_TXTYPE3_PROTO_M         0x00000030 // Protocol
#define USB_TXTYPE3_PROTO_S         4          // Protocol
#define USB_TXTYPE3_PROTO_CTRL_V    0x00000000 // Control
#define USB_TXTYPE3_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_TXTYPE3_PROTO_BULK_V    0x00000020 // Bulk
#define USB_TXTYPE3_PROTO_INT_V     0x00000030 // Interrupt
#define USB_TXTYPE3_TEP_M           0x0000000F // Target Endpoint Number
#define USB_TXTYPE3_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register RXTYPE3 of Module USB                               */
#define USB_RXTYPE3_SPEED_M         0x000000C0 // Operating Speed
#define USB_RXTYPE3_SPEED_S         6          // Operating Speed
#define USB_RXTYPE3_SPEED_DFLT_V    0x00000000 // Default
#define USB_RXTYPE3_SPEED_FULL_V    0x00000080 // Full
#define USB_RXTYPE3_SPEED_LOW_V     0x000000C0 // Low
#define USB_RXTYPE3_PROTO_M         0x00000030 // Protocol
#define USB_RXTYPE3_PROTO_S         4          // Protocol
#define USB_RXTYPE3_PROTO_CTRL_V    0x00000000 // Control
#define USB_RXTYPE3_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_RXTYPE3_PROTO_BULK_V    0x00000020 // Bulk
#define USB_RXTYPE3_PROTO_INT_V     0x00000030 // Interrupt
#define USB_RXTYPE3_TEP_M           0x0000000F // Target Endpoint Number
#define USB_RXTYPE3_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register TXMAXP4 of Module USB                               */
#define USB_TXMAXP4_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_TXMAXP4_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register TXCSRL4 of Module USB                               */
#define USB_TXCSRL4_NAKTO_B         0x00000080 // NAK Timeout
#define USB_TXCSRL4_CLRDT_B         0x00000040 // Clear Data Toggle
#define USB_TXCSRL4_STALLED_B       0x00000020 // Endpoint Stalled
#define USB_TXCSRL4_SETUP_B         0x00000010 // Setup Packet
#define USB_TXCSRL4_STALL_B         0x00000010 // Send STALL
#define USB_TXCSRL4_FLUSH_B         0x00000008 // Flush FIFO
#define USB_TXCSRL4_ERROR_B         0x00000004 // Error
#define USB_TXCSRL4_UNDRN_B         0x00000004 // Underrun
#define USB_TXCSRL4_FIFONE_B        0x00000002 // FIFO Not Empty
#define USB_TXCSRL4_TXRDY_B         0x00000001 // Transmit Packet Ready

/* Bit/Fields in Register TXCSRH4 of Module USB                               */
#define USB_TXCSRH4_AUTOSET_B       0x00000080 // Auto Set
#define USB_TXCSRH4_ISO_B           0x00000040 // Isochronous Transfers
#define USB_TXCSRH4_MODE_B          0x00000020 // Mode
#define USB_TXCSRH4_DMAEN_B         0x00000010 // DMA Request Enable
#define USB_TXCSRH4_FDT_B           0x00000008 // Force Data Toggle
#define USB_TXCSRH4_DMAMOD_B        0x00000004 // DMA Request Mode
#define USB_TXCSRH4_DTWE_B          0x00000002 // Data Toggle Write Enable
#define USB_TXCSRH4_DT_B            0x00000001 // Data Toggle

/* Bit/Fields in Register RXMAXP4 of Module USB                               */
#define USB_RXMAXP4_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_RXMAXP4_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register RXCSRL4 of Module USB                               */
#define USB_RXCSRL4_CLRDT_B         0x00000080 // Clear Data Toggle
#define USB_RXCSRL4_STALLED_B       0x00000040 // Endpoint Stalled
#define USB_RXCSRL4_STALL_B         0x00000020 // Send STALL
#define USB_RXCSRL4_REQPKT_B        0x00000020 // Request Packet
#define USB_RXCSRL4_FLUSH_B         0x00000010 // Flush FIFO
#define USB_RXCSRL4_NAKTO_B         0x00000008 // NAK Timeout
#define USB_RXCSRL4_DATAERR_B       0x00000008 // Data Error
#define USB_RXCSRL4_OVER_B          0x00000004 // Overrun
#define USB_RXCSRL4_ERROR_B         0x00000004 // Error
#define USB_RXCSRL4_FULL_B          0x00000002 // FIFO Full
#define USB_RXCSRL4_RXRDY_B         0x00000001 // Receive Packet Ready

/* Bit/Fields in Register RXCSRH4 of Module USB                               */
#define USB_RXCSRH4_AUTOCL_B        0x00000080 // Auto Clear
#define USB_RXCSRH4_AUTORQ_B        0x00000040 // Auto Request
#define USB_RXCSRH4_ISO_B           0x00000040 // Isochronous Transfers
#define USB_RXCSRH4_DMAEN_B         0x00000020 // DMA Request Enable
#define USB_RXCSRH4_DISNYET_B       0x00000010 // Disable NYET
#define USB_RXCSRH4_PIDERR_B        0x00000010 // PID Error
#define USB_RXCSRH4_DMAMOD_B        0x00000008 // DMA Request Mode
#define USB_RXCSRH4_DTWE_B          0x00000004 // Data Toggle Write Enable
#define USB_RXCSRH4_DT_B            0x00000002 // Data Toggle

/* Bit/Fields in Register RXCOUNT4 of Module USB                              */
#define USB_RXCOUNT4_COUNT_M        0x00001FFF // Receive Packet Count
#define USB_RXCOUNT4_COUNT_S        0          // Receive Packet Count

/* Bit/Fields in Register TXTYPE4 of Module USB                               */
#define USB_TXTYPE4_SPEED_M         0x000000C0 // Operating Speed
#define USB_TXTYPE4_SPEED_S         6          // Operating Speed
#define USB_TXTYPE4_SPEED_DFLT_V    0x00000000 // Default
#define USB_TXTYPE4_SPEED_FULL_V    0x00000080 // Full
#define USB_TXTYPE4_SPEED_LOW_V     0x000000C0 // Low
#define USB_TXTYPE4_PROTO_M         0x00000030 // Protocol
#define USB_TXTYPE4_PROTO_S         4          // Protocol
#define USB_TXTYPE4_PROTO_CTRL_V    0x00000000 // Control
#define USB_TXTYPE4_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_TXTYPE4_PROTO_BULK_V    0x00000020 // Bulk
#define USB_TXTYPE4_PROTO_INT_V     0x00000030 // Interrupt
#define USB_TXTYPE4_TEP_M           0x0000000F // Target Endpoint Number
#define USB_TXTYPE4_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register RXTYPE4 of Module USB                               */
#define USB_RXTYPE4_SPEED_M         0x000000C0 // Operating Speed
#define USB_RXTYPE4_SPEED_S         6          // Operating Speed
#define USB_RXTYPE4_SPEED_DFLT_V    0x00000000 // Default
#define USB_RXTYPE4_SPEED_FULL_V    0x00000080 // Full
#define USB_RXTYPE4_SPEED_LOW_V     0x000000C0 // Low
#define USB_RXTYPE4_PROTO_M         0x00000030 // Protocol
#define USB_RXTYPE4_PROTO_S         4          // Protocol
#define USB_RXTYPE4_PROTO_CTRL_V    0x00000000 // Control
#define USB_RXTYPE4_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_RXTYPE4_PROTO_BULK_V    0x00000020 // Bulk
#define USB_RXTYPE4_PROTO_INT_V     0x00000030 // Interrupt
#define USB_RXTYPE4_TEP_M           0x0000000F // Target Endpoint Number
#define USB_RXTYPE4_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register TXMAXP5 of Module USB                               */
#define USB_TXMAXP5_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_TXMAXP5_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register TXCSRL5 of Module USB                               */
#define USB_TXCSRL5_NAKTO_B         0x00000080 // NAK Timeout
#define USB_TXCSRL5_CLRDT_B         0x00000040 // Clear Data Toggle
#define USB_TXCSRL5_STALLED_B       0x00000020 // Endpoint Stalled
#define USB_TXCSRL5_SETUP_B         0x00000010 // Setup Packet
#define USB_TXCSRL5_STALL_B         0x00000010 // Send STALL
#define USB_TXCSRL5_FLUSH_B         0x00000008 // Flush FIFO
#define USB_TXCSRL5_ERROR_B         0x00000004 // Error
#define USB_TXCSRL5_UNDRN_B         0x00000004 // Underrun
#define USB_TXCSRL5_FIFONE_B        0x00000002 // FIFO Not Empty
#define USB_TXCSRL5_TXRDY_B         0x00000001 // Transmit Packet Ready

/* Bit/Fields in Register TXCSRH5 of Module USB                               */
#define USB_TXCSRH5_AUTOSET_B       0x00000080 // Auto Set
#define USB_TXCSRH5_ISO_B           0x00000040 // Isochronous Transfers
#define USB_TXCSRH5_MODE_B          0x00000020 // Mode
#define USB_TXCSRH5_DMAEN_B         0x00000010 // DMA Request Enable
#define USB_TXCSRH5_FDT_B           0x00000008 // Force Data Toggle
#define USB_TXCSRH5_DMAMOD_B        0x00000004 // DMA Request Mode
#define USB_TXCSRH5_DTWE_B          0x00000002 // Data Toggle Write Enable
#define USB_TXCSRH5_DT_B            0x00000001 // Data Toggle

/* Bit/Fields in Register RXMAXP5 of Module USB                               */
#define USB_RXMAXP5_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_RXMAXP5_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register RXCSRL5 of Module USB                               */
#define USB_RXCSRL5_CLRDT_B         0x00000080 // Clear Data Toggle
#define USB_RXCSRL5_STALLED_B       0x00000040 // Endpoint Stalled
#define USB_RXCSRL5_STALL_B         0x00000020 // Send STALL
#define USB_RXCSRL5_REQPKT_B        0x00000020 // Request Packet
#define USB_RXCSRL5_FLUSH_B         0x00000010 // Flush FIFO
#define USB_RXCSRL5_NAKTO_B         0x00000008 // NAK Timeout
#define USB_RXCSRL5_DATAERR_B       0x00000008 // Data Error
#define USB_RXCSRL5_ERROR_B         0x00000004 // Error
#define USB_RXCSRL5_OVER_B          0x00000004 // Overrun
#define USB_RXCSRL5_FULL_B          0x00000002 // FIFO Full
#define USB_RXCSRL5_RXRDY_B         0x00000001 // Receive Packet Ready

/* Bit/Fields in Register RXCSRH5 of Module USB                               */
#define USB_RXCSRH5_AUTOCL_B        0x00000080 // Auto Clear
#define USB_RXCSRH5_AUTORQ_B        0x00000040 // Auto Request
#define USB_RXCSRH5_ISO_B           0x00000040 // Isochronous Transfers
#define USB_RXCSRH5_DMAEN_B         0x00000020 // DMA Request Enable
#define USB_RXCSRH5_DISNYET_B       0x00000010 // Disable NYET
#define USB_RXCSRH5_PIDERR_B        0x00000010 // PID Error
#define USB_RXCSRH5_DMAMOD_B        0x00000008 // DMA Request Mode
#define USB_RXCSRH5_DTWE_B          0x00000004 // Data Toggle Write Enable
#define USB_RXCSRH5_DT_B            0x00000002 // Data Toggle

/* Bit/Fields in Register RXCOUNT5 of Module USB                              */
#define USB_RXCOUNT5_COUNT_M        0x00001FFF // Receive Packet Count
#define USB_RXCOUNT5_COUNT_S        0          // Receive Packet Count

/* Bit/Fields in Register TXTYPE5 of Module USB                               */
#define USB_TXTYPE5_SPEED_M         0x000000C0 // Operating Speed
#define USB_TXTYPE5_SPEED_S         6          // Operating Speed
#define USB_TXTYPE5_SPEED_DFLT_V    0x00000000 // Default
#define USB_TXTYPE5_SPEED_FULL_V    0x00000080 // Full
#define USB_TXTYPE5_SPEED_LOW_V     0x000000C0 // Low
#define USB_TXTYPE5_PROTO_M         0x00000030 // Protocol
#define USB_TXTYPE5_PROTO_S         4          // Protocol
#define USB_TXTYPE5_PROTO_CTRL_V    0x00000000 // Control
#define USB_TXTYPE5_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_TXTYPE5_PROTO_BULK_V    0x00000020 // Bulk
#define USB_TXTYPE5_PROTO_INT_V     0x00000030 // Interrupt
#define USB_TXTYPE5_TEP_M           0x0000000F // Target Endpoint Number
#define USB_TXTYPE5_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register RXTYPE5 of Module USB                               */
#define USB_RXTYPE5_SPEED_M         0x000000C0 // Operating Speed
#define USB_RXTYPE5_SPEED_S         6          // Operating Speed
#define USB_RXTYPE5_SPEED_DFLT_V    0x00000000 // Default
#define USB_RXTYPE5_SPEED_FULL_V    0x00000080 // Full
#define USB_RXTYPE5_SPEED_LOW_V     0x000000C0 // Low
#define USB_RXTYPE5_PROTO_M         0x00000030 // Protocol
#define USB_RXTYPE5_PROTO_S         4          // Protocol
#define USB_RXTYPE5_PROTO_CTRL_V    0x00000000 // Control
#define USB_RXTYPE5_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_RXTYPE5_PROTO_BULK_V    0x00000020 // Bulk
#define USB_RXTYPE5_PROTO_INT_V     0x00000030 // Interrupt
#define USB_RXTYPE5_TEP_M           0x0000000F // Target Endpoint Number
#define USB_RXTYPE5_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register TXMAXP6 of Module USB                               */
#define USB_TXMAXP6_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_TXMAXP6_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register TXCSRL6 of Module USB                               */
#define USB_TXCSRL6_NAKTO_B         0x00000080 // NAK Timeout
#define USB_TXCSRL6_CLRDT_B         0x00000040 // Clear Data Toggle
#define USB_TXCSRL6_STALLED_B       0x00000020 // Endpoint Stalled
#define USB_TXCSRL6_STALL_B         0x00000010 // Send STALL
#define USB_TXCSRL6_SETUP_B         0x00000010 // Setup Packet
#define USB_TXCSRL6_FLUSH_B         0x00000008 // Flush FIFO
#define USB_TXCSRL6_ERROR_B         0x00000004 // Error
#define USB_TXCSRL6_UNDRN_B         0x00000004 // Underrun
#define USB_TXCSRL6_FIFONE_B        0x00000002 // FIFO Not Empty
#define USB_TXCSRL6_TXRDY_B         0x00000001 // Transmit Packet Ready

/* Bit/Fields in Register TXCSRH6 of Module USB                               */
#define USB_TXCSRH6_AUTOSET_B       0x00000080 // Auto Set
#define USB_TXCSRH6_ISO_B           0x00000040 // Isochronous Transfers
#define USB_TXCSRH6_MODE_B          0x00000020 // Mode
#define USB_TXCSRH6_DMAEN_B         0x00000010 // DMA Request Enable
#define USB_TXCSRH6_FDT_B           0x00000008 // Force Data Toggle
#define USB_TXCSRH6_DMAMOD_B        0x00000004 // DMA Request Mode
#define USB_TXCSRH6_DTWE_B          0x00000002 // Data Toggle Write Enable
#define USB_TXCSRH6_DT_B            0x00000001 // Data Toggle

/* Bit/Fields in Register RXMAXP6 of Module USB                               */
#define USB_RXMAXP6_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_RXMAXP6_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register RXCSRL6 of Module USB                               */
#define USB_RXCSRL6_CLRDT_B         0x00000080 // Clear Data Toggle
#define USB_RXCSRL6_STALLED_B       0x00000040 // Endpoint Stalled
#define USB_RXCSRL6_REQPKT_B        0x00000020 // Request Packet
#define USB_RXCSRL6_STALL_B         0x00000020 // Send STALL
#define USB_RXCSRL6_FLUSH_B         0x00000010 // Flush FIFO
#define USB_RXCSRL6_NAKTO_B         0x00000008 // NAK Timeout
#define USB_RXCSRL6_DATAERR_B       0x00000008 // Data Error
#define USB_RXCSRL6_ERROR_B         0x00000004 // Error
#define USB_RXCSRL6_OVER_B          0x00000004 // Overrun
#define USB_RXCSRL6_FULL_B          0x00000002 // FIFO Full
#define USB_RXCSRL6_RXRDY_B         0x00000001 // Receive Packet Ready

/* Bit/Fields in Register RXCSRH6 of Module USB                               */
#define USB_RXCSRH6_AUTOCL_B        0x00000080 // Auto Clear
#define USB_RXCSRH6_AUTORQ_B        0x00000040 // Auto Request
#define USB_RXCSRH6_ISO_B           0x00000040 // Isochronous Transfers
#define USB_RXCSRH6_DMAEN_B         0x00000020 // DMA Request Enable
#define USB_RXCSRH6_DISNYET_B       0x00000010 // Disable NYET
#define USB_RXCSRH6_PIDERR_B        0x00000010 // PID Error
#define USB_RXCSRH6_DMAMOD_B        0x00000008 // DMA Request Mode
#define USB_RXCSRH6_DTWE_B          0x00000004 // Data Toggle Write Enable
#define USB_RXCSRH6_DT_B            0x00000002 // Data Toggle

/* Bit/Fields in Register RXCOUNT6 of Module USB                              */
#define USB_RXCOUNT6_COUNT_M        0x00001FFF // Receive Packet Count
#define USB_RXCOUNT6_COUNT_S        0          // Receive Packet Count

/* Bit/Fields in Register TXTYPE6 of Module USB                               */
#define USB_TXTYPE6_SPEED_M         0x000000C0 // Operating Speed
#define USB_TXTYPE6_SPEED_S         6          // Operating Speed
#define USB_TXTYPE6_SPEED_DFLT_V    0x00000000 // Default
#define USB_TXTYPE6_SPEED_FULL_V    0x00000080 // Full
#define USB_TXTYPE6_SPEED_LOW_V     0x000000C0 // Low
#define USB_TXTYPE6_PROTO_M         0x00000030 // Protocol
#define USB_TXTYPE6_PROTO_S         4          // Protocol
#define USB_TXTYPE6_PROTO_CTRL_V    0x00000000 // Control
#define USB_TXTYPE6_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_TXTYPE6_PROTO_BULK_V    0x00000020 // Bulk
#define USB_TXTYPE6_PROTO_INT_V     0x00000030 // Interrupt
#define USB_TXTYPE6_TEP_M           0x0000000F // Target Endpoint Number
#define USB_TXTYPE6_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register RXTYPE6 of Module USB                               */
#define USB_RXTYPE6_SPEED_M         0x000000C0 // Operating Speed
#define USB_RXTYPE6_SPEED_S         6          // Operating Speed
#define USB_RXTYPE6_SPEED_DFLT_V    0x00000000 // Default
#define USB_RXTYPE6_SPEED_FULL_V    0x00000080 // Full
#define USB_RXTYPE6_SPEED_LOW_V     0x000000C0 // Low
#define USB_RXTYPE6_PROTO_M         0x00000030 // Protocol
#define USB_RXTYPE6_PROTO_S         4          // Protocol
#define USB_RXTYPE6_PROTO_CTRL_V    0x00000000 // Control
#define USB_RXTYPE6_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_RXTYPE6_PROTO_BULK_V    0x00000020 // Bulk
#define USB_RXTYPE6_PROTO_INT_V     0x00000030 // Interrupt
#define USB_RXTYPE6_TEP_M           0x0000000F // Target Endpoint Number
#define USB_RXTYPE6_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register TXMAXP7 of Module USB                               */
#define USB_TXMAXP7_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_TXMAXP7_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register TXCSRL7 of Module USB                               */
#define USB_TXCSRL7_NAKTO_B         0x00000080 // NAK Timeout
#define USB_TXCSRL7_CLRDT_B         0x00000040 // Clear Data Toggle
#define USB_TXCSRL7_STALLED_B       0x00000020 // Endpoint Stalled
#define USB_TXCSRL7_STALL_B         0x00000010 // Send STALL
#define USB_TXCSRL7_SETUP_B         0x00000010 // Setup Packet
#define USB_TXCSRL7_FLUSH_B         0x00000008 // Flush FIFO
#define USB_TXCSRL7_ERROR_B         0x00000004 // Error
#define USB_TXCSRL7_UNDRN_B         0x00000004 // Underrun
#define USB_TXCSRL7_FIFONE_B        0x00000002 // FIFO Not Empty
#define USB_TXCSRL7_TXRDY_B         0x00000001 // Transmit Packet Ready

/* Bit/Fields in Register TXCSRH7 of Module USB                               */
#define USB_TXCSRH7_AUTOSET_B       0x00000080 // Auto Set
#define USB_TXCSRH7_ISO_B           0x00000040 // Isochronous Transfers
#define USB_TXCSRH7_MODE_B          0x00000020 // Mode
#define USB_TXCSRH7_DMAEN_B         0x00000010 // DMA Request Enable
#define USB_TXCSRH7_FDT_B           0x00000008 // Force Data Toggle
#define USB_TXCSRH7_DMAMOD_B        0x00000004 // DMA Request Mode
#define USB_TXCSRH7_DTWE_B          0x00000002 // Data Toggle Write Enable
#define USB_TXCSRH7_DT_B            0x00000001 // Data Toggle

/* Bit/Fields in Register RXMAXP7 of Module USB                               */
#define USB_RXMAXP7_MAXLOAD_M       0x000007FF // Maximum Payload
#define USB_RXMAXP7_MAXLOAD_S       0          // Maximum Payload

/* Bit/Fields in Register RXCSRL7 of Module USB                               */
#define USB_RXCSRL7_CLRDT_B         0x00000080 // Clear Data Toggle
#define USB_RXCSRL7_STALLED_B       0x00000040 // Endpoint Stalled
#define USB_RXCSRL7_REQPKT_B        0x00000020 // Request Packet
#define USB_RXCSRL7_STALL_B         0x00000020 // Send STALL
#define USB_RXCSRL7_FLUSH_B         0x00000010 // Flush FIFO
#define USB_RXCSRL7_DATAERR_B       0x00000008 // Data Error
#define USB_RXCSRL7_NAKTO_B         0x00000008 // NAK Timeout
#define USB_RXCSRL7_ERROR_B         0x00000004 // Error
#define USB_RXCSRL7_OVER_B          0x00000004 // Overrun
#define USB_RXCSRL7_FULL_B          0x00000002 // FIFO Full
#define USB_RXCSRL7_RXRDY_B         0x00000001 // Receive Packet Ready

/* Bit/Fields in Register RXCSRH7 of Module USB                               */
#define USB_RXCSRH7_AUTOCL_B        0x00000080 // Auto Clear
#define USB_RXCSRH7_ISO_B           0x00000040 // Isochronous Transfers
#define USB_RXCSRH7_AUTORQ_B        0x00000040 // Auto Request
#define USB_RXCSRH7_DMAEN_B         0x00000020 // DMA Request Enable
#define USB_RXCSRH7_PIDERR_B        0x00000010 // PID Error
#define USB_RXCSRH7_DISNYET_B       0x00000010 // Disable NYET
#define USB_RXCSRH7_DMAMOD_B        0x00000008 // DMA Request Mode
#define USB_RXCSRH7_DTWE_B          0x00000004 // Data Toggle Write Enable
#define USB_RXCSRH7_DT_B            0x00000002 // Data Toggle

/* Bit/Fields in Register RXCOUNT7 of Module USB                              */
#define USB_RXCOUNT7_COUNT_M        0x00001FFF // Receive Packet Count
#define USB_RXCOUNT7_COUNT_S        0          // Receive Packet Count

/* Bit/Fields in Register TXTYPE7 of Module USB                               */
#define USB_TXTYPE7_SPEED_M         0x000000C0 // Operating Speed
#define USB_TXTYPE7_SPEED_S         6          // Operating Speed
#define USB_TXTYPE7_SPEED_DFLT_V    0x00000000 // Default
#define USB_TXTYPE7_SPEED_FULL_V    0x00000080 // Full
#define USB_TXTYPE7_SPEED_LOW_V     0x000000C0 // Low
#define USB_TXTYPE7_PROTO_M         0x00000030 // Protocol
#define USB_TXTYPE7_PROTO_S         4          // Protocol
#define USB_TXTYPE7_PROTO_CTRL_V    0x00000000 // Control
#define USB_TXTYPE7_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_TXTYPE7_PROTO_BULK_V    0x00000020 // Bulk
#define USB_TXTYPE7_PROTO_INT_V     0x00000030 // Interrupt
#define USB_TXTYPE7_TEP_M           0x0000000F // Target Endpoint Number
#define USB_TXTYPE7_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register RXTYPE7 of Module USB                               */
#define USB_RXTYPE7_SPEED_M         0x000000C0 // Operating Speed
#define USB_RXTYPE7_SPEED_S         6          // Operating Speed
#define USB_RXTYPE7_SPEED_DFLT_V    0x00000000 // Default
#define USB_RXTYPE7_SPEED_FULL_V    0x00000080 // Full
#define USB_RXTYPE7_SPEED_LOW_V     0x000000C0 // Low
#define USB_RXTYPE7_PROTO_M         0x00000030 // Protocol
#define USB_RXTYPE7_PROTO_S         4          // Protocol
#define USB_RXTYPE7_PROTO_CTRL_V    0x00000000 // Control
#define USB_RXTYPE7_PROTO_ISOC_V    0x00000010 // Isochronous
#define USB_RXTYPE7_PROTO_BULK_V    0x00000020 // Bulk
#define USB_RXTYPE7_PROTO_INT_V     0x00000030 // Interrupt
#define USB_RXTYPE7_TEP_M           0x0000000F // Target Endpoint Number
#define USB_RXTYPE7_TEP_S           0          // Target Endpoint Number

/* Bit/Fields in Register RQPKTCOUNT1 of Module USB                           */
#define USB_RQPKTCOUNT1_M           0x0000FFFF // Block Transfer Packet Count
#define USB_RQPKTCOUNT1_S           0          // Block Transfer Packet Count

/* Bit/Fields in Register RQPKTCOUNT2 of Module USB                           */
#define USB_RQPKTCOUNT2_M           0x0000FFFF // Block Transfer Packet Count
#define USB_RQPKTCOUNT2_S           0          // Block Transfer Packet Count

/* Bit/Fields in Register RQPKTCOUNT3 of Module USB                           */
#define USB_RQPKTCOUNT3_M           0x0000FFFF // Block Transfer Packet Count
#define USB_RQPKTCOUNT3_S           0          // Block Transfer Packet Count

/* Bit/Fields in Register RQPKTCOUNT4 of Module USB                           */
#define USB_RQPKTCOUNT4_COUNT_M     0x0000FFFF // Block Transfer Packet Count
#define USB_RQPKTCOUNT4_COUNT_S     0          // Block Transfer Packet Count

/* Bit/Fields in Register RQPKTCOUNT5 of Module USB                           */
#define USB_RQPKTCOUNT5_COUNT_M     0x0000FFFF // Block Transfer Packet Count
#define USB_RQPKTCOUNT5_COUNT_S     0          // Block Transfer Packet Count

/* Bit/Fields in Register RQPKTCOUNT6 of Module USB                           */
#define USB_RQPKTCOUNT6_COUNT_M     0x0000FFFF // Block Transfer Packet Count
#define USB_RQPKTCOUNT6_COUNT_S     0          // Block Transfer Packet Count

/* Bit/Fields in Register RQPKTCOUNT7 of Module USB                           */
#define USB_RQPKTCOUNT7_COUNT_M     0x0000FFFF // Block Transfer Packet Count
#define USB_RQPKTCOUNT7_COUNT_S     0          // Block Transfer Packet Count

/* Bit/Fields in Register RXDPKTBUFDIS of Module USB                          */
#define USB_RXDPKTBUFDIS_EP7_B      0x00000080 // EP7 RX Double-Packet Buffer
#define USB_RXDPKTBUFDIS_EP6_B      0x00000040 // EP6 RX Double-Packet Buffer
#define USB_RXDPKTBUFDIS_EP5_B      0x00000020 // EP5 RX Double-Packet Buffer
#define USB_RXDPKTBUFDIS_EP4_B      0x00000010 // EP4 RX Double-Packet Buffer
#define USB_RXDPKTBUFDIS_EP3_B      0x00000008 // EP3 RX Double-Packet Buffer
#define USB_RXDPKTBUFDIS_EP2_B      0x00000004 // EP2 RX Double-Packet Buffer
#define USB_RXDPKTBUFDIS_EP1_B      0x00000002 // EP1 RX Double-Packet Buffer

/* Bit/Fields in Register TXDPKTBUFDIS of Module USB                          */
#define USB_TXDPKTBUFDIS_EP7_B      0x00000080 // EP7 TX Double-Packet Buffer
#define USB_TXDPKTBUFDIS_EP6_B      0x00000040 // EP6 TX Double-Packet Buffer
#define USB_TXDPKTBUFDIS_EP5_B      0x00000020 // EP5 TX Double-Packet Buffer
#define USB_TXDPKTBUFDIS_EP4_B      0x00000010 // EP4 TX Double-Packet Buffer
#define USB_TXDPKTBUFDIS_EP3_B      0x00000008 // EP3 TX Double-Packet Buffer
#define USB_TXDPKTBUFDIS_EP2_B      0x00000004 // EP2 TX Double-Packet Buffer
#define USB_TXDPKTBUFDIS_EP1_B      0x00000002 // EP1 TX Double-Packet Buffer

/* Bit/Fields in Register EPC of Module USB                                   */
#define USB_EPC_PFLTACT_M           0x00000300 // Power Fault Action
#define USB_EPC_PFLTACT_S           8          // Power Fault Action
#define USB_EPC_PFLTACT_UNCHG_V     0x00000000 // Unchanged
#define USB_EPC_PFLTACT_TRIS_V      0x00000100 // Tristate
#define USB_EPC_PFLTACT_LOW_V       0x00000200 // Low
#define USB_EPC_PFLTACT_HIGH_V      0x00000300 // High
#define USB_EPC_PFLTAEN_B           0x00000040 // Power Fault Action Enable
#define USB_EPC_PFLTSEN_HIGH_B      0x00000020 // Power Fault Sense
#define USB_EPC_PFLTEN_B            0x00000010 // Power Fault Input Enable
#define USB_EPC_EPENDE_B            0x00000004 // EPEN Drive Enable
#define USB_EPC_EPEN_M              0x00000003 // External Power Supply Enable
#define USB_EPC_EPEN_S              0          // External Power Supply Enable
#define USB_EPC_EPEN_LOW_V          0x00000000 // Power Enable Active Low
#define USB_EPC_EPEN_HIGH_V         0x00000001 // Power Enable Active High
#define USB_EPC_EPEN_VBLOW_V        0x00000002 // Power Enable High if VBUS Low
#define USB_EPC_EPEN_VBHIGH_V       0x00000003 // Power Enable High if VBUS High

/* Bit/Fields in Register EPCRIS of Module USB                                */
#define USB_EPCRIS_PF_B             0x00000001 // USB Power Fault Interrupt Status

/* Bit/Fields in Register EPCIM of Module USB                                 */
#define USB_EPCIM_PF_B              0x00000001 // USB Power Fault Interrupt Mask

/* Bit/Fields in Register EPCISC of Module USB                                */
#define USB_EPCISC_PF_B             0x00000001 // USB Power Fault Interrupt Status

/* Bit/Fields in Register DRRIS of Module USB                                 */
#define USB_DRRIS_RESUME_B          0x00000001 // RESUME Interrupt Status

/* Bit/Fields in Register DRIM of Module USB                                  */
#define USB_DRIM_RESUME_B           0x00000001 // RESUME Interrupt Mask

/* Bit/Fields in Register DRISC of Module USB                                 */
#define USB_DRISC_RESUME_B          0x00000001 // RESUME Interrupt Status and

/* Bit/Fields in Register GPCS of Module USB                                  */
#define USB_GPCS_DEVMODOTG_B        0x00000002 // Enable Device Mode
#define USB_GPCS_DEVMOD_B           0x00000001 // Device Mode

/* Bit/Fields in Register VDC of Module USB                                   */
#define USB_VDC_VBDEN_B             0x00000001 // VBUS Droop Enable

/* Bit/Fields in Register VDCRIS of Module USB                                */
#define USB_VDCRIS_VD_B             0x00000001 // VBUS Droop Raw Interrupt Status

/* Bit/Fields in Register VDCIM of Module USB                                 */
#define USB_VDCIM_VD_B              0x00000001 // VBUS Droop Interrupt Mask

/* Bit/Fields in Register VDCISC of Module USB                                */
#define USB_VDCISC_VD_B             0x00000001 // VBUS Droop Interrupt Status and

/* Bit/Fields in Register IDVRIS of Module USB                                */
#define USB_IDVRIS_ID_B             0x00000001 // ID Valid Detect Raw Interrupt

/* Bit/Fields in Register IDVIM of Module USB                                 */
#define USB_IDVIM_ID_B              0x00000001 // ID Valid Detect Interrupt Mask

/* Bit/Fields in Register IDVISC of Module USB                                */
#define USB_IDVISC_ID_B             0x00000001 // ID Valid Detect Interrupt Status

/* Bit/Fields in Register DMASEL of Module USB                                */
#define USB_DMASEL_DMACTX_M         0x00F00000 // DMA C TX Select
#define USB_DMASEL_DMACTX_S         20         // DMA C TX Select
#define USB_DMASEL_DMACRX_M         0x000F0000 // DMA C RX Select
#define USB_DMASEL_DMACRX_S         16         // DMA C RX Select
#define USB_DMASEL_DMABTX_M         0x0000F000 // DMA B TX Select
#define USB_DMASEL_DMABTX_S         12         // DMA B TX Select
#define USB_DMASEL_DMABRX_M         0x00000F00 // DMA B RX Select
#define USB_DMASEL_DMABRX_S         8          // DMA B RX Select
#define USB_DMASEL_DMAATX_M         0x000000F0 // DMA A TX Select
#define USB_DMASEL_DMAATX_S         4          // DMA A TX Select
#define USB_DMASEL_DMAARX_M         0x0000000F // DMA A RX Select
#define USB_DMASEL_DMAARX_S         0          // DMA A RX Select

/* Bit/Fields in Register PP of Module USB                                    */
#define USB_PP_ECNT_M               0x0000FF00 // Endpoint Count
#define USB_PP_ECNT_S               8          // Endpoint Count
#define USB_PP_USB_M                0x000000C0 // USB Capability
#define USB_PP_USB_S                6          // USB Capability
#define USB_PP_USB_DEVICE_V         0x00000040 // DEVICE
#define USB_PP_USB_HOSTDEVICE_V     0x00000080 // HOST
#define USB_PP_USB_OTG_V            0x000000C0 // OTG
#define USB_PP_PHY_B                0x00000010 // PHY Present
#define USB_PP_TYPE_M               0x0000000F // Controller Type
#define USB_PP_TYPE_S               0          // Controller Type
#define USB_PP_TYPE_0_V             0x00000000 // The first-generation USB


/******************************************************************************/
/*                                                                            */
/*                      EEPROM                                                */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register EESIZE of Module EEPROM                             */
#define EEPROM_EESIZE_BLKCNT_M      0x07FF0000 // Number of 16-Word Blocks
#define EEPROM_EESIZE_BLKCNT_S      16         // Number of 16-Word Blocks
#define EEPROM_EESIZE_WORDCNT_M     0x0000FFFF // Number of 32-Bit Words
#define EEPROM_EESIZE_WORDCNT_S     0          // Number of 32-Bit Words

/* Bit/Fields in Register EEBLOCK of Module EEPROM                            */
#define EEPROM_EEBLOCK_BLOCK_M      0x0000FFFF // Current Block
#define EEPROM_EEBLOCK_BLOCK_S      0          // Current Block

/* Bit/Fields in Register EERDWR of Module EEPROM                             */
#define EEPROM_EERDWR_VALUE_M       0xFFFFFFFF // EEPROM Read or Write Data
#define EEPROM_EERDWR_VALUE_S       0          // EEPROM Read or Write Data

/* Bit/Fields in Register EEDONE of Module EEPROM                             */
#define EEPROM_EEDONE_WRBUSY_B      0x00000020 // Write Busy
#define EEPROM_EEDONE_NOPERM_B      0x00000010 // Write Without Permission
#define EEPROM_EEDONE_WKCOPY_B      0x00000008 // Working on a Copy
#define EEPROM_EEDONE_WKERASE_B     0x00000004 // Working on an Erase
#define EEPROM_EEDONE_WORKING_B     0x00000001 // EEPROM Working

/* Bit/Fields in Register EESUPP of Module EEPROM                             */
#define EEPROM_EESUPP_PRETRY_B      0x00000008 // Programming Must Be Retried
#define EEPROM_EESUPP_ERETRY_B      0x00000004 // Erase Must Be Retried

/* Bit/Fields in Register EEPROT of Module EEPROM                             */
#define EEPROM_EEPROT_ACC_B         0x00000008 // Access Control
#define EEPROM_EEPROT_PROT_M        0x00000007 // Protection Control
#define EEPROM_EEPROT_PROT_S        0          // Protection Control
#define EEPROM_EEPROT_PROT_RWPW_V   0x00000001 // If there is a password, the

/* Bit/Fields in Register EEPASS0 of Module EEPROM                            */
#define EEPROM_EEPASS0_PASS_M       0xFFFFFFFF // Password
#define EEPROM_EEPASS0_PASS_S       0          // Password

/* Bit/Fields in Register EEPASS1 of Module EEPROM                            */
#define EEPROM_EEPASS1_PASS_M       0xFFFFFFFF // Password
#define EEPROM_EEPASS1_PASS_S       0          // Password

/* Bit/Fields in Register EEPASS2 of Module EEPROM                            */
#define EEPROM_EEPASS2_PASS_M       0xFFFFFFFF // Password
#define EEPROM_EEPASS2_PASS_S       0          // Password

/* Bit/Fields in Register EEINT of Module EEPROM                              */
#define EEPROM_EEINT_INT_B          0x00000001 // Interrupt Enable

/* Bit/Fields in Register EEHIDE of Module EEPROM                             */
#define EEPROM_EEHIDE_HN_M          0xFFFFFFFE // Hide Block
#define EEPROM_EEHIDE_HN_S          1          // Hide Block

/* Bit/Fields in Register EEDBGME of Module EEPROM                            */
#define EEPROM_EEDBGME_KEY_M        0xFFFF0000 // Erase Key
#define EEPROM_EEDBGME_KEY_S        16         // Erase Key
#define EEPROM_EEDBGME_ME_B         0x00000001 // Mass Erase

/* Bit/Fields in Register PP of Module EEPROM                                 */
#define EEPROM_PP_SIZE_M            0x0000001F // EEPROM Size
#define EEPROM_PP_SIZE_S            0          // EEPROM Size


/******************************************************************************/
/*                                                                            */
/*                      SYSEXC                                                */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register RIS of Module SYSEXC                                */
#define SYSEXC_RIS_FPIXCRIS_B       0x00000020 // Floating-Point Inexact Exception
#define SYSEXC_RIS_FPOFCRIS_B       0x00000010 // Floating-Point Overflow
#define SYSEXC_RIS_FPUFCRIS_B       0x00000008 // Floating-Point Underflow
#define SYSEXC_RIS_FPIOCRIS_B       0x00000004 // Floating-Point Invalid Operation
#define SYSEXC_RIS_FPDZCRIS_B       0x00000002 // Floating-Point Divide By 0
#define SYSEXC_RIS_FPIDCRIS_B       0x00000001 // Floating-Point Input Denormal

/* Bit/Fields in Register IM of Module SYSEXC                                 */
#define SYSEXC_IM_FPIXCIM_B         0x00000020 // Floating-Point Inexact Exception
#define SYSEXC_IM_FPOFCIM_B         0x00000010 // Floating-Point Overflow
#define SYSEXC_IM_FPUFCIM_B         0x00000008 // Floating-Point Underflow
#define SYSEXC_IM_FPIOCIM_B         0x00000004 // Floating-Point Invalid Operation
#define SYSEXC_IM_FPDZCIM_B         0x00000002 // Floating-Point Divide By 0
#define SYSEXC_IM_FPIDCIM_B         0x00000001 // Floating-Point Input Denormal

/* Bit/Fields in Register MIS of Module SYSEXC                                */
#define SYSEXC_MIS_FPIXCMIS_B       0x00000020 // Floating-Point Inexact Exception
#define SYSEXC_MIS_FPOFCMIS_B       0x00000010 // Floating-Point Overflow
#define SYSEXC_MIS_FPUFCMIS_B       0x00000008 // Floating-Point Underflow
#define SYSEXC_MIS_FPIOCMIS_B       0x00000004 // Floating-Point Invalid Operation
#define SYSEXC_MIS_FPDZCMIS_B       0x00000002 // Floating-Point Divide By 0
#define SYSEXC_MIS_FPIDCMIS_B       0x00000001 // Floating-Point Input Denormal

/* Bit/Fields in Register IC of Module SYSEXC                                 */
#define SYSEXC_IC_FPIXCIC_B         0x00000020 // Floating-Point Inexact Exception
#define SYSEXC_IC_FPOFCIC_B         0x00000010 // Floating-Point Overflow
#define SYSEXC_IC_FPUFCIC_B         0x00000008 // Floating-Point Underflow
#define SYSEXC_IC_FPIOCIC_B         0x00000004 // Floating-Point Invalid Operation
#define SYSEXC_IC_FPDZCIC_B         0x00000002 // Floating-Point Divide By 0
#define SYSEXC_IC_FPIDCIC_B         0x00000001 // Floating-Point Input Denormal


/******************************************************************************/
/*                                                                            */
/*                      HIB                                                   */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register RTCC of Module HIB                                  */
#define HIB_RTCC_M                  0xFFFFFFFF // RTC Counter
#define HIB_RTCC_S                  0          // RTC Counter

/* Bit/Fields in Register RTCM0 of Module HIB                                 */
#define HIB_RTCM0_M                 0xFFFFFFFF // RTC Match 0
#define HIB_RTCM0_S                 0          // RTC Match 0

/* Bit/Fields in Register RTCLD of Module HIB                                 */
#define HIB_RTCLD_M                 0xFFFFFFFF // RTC Load
#define HIB_RTCLD_S                 0          // RTC Load

/* Bit/Fields in Register CTL of Module HIB                                   */
#define HIB_CTL_WRC_B               0x80000000 // Write Complete/Capable
#define HIB_CTL_OSCDRV_B            0x00020000 // Oscillator Drive Capability
#define HIB_CTL_OSCBYP_B            0x00010000 // Oscillator Bypass
#define HIB_CTL_VBATSEL_M           0x00006000 // Select for Low-Battery
#define HIB_CTL_VBATSEL_S           13         // Select for Low-Battery
#define HIB_CTL_VBATSEL_1_9V_V      0x00000000 // 1.9 Volts
#define HIB_CTL_VBATSEL_2_1V_V      0x00002000 // 2.1 Volts (default)
#define HIB_CTL_VBATSEL_2_3V_V      0x00004000 // 2.3 Volts
#define HIB_CTL_VBATSEL_2_5V_V      0x00006000 // 2.5 Volts
#define HIB_CTL_BATCHK_B            0x00000400 // Check Battery Status
#define HIB_CTL_BATWKEN_B           0x00000200 // Wake on Low Battery
#define HIB_CTL_VDD3ON_B            0x00000100 // VDD Powered
#define HIB_CTL_VABORT_B            0x00000080 // Power Cut Abort Enable
#define HIB_CTL_CLK32EN_B           0x00000040 // Clocking Enable
#define HIB_CTL_PINWEN_B            0x00000010 // External Wake Pin Enable
#define HIB_CTL_RTCWEN_B            0x00000008 // RTC Wake-up Enable
#define HIB_CTL_HIBREQ_B            0x00000002 // Hibernation Request
#define HIB_CTL_RTCEN_B             0x00000001 // RTC Timer Enable

/* Bit/Fields in Register IM of Module HIB                                    */
#define HIB_IM_WC_B                 0x00000010 // External Write Complete/Capable
#define HIB_IM_EXTW_B               0x00000008 // External Wake-Up Interrupt Mask
#define HIB_IM_LOWBAT_B             0x00000004 // Low Battery Voltage Interrupt
#define HIB_IM_RTCALT0_B            0x00000001 // RTC Alert 0 Interrupt Mask

/* Bit/Fields in Register RIS of Module HIB                                   */
#define HIB_RIS_WC_B                0x00000010 // Write Complete/Capable Raw
#define HIB_RIS_EXTW_B              0x00000008 // External Wake-Up Raw Interrupt
#define HIB_RIS_LOWBAT_B            0x00000004 // Low Battery Voltage Raw
#define HIB_RIS_RTCALT0_B           0x00000001 // RTC Alert 0 Raw Interrupt Status

/* Bit/Fields in Register MIS of Module HIB                                   */
#define HIB_MIS_WC_B                0x00000010 // Write Complete/Capable Masked
#define HIB_MIS_EXTW_B              0x00000008 // External Wake-Up Masked
#define HIB_MIS_LOWBAT_B            0x00000004 // Low Battery Voltage Masked
#define HIB_MIS_RTCALT0_B           0x00000001 // RTC Alert 0 Masked Interrupt

/* Bit/Fields in Register IC of Module HIB                                    */
#define HIB_IC_WC_B                 0x00000010 // Write Complete/Capable Interrupt
#define HIB_IC_EXTW_B               0x00000008 // External Wake-Up Interrupt Clear
#define HIB_IC_LOWBAT_B             0x00000004 // Low Battery Voltage Interrupt
#define HIB_IC_RTCALT0_B            0x00000001 // RTC Alert0 Masked Interrupt

/* Bit/Fields in Register RTCT of Module HIB                                  */
#define HIB_RTCT_TRIM_M             0x0000FFFF // RTC Trim Value
#define HIB_RTCT_TRIM_S             0          // RTC Trim Value

/* Bit/Fields in Register RTCSS of Module HIB                                 */
#define HIB_RTCSS_RTCSSM_M          0x7FFF0000 // RTC Sub Seconds Match
#define HIB_RTCSS_RTCSSM_S          16         // RTC Sub Seconds Match
#define HIB_RTCSS_RTCSSC_M          0x00007FFF // RTC Sub Seconds Count
#define HIB_RTCSS_RTCSSC_S          0          // RTC Sub Seconds Count

/* Bit/Fields in Register DATA of Module HIB                                  */
#define HIB_DATA_RTD_M              0xFFFFFFFF // Hibernation Module NV Data
#define HIB_DATA_RTD_S              0          // Hibernation Module NV Data


/******************************************************************************/
/*                                                                            */
/*                      FLASH                                                 */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register FMA of Module FLASH                                 */
#define FLASH_FMA_OFFSET_M          0x0003FFFF // Address Offset
#define FLASH_FMA_OFFSET_S          0          // Address Offset

/* Bit/Fields in Register FMD of Module FLASH                                 */
#define FLASH_FMD_DATA_M            0xFFFFFFFF // Data Value
#define FLASH_FMD_DATA_S            0          // Data Value

/* Bit/Fields in Register FMC of Module FLASH                                 */
#define FLASH_FMC_WRKEY_V           0xA4420000 // FLASH write key
#define FLASH_FMC_COMT_B            0x00000008 // Commit Register Value
#define FLASH_FMC_MERASE_B          0x00000004 // Mass Erase Flash Memory
#define FLASH_FMC_ERASE_B           0x00000002 // Erase a Page of Flash Memory
#define FLASH_FMC_WRITE_B           0x00000001 // Write a Word into Flash Memory

/* Bit/Fields in Register FCRIS of Module FLASH                               */
#define FLASH_FCRIS_PROGRIS_B       0x00002000 // Program Verify Error Raw
#define FLASH_FCRIS_ERRIS_B         0x00000800 // Erase Verify Error Raw Interrupt
#define FLASH_FCRIS_INVDRIS_B       0x00000400 // Invalid Data Raw Interrupt
#define FLASH_FCRIS_VOLTRIS_B       0x00000200 // Pump Voltage Raw Interrupt
#define FLASH_FCRIS_ERIS_B          0x00000004 // EEPROM Raw Interrupt Status
#define FLASH_FCRIS_PRIS_B          0x00000002 // Programming Raw Interrupt Status
#define FLASH_FCRIS_ARIS_B          0x00000001 // Access Raw Interrupt Status

/* Bit/Fields in Register FCIM of Module FLASH                                */
#define FLASH_FCIM_PROGMASK_B       0x00002000 // PROGVER Interrupt Mask
#define FLASH_FCIM_ERMASK_B         0x00000800 // ERVER Interrupt Mask
#define FLASH_FCIM_INVDMASK_B       0x00000400 // Invalid Data Interrupt Mask
#define FLASH_FCIM_VOLTMASK_B       0x00000200 // VOLT Interrupt Mask
#define FLASH_FCIM_EMASK_B          0x00000004 // EEPROM Interrupt Mask
#define FLASH_FCIM_PMASK_B          0x00000002 // Programming Interrupt Mask
#define FLASH_FCIM_AMASK_B          0x00000001 // Access Interrupt Mask

/* Bit/Fields in Register FCMISC of Module FLASH                              */
#define FLASH_FCMISC_PROGMISC_B     0x00002000 // PROGVER Masked Interrupt Status
#define FLASH_FCMISC_ERMISC_B       0x00000800 // ERVER Masked Interrupt Status
#define FLASH_FCMISC_INVDMISC_B     0x00000400 // Invalid Data Masked Interrupt
#define FLASH_FCMISC_VOLTMISC_B     0x00000200 // VOLT Masked Interrupt Status and
#define FLASH_FCMISC_EMISC_B        0x00000004 // EEPROM Masked Interrupt Status
#define FLASH_FCMISC_PMISC_B        0x00000002 // Programming Masked Interrupt
#define FLASH_FCMISC_AMISC_B        0x00000001 // Access Masked Interrupt Status

/* Bit/Fields in Register FMC2 of Module FLASH                                */
#define FLASH_FMC2_WRBUF_B          0x00000001 // Buffered Flash Memory Write

/* Bit/Fields in Register FWBVAL of Module FLASH                              */
#define FLASH_FWBVAL_FWB_M          0xFFFFFFFF // Flash Memory Write Buffer
#define FLASH_FWBVAL_FWB_S          0          // Flash Memory Write Buffer

/* Bit/Fields in Register FWBN of Module FLASH                                */
#define FLASH_FWBN_DATA_M           0xFFFFFFFF // Data
#define FLASH_FWBN_DATA_S           0          // Data

/* Bit/Fields in Register FSIZE of Module FLASH                               */
#define FLASH_FSIZE_SIZE_M          0x0000FFFF // Flash Size
#define FLASH_FSIZE_SIZE_S          0          // Flash Size
#define FLASH_FSIZE_SIZE_256KB_V    0x0000007F // 256 KB of Flash

/* Bit/Fields in Register SSIZE of Module FLASH                               */
#define FLASH_SSIZE_SIZE_M          0x0000FFFF // SRAM Size
#define FLASH_SSIZE_SIZE_S          0          // SRAM Size
#define FLASH_SSIZE_SIZE_32KB_V     0x0000007F // 32 KB of SRAM

/* Bit/Fields in Register ROMSWMAP of Module FLASH                            */
#define FLASH_ROMSWMAP_SAFERTOS_B   0x00000001 // SafeRTOS Present

/* Bit/Fields in Register RMCTL of Module FLASH                               */
#define FLASH_RMCTL_BA_B            0x00000001 // Boot Alias

/* Bit/Fields in Register BOOTCFG of Module FLASH                             */
#define FLASH_BOOTCFG_NW_B          0x80000000 // Not Written
#define FLASH_BOOTCFG_PORT_M        0x0000E000 // Boot GPIO Port
#define FLASH_BOOTCFG_PORT_S        13         // Boot GPIO Port
#define FLASH_BOOTCFG_PORT_A_V      0x00000000 // Port A
#define FLASH_BOOTCFG_PORT_B_V      0x00002000 // Port B
#define FLASH_BOOTCFG_PORT_C_V      0x00004000 // Port C
#define FLASH_BOOTCFG_PORT_D_V      0x00006000 // Port D
#define FLASH_BOOTCFG_PORT_E_V      0x00008000 // Port E
#define FLASH_BOOTCFG_PORT_F_V      0x0000A000 // Port F
#define FLASH_BOOTCFG_PORT_G_V      0x0000C000 // Port G
#define FLASH_BOOTCFG_PORT_H_V      0x0000E000 // Port H
#define FLASH_BOOTCFG_PIN_M         0x00001C00 // Boot GPIO Pin
#define FLASH_BOOTCFG_PIN_S         10         // Boot GPIO Pin
#define FLASH_BOOTCFG_PIN_0_V       0x00000000 // Pin 0
#define FLASH_BOOTCFG_PIN_1_V       0x00000400 // Pin 1
#define FLASH_BOOTCFG_PIN_2_V       0x00000800 // Pin 2
#define FLASH_BOOTCFG_PIN_3_V       0x00000C00 // Pin 3
#define FLASH_BOOTCFG_PIN_4_V       0x00001000 // Pin 4
#define FLASH_BOOTCFG_PIN_5_V       0x00001400 // Pin 5
#define FLASH_BOOTCFG_PIN_6_V       0x00001800 // Pin 6
#define FLASH_BOOTCFG_PIN_7_V       0x00001C00 // Pin 7
#define FLASH_BOOTCFG_POL_B         0x00000200 // Boot GPIO Polarity
#define FLASH_BOOTCFG_EN_B          0x00000100 // Boot GPIO Enable
#define FLASH_BOOTCFG_KEY_B         0x00000010 // KEY Select
#define FLASH_BOOTCFG_DBG1_B        0x00000002 // Debug Control 1
#define FLASH_BOOTCFG_DBG0_B        0x00000001 // Debug Control 0

/* Bit/Fields in Register USERREG0 of Module FLASH                            */
#define FLASH_USERREG0_DATA_M       0xFFFFFFFF // User Data
#define FLASH_USERREG0_DATA_S       0          // User Data

/* Bit/Fields in Register USERREG1 of Module FLASH                            */
#define FLASH_USERREG1_DATA_M       0xFFFFFFFF // User Data
#define FLASH_USERREG1_DATA_S       0          // User Data

/* Bit/Fields in Register USERREG2 of Module FLASH                            */
#define FLASH_USERREG2_DATA_M       0xFFFFFFFF // User Data
#define FLASH_USERREG2_DATA_S       0          // User Data

/* Bit/Fields in Register USERREG3 of Module FLASH                            */
#define FLASH_USERREG3_DATA_M       0xFFFFFFFF // User Data
#define FLASH_USERREG3_DATA_S       0          // User Data


/******************************************************************************/
/*                                                                            */
/*                      SYSCTL                                                */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register DID0 of Module SYSCTL                               */
#define SYSCTL_DID0_VER_M           0x70000000 // DID0 Version
#define SYSCTL_DID0_VER_S           28         // DID0 Version
#define SYSCTL_DID0_VER_1_V         0x10000000 // Second version of the DID0
#define SYSCTL_DID0_CLASS_M         0x00FF0000 // Device Class
#define SYSCTL_DID0_CLASS_S         16         // Device Class
#define SYSCTL_DID0_MAJ_M           0x0000FF00 // Major Revision
#define SYSCTL_DID0_MAJ_S           8          // Major Revision
#define SYSCTL_DID0_MAJ_REVA_V      0x00000000 // Revision A (initial device)
#define SYSCTL_DID0_MAJ_REVB_V      0x00000100 // Revision B (first base layer
#define SYSCTL_DID0_MAJ_REVC_V      0x00000200 // Revision C (second base layer
#define SYSCTL_DID0_MIN_M           0x000000FF // Minor Revision
#define SYSCTL_DID0_MIN_S           0          // Minor Revision
#define SYSCTL_DID0_MIN_0_V         0x00000000 // Initial device, or a major
#define SYSCTL_DID0_MIN_1_V         0x00000001 // First metal layer change
#define SYSCTL_DID0_MIN_2_V         0x00000002 // Second metal layer change

/* Bit/Fields in Register DID1 of Module SYSCTL                               */
#define SYSCTL_DID1_VER_M           0xF0000000 // DID1 Version
#define SYSCTL_DID1_VER_S           28         // DID1 Version
#define SYSCTL_DID1_VER_1_V         0x10000000 // fury_ib
#define SYSCTL_DID1_FAM_M           0x0F000000 // Family
#define SYSCTL_DID1_FAM_S           24         // Family
#define SYSCTL_DID1_FAM_TIVA_V      0x00000000 // Tiva family of microcontollers
#define SYSCTL_DID1_PRTNO_M         0x00FF0000 // Part Number
#define SYSCTL_DID1_PRTNO_S         16         // Part Number
#define SYSCTL_DID1_PINCNT_M        0x0000E000 // Package Pin Count
#define SYSCTL_DID1_PINCNT_S        13         // Package Pin Count
#define SYSCTL_DID1_PINCNT_100_V    0x00004000 // 100-pin LQFP package
#define SYSCTL_DID1_PINCNT_64_V     0x00006000 // 64-pin LQFP package
#define SYSCTL_DID1_PINCNT_144_V    0x00008000 // 144-pin LQFP package
#define SYSCTL_DID1_PINCNT_157_V    0x0000A000 // 157-pin BGA package
#define SYSCTL_DID1_PINCNT_128_V    0x0000C000 // 128-pin TQFP package
#define SYSCTL_DID1_TEMP_M          0x000000E0 // Temperature Range
#define SYSCTL_DID1_TEMP_S          5          // Temperature Range
#define SYSCTL_DID1_TEMP_I_V        0x00000020 // Industrial temperature range
#define SYSCTL_DID1_TEMP_E_V        0x00000040 // Extended temperature range
#define SYSCTL_DID1_TEMP_IE_V       0x00000060 // Available in both industrial
#define SYSCTL_DID1_PKG_M           0x00000018 // Package Type
#define SYSCTL_DID1_PKG_S           3          // Package Type
#define SYSCTL_DID1_PKG_QFP_V       0x00000008 // QFP package
#define SYSCTL_DID1_PKG_BGA_V       0x00000010 // BGA package
#define SYSCTL_DID1_ROHS_B          0x00000004 // RoHS-Compliance
#define SYSCTL_DID1_QUAL_M          0x00000003 // Qualification Status
#define SYSCTL_DID1_QUAL_S          0          // Qualification Status
#define SYSCTL_DID1_QUAL_ES_V       0x00000000 // Engineering Sample (unqualified)
#define SYSCTL_DID1_QUAL_PP_V       0x00000001 // Pilot Production (unqualified)
#define SYSCTL_DID1_QUAL_FQ_V       0x00000002 // Fully Qualified

/* Bit/Fields in Register DC0 of Module SYSCTL                                */
#define SYSCTL_DC0_SRAMSZ_M         0xFFFF0000 // SRAM Size
#define SYSCTL_DC0_SRAMSZ_S         16         // SRAM Size
#define SYSCTL_DC0_SRAMSZ_2KB_V     0x00070000 // 2 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_4KB_V     0x000F0000 // 4 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_6KB_V     0x00170000 // 6 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_8KB_V     0x001F0000 // 8 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_12KB_V    0x002F0000 // 12 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_16KB_V    0x003F0000 // 16 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_20KB_V    0x004F0000 // 20 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_24KB_V    0x005F0000 // 24 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_32KB_V    0x007F0000 // 32 KB of SRAM
#define SYSCTL_DC0_FLASHSZ_M        0x0000FFFF // Flash Size
#define SYSCTL_DC0_FLASHSZ_S        0          // Flash Size
#define SYSCTL_DC0_FLASHSZ_8KB_V    0x00000003 // 8 KB of Flash
#define SYSCTL_DC0_FLASHSZ_16KB_V   0x00000007 // 16 KB of Flash
#define SYSCTL_DC0_FLASHSZ_32KB_V   0x0000000F // 32 KB of Flash
#define SYSCTL_DC0_FLASHSZ_64KB_V   0x0000001F // 64 KB of Flash
#define SYSCTL_DC0_FLASHSZ_96KB_V   0x0000002F // 96 KB of Flash
#define SYSCTL_DC0_FLASHSZ_128K_V   0x0000003F // 128 KB of Flash
#define SYSCTL_DC0_FLASHSZ_192K_V   0x0000005F // 192 KB of Flash
#define SYSCTL_DC0_FLASHSZ_256K_V   0x0000007F // 256 KB of Flash

/* Bit/Fields in Register DC1 of Module SYSCTL                                */
#define SYSCTL_DC1_WDT1_B           0x10000000 // Watchdog Timer1 Present
#define SYSCTL_DC1_CAN1_B           0x02000000 // CAN Module 1 Present
#define SYSCTL_DC1_CAN0_B           0x01000000 // CAN Module 0 Present
#define SYSCTL_DC1_PWM1_B           0x00200000 // PWM Module 1 Present
#define SYSCTL_DC1_PWM0_B           0x00100000 // PWM Module 0 Present
#define SYSCTL_DC1_ADC1_B           0x00020000 // ADC Module 1 Present
#define SYSCTL_DC1_ADC0_B           0x00010000 // ADC Module 0 Present
#define SYSCTL_DC1_MINSYSDIV_M      0x0000F000 // System Clock Divider
#define SYSCTL_DC1_MINSYSDIV_S      12         // System Clock Divider
#define SYSCTL_DC1_MINSYSDIV_80_V   0x00001000 // Specifies an 80-MHz CPU clock
#define SYSCTL_DC1_MINSYSDIV_66_V   0x00002000 // Specifies a 66-MHz CPU clock
#define SYSCTL_DC1_MINSYSDIV_50_V   0x00003000 // Specifies a 50-MHz CPU clock
#define SYSCTL_DC1_MINSYSDIV_40_V   0x00004000 // Specifies a 40-MHz CPU clock
#define SYSCTL_DC1_MINSYSDIV_25_V   0x00007000 // Specifies a 25-MHz clock with a
#define SYSCTL_DC1_MINSYSDIV_20_V   0x00009000 // Specifies a 20-MHz clock with a
#define SYSCTL_DC1_ADC1SPD_M        0x00000C00 // Max ADC1 Speed
#define SYSCTL_DC1_ADC1SPD_S        10         // Max ADC1 Speed
#define SYSCTL_DC1_ADC1SPD_125K_V   0x00000000 // 125K samples/second
#define SYSCTL_DC1_ADC1SPD_250K_V   0x00000400 // 250K samples/second
#define SYSCTL_DC1_ADC1SPD_500K_V   0x00000800 // 500K samples/second
#define SYSCTL_DC1_ADC1SPD_1M_V     0x00000C00 // 1M samples/second
#define SYSCTL_DC1_ADC0SPD_M        0x00000300 // Max ADC0 Speed
#define SYSCTL_DC1_ADC0SPD_S        8          // Max ADC0 Speed
#define SYSCTL_DC1_ADC0SPD_125K_V   0x00000000 // 125K samples/second
#define SYSCTL_DC1_ADC0SPD_250K_V   0x00000100 // 250K samples/second
#define SYSCTL_DC1_ADC0SPD_500K_V   0x00000200 // 500K samples/second
#define SYSCTL_DC1_ADC0SPD_1M_V     0x00000300 // 1M samples/second
#define SYSCTL_DC1_MPU_B            0x00000080 // MPU Present
#define SYSCTL_DC1_HIB_B            0x00000040 // Hibernation Module Present
#define SYSCTL_DC1_TEMP_B           0x00000020 // Temp Sensor Present
#define SYSCTL_DC1_PLL_B            0x00000010 // PLL Present
#define SYSCTL_DC1_WDT0_B           0x00000008 // Watchdog Timer 0 Present
#define SYSCTL_DC1_SWO_B            0x00000004 // SWO Trace Port Present
#define SYSCTL_DC1_SWD_B            0x00000002 // SWD Present
#define SYSCTL_DC1_JTAG_B           0x00000001 // JTAG Present

/* Bit/Fields in Register DC2 of Module SYSCTL                                */
#define SYSCTL_DC2_EPI0_B           0x40000000 // EPI Module 0 Present
#define SYSCTL_DC2_I2S0_B           0x10000000 // I2S Module 0 Present
#define SYSCTL_DC2_COMP2_B          0x04000000 // Analog Comparator 2 Present
#define SYSCTL_DC2_COMP1_B          0x02000000 // Analog Comparator 1 Present
#define SYSCTL_DC2_COMP0_B          0x01000000 // Analog Comparator 0 Present
#define SYSCTL_DC2_TIMER3_B         0x00080000 // Timer Module 3 Present
#define SYSCTL_DC2_TIMER2_B         0x00040000 // Timer Module 2 Present
#define SYSCTL_DC2_TIMER1_B         0x00020000 // Timer Module 1 Present
#define SYSCTL_DC2_TIMER0_B         0x00010000 // Timer Module 0 Present
#define SYSCTL_DC2_I2C1HS_B         0x00008000 // I2C Module 1 Speed
#define SYSCTL_DC2_I2C1_B           0x00004000 // I2C Module 1 Present
#define SYSCTL_DC2_I2C0HS_B         0x00002000 // I2C Module 0 Speed
#define SYSCTL_DC2_I2C0_B           0x00001000 // I2C Module 0 Present
#define SYSCTL_DC2_QEI1_B           0x00000200 // QEI Module 1 Present
#define SYSCTL_DC2_QEI0_B           0x00000100 // QEI Module 0 Present
#define SYSCTL_DC2_SSI1_B           0x00000020 // SSI Module 1 Present
#define SYSCTL_DC2_SSI0_B           0x00000010 // SSI Module 0 Present
#define SYSCTL_DC2_UART2_B          0x00000004 // UART Module 2 Present
#define SYSCTL_DC2_UART1_B          0x00000002 // UART Module 1 Present
#define SYSCTL_DC2_UART0_B          0x00000001 // UART Module 0 Present

/* Bit/Fields in Register DC3 of Module SYSCTL                                */
#define SYSCTL_DC3_32KHZ_B          0x80000000 // 32KHz Input Clock Available
#define SYSCTL_DC3_CCP5_B           0x20000000 // T2CCP1 Pin Present
#define SYSCTL_DC3_CCP4_B           0x10000000 // T2CCP0 Pin Present
#define SYSCTL_DC3_CCP3_B           0x08000000 // T1CCP1 Pin Present
#define SYSCTL_DC3_CCP2_B           0x04000000 // T1CCP0 Pin Present
#define SYSCTL_DC3_CCP1_B           0x02000000 // T0CCP1 Pin Present
#define SYSCTL_DC3_CCP0_B           0x01000000 // T0CCP0 Pin Present
#define SYSCTL_DC3_ADC0AIN7_B       0x00800000 // ADC Module 0 AIN7 Pin Present
#define SYSCTL_DC3_ADC0AIN6_B       0x00400000 // ADC Module 0 AIN6 Pin Present
#define SYSCTL_DC3_ADC0AIN5_B       0x00200000 // ADC Module 0 AIN5 Pin Present
#define SYSCTL_DC3_ADC0AIN4_B       0x00100000 // ADC Module 0 AIN4 Pin Present
#define SYSCTL_DC3_ADC0AIN3_B       0x00080000 // ADC Module 0 AIN3 Pin Present
#define SYSCTL_DC3_ADC0AIN2_B       0x00040000 // ADC Module 0 AIN2 Pin Present
#define SYSCTL_DC3_ADC0AIN1_B       0x00020000 // ADC Module 0 AIN1 Pin Present
#define SYSCTL_DC3_ADC0AIN0_B       0x00010000 // ADC Module 0 AIN0 Pin Present
#define SYSCTL_DC3_PWMFAULT_B       0x00008000 // PWM Fault Pin Present
#define SYSCTL_DC3_C2O_B            0x00004000 // C2o Pin Present
#define SYSCTL_DC3_C2PLUS_B         0x00002000 // C2+ Pin Present
#define SYSCTL_DC3_C2MINUS_B        0x00001000 // C2- Pin Present
#define SYSCTL_DC3_C1O_B            0x00000800 // C1o Pin Present
#define SYSCTL_DC3_C1PLUS_B         0x00000400 // C1+ Pin Present
#define SYSCTL_DC3_C1MINUS_B        0x00000200 // C1- Pin Present
#define SYSCTL_DC3_C0O_B            0x00000100 // C0o Pin Present
#define SYSCTL_DC3_C0PLUS_B         0x00000080 // C0+ Pin Present
#define SYSCTL_DC3_C0MINUS_B        0x00000040 // C0- Pin Present
#define SYSCTL_DC3_PWM5_B           0x00000020 // PWM5 Pin Present
#define SYSCTL_DC3_PWM4_B           0x00000010 // PWM4 Pin Present
#define SYSCTL_DC3_PWM3_B           0x00000008 // PWM3 Pin Present
#define SYSCTL_DC3_PWM2_B           0x00000004 // PWM2 Pin Present
#define SYSCTL_DC3_PWM1_B           0x00000002 // PWM1 Pin Present
#define SYSCTL_DC3_PWM0_B           0x00000001 // PWM0 Pin Present

/* Bit/Fields in Register DC4 of Module SYSCTL                                */
#define SYSCTL_DC4_EPHY0_B          0x40000000 // Ethernet PHY Layer 0 Present
#define SYSCTL_DC4_EMAC0_B          0x10000000 // Ethernet MAC Layer 0 Present
#define SYSCTL_DC4_E1588_B          0x01000000 // 1588 Capable
#define SYSCTL_DC4_PICAL_B          0x00040000 // PIOSC Calibrate
#define SYSCTL_DC4_CCP7_B           0x00008000 // T3CCP1 Pin Present
#define SYSCTL_DC4_CCP6_B           0x00004000 // T3CCP0 Pin Present
#define SYSCTL_DC4_UDMA_B           0x00002000 // Micro-DMA Module Present
#define SYSCTL_DC4_ROM_B            0x00001000 // Internal Code ROM Present
#define SYSCTL_DC4_GPIOJ_B          0x00000100 // GPIO Port J Present
#define SYSCTL_DC4_GPIOH_B          0x00000080 // GPIO Port H Present
#define SYSCTL_DC4_GPIOG_B          0x00000040 // GPIO Port G Present
#define SYSCTL_DC4_GPIOF_B          0x00000020 // GPIO Port F Present
#define SYSCTL_DC4_GPIOE_B          0x00000010 // GPIO Port E Present
#define SYSCTL_DC4_GPIOD_B          0x00000008 // GPIO Port D Present
#define SYSCTL_DC4_GPIOC_B          0x00000004 // GPIO Port C Present
#define SYSCTL_DC4_GPIOB_B          0x00000002 // GPIO Port B Present
#define SYSCTL_DC4_GPIOA_B          0x00000001 // GPIO Port A Present

/* Bit/Fields in Register DC5 of Module SYSCTL                                */
#define SYSCTL_DC5_PWMFAULT3_B      0x08000000 // PWM Fault 3 Pin Present
#define SYSCTL_DC5_PWMFAULT2_B      0x04000000 // PWM Fault 2 Pin Present
#define SYSCTL_DC5_PWMFAULT1_B      0x02000000 // PWM Fault 1 Pin Present
#define SYSCTL_DC5_PWMFAULT0_B      0x01000000 // PWM Fault 0 Pin Present
#define SYSCTL_DC5_PWMEFLT_B        0x00200000 // PWM Extended Fault Active
#define SYSCTL_DC5_PWMESYNC_B       0x00100000 // PWM Extended SYNC Active
#define SYSCTL_DC5_PWM7_B           0x00000080 // PWM7 Pin Present
#define SYSCTL_DC5_PWM6_B           0x00000040 // PWM6 Pin Present
#define SYSCTL_DC5_PWM5_B           0x00000020 // PWM5 Pin Present
#define SYSCTL_DC5_PWM4_B           0x00000010 // PWM4 Pin Present
#define SYSCTL_DC5_PWM3_B           0x00000008 // PWM3 Pin Present
#define SYSCTL_DC5_PWM2_B           0x00000004 // PWM2 Pin Present
#define SYSCTL_DC5_PWM1_B           0x00000002 // PWM1 Pin Present
#define SYSCTL_DC5_PWM0_B           0x00000001 // PWM0 Pin Present

/* Bit/Fields in Register DC6 of Module SYSCTL                                */
#define SYSCTL_DC6_USB0PHY_B        0x00000010 // USB Module 0 PHY Present
#define SYSCTL_DC6_USB0_M           0x00000003 // USB Module 0 Present
#define SYSCTL_DC6_USB0_S           0          // USB Module 0 Present
#define SYSCTL_DC6_USB0_DEV_V       0x00000001 // USB0 is Device Only
#define SYSCTL_DC6_USB0_HOSTDEV_V   0x00000002 // USB is Device or Host
#define SYSCTL_DC6_USB0_OTG_V       0x00000003 // USB0 is OTG

/* Bit/Fields in Register DC7 of Module SYSCTL                                */
#define SYSCTL_DC7_DMACH30_B        0x40000000 // DMA Channel 30
#define SYSCTL_DC7_DMACH29_B        0x20000000 // DMA Channel 29
#define SYSCTL_DC7_DMACH28_B        0x10000000 // DMA Channel 28
#define SYSCTL_DC7_DMACH27_B        0x08000000 // DMA Channel 27
#define SYSCTL_DC7_DMACH26_B        0x04000000 // DMA Channel 26
#define SYSCTL_DC7_DMACH25_B        0x02000000 // DMA Channel 25
#define SYSCTL_DC7_DMACH24_B        0x01000000 // DMA Channel 24
#define SYSCTL_DC7_DMACH23_B        0x00800000 // DMA Channel 23
#define SYSCTL_DC7_DMACH22_B        0x00400000 // DMA Channel 22
#define SYSCTL_DC7_DMACH21_B        0x00200000 // DMA Channel 21
#define SYSCTL_DC7_DMACH20_B        0x00100000 // DMA Channel 20
#define SYSCTL_DC7_DMACH19_B        0x00080000 // DMA Channel 19
#define SYSCTL_DC7_DMACH18_B        0x00040000 // DMA Channel 18
#define SYSCTL_DC7_DMACH17_B        0x00020000 // DMA Channel 17
#define SYSCTL_DC7_DMACH16_B        0x00010000 // DMA Channel 16
#define SYSCTL_DC7_DMACH15_B        0x00008000 // DMA Channel 15
#define SYSCTL_DC7_DMACH14_B        0x00004000 // DMA Channel 14
#define SYSCTL_DC7_DMACH13_B        0x00002000 // DMA Channel 13
#define SYSCTL_DC7_DMACH12_B        0x00001000 // DMA Channel 12
#define SYSCTL_DC7_DMACH11_B        0x00000800 // DMA Channel 11
#define SYSCTL_DC7_DMACH10_B        0x00000400 // DMA Channel 10
#define SYSCTL_DC7_DMACH9_B         0x00000200 // DMA Channel 9
#define SYSCTL_DC7_DMACH8_B         0x00000100 // DMA Channel 8
#define SYSCTL_DC7_DMACH7_B         0x00000080 // DMA Channel 7
#define SYSCTL_DC7_DMACH6_B         0x00000040 // DMA Channel 6
#define SYSCTL_DC7_DMACH5_B         0x00000020 // DMA Channel 5
#define SYSCTL_DC7_DMACH4_B         0x00000010 // DMA Channel 4
#define SYSCTL_DC7_DMACH3_B         0x00000008 // DMA Channel 3
#define SYSCTL_DC7_DMACH2_B         0x00000004 // DMA Channel 2
#define SYSCTL_DC7_DMACH1_B         0x00000002 // DMA Channel 1
#define SYSCTL_DC7_DMACH0_B         0x00000001 // DMA Channel 0

/* Bit/Fields in Register DC8 of Module SYSCTL                                */
#define SYSCTL_DC8_ADC1AIN15_B      0x80000000 // ADC Module 1 AIN15 Pin Present
#define SYSCTL_DC8_ADC1AIN14_B      0x40000000 // ADC Module 1 AIN14 Pin Present
#define SYSCTL_DC8_ADC1AIN13_B      0x20000000 // ADC Module 1 AIN13 Pin Present
#define SYSCTL_DC8_ADC1AIN12_B      0x10000000 // ADC Module 1 AIN12 Pin Present
#define SYSCTL_DC8_ADC1AIN11_B      0x08000000 // ADC Module 1 AIN11 Pin Present
#define SYSCTL_DC8_ADC1AIN10_B      0x04000000 // ADC Module 1 AIN10 Pin Present
#define SYSCTL_DC8_ADC1AIN9_B       0x02000000 // ADC Module 1 AIN9 Pin Present
#define SYSCTL_DC8_ADC1AIN8_B       0x01000000 // ADC Module 1 AIN8 Pin Present
#define SYSCTL_DC8_ADC1AIN7_B       0x00800000 // ADC Module 1 AIN7 Pin Present
#define SYSCTL_DC8_ADC1AIN6_B       0x00400000 // ADC Module 1 AIN6 Pin Present
#define SYSCTL_DC8_ADC1AIN5_B       0x00200000 // ADC Module 1 AIN5 Pin Present
#define SYSCTL_DC8_ADC1AIN4_B       0x00100000 // ADC Module 1 AIN4 Pin Present
#define SYSCTL_DC8_ADC1AIN3_B       0x00080000 // ADC Module 1 AIN3 Pin Present
#define SYSCTL_DC8_ADC1AIN2_B       0x00040000 // ADC Module 1 AIN2 Pin Present
#define SYSCTL_DC8_ADC1AIN1_B       0x00020000 // ADC Module 1 AIN1 Pin Present
#define SYSCTL_DC8_ADC1AIN0_B       0x00010000 // ADC Module 1 AIN0 Pin Present
#define SYSCTL_DC8_ADC0AIN15_B      0x00008000 // ADC Module 0 AIN15 Pin Present
#define SYSCTL_DC8_ADC0AIN14_B      0x00004000 // ADC Module 0 AIN14 Pin Present
#define SYSCTL_DC8_ADC0AIN13_B      0x00002000 // ADC Module 0 AIN13 Pin Present
#define SYSCTL_DC8_ADC0AIN12_B      0x00001000 // ADC Module 0 AIN12 Pin Present
#define SYSCTL_DC8_ADC0AIN11_B      0x00000800 // ADC Module 0 AIN11 Pin Present
#define SYSCTL_DC8_ADC0AIN10_B      0x00000400 // ADC Module 0 AIN10 Pin Present
#define SYSCTL_DC8_ADC0AIN9_B       0x00000200 // ADC Module 0 AIN9 Pin Present
#define SYSCTL_DC8_ADC0AIN8_B       0x00000100 // ADC Module 0 AIN8 Pin Present
#define SYSCTL_DC8_ADC0AIN7_B       0x00000080 // ADC Module 0 AIN7 Pin Present
#define SYSCTL_DC8_ADC0AIN6_B       0x00000040 // ADC Module 0 AIN6 Pin Present
#define SYSCTL_DC8_ADC0AIN5_B       0x00000020 // ADC Module 0 AIN5 Pin Present
#define SYSCTL_DC8_ADC0AIN4_B       0x00000010 // ADC Module 0 AIN4 Pin Present
#define SYSCTL_DC8_ADC0AIN3_B       0x00000008 // ADC Module 0 AIN3 Pin Present
#define SYSCTL_DC8_ADC0AIN2_B       0x00000004 // ADC Module 0 AIN2 Pin Present
#define SYSCTL_DC8_ADC0AIN1_B       0x00000002 // ADC Module 0 AIN1 Pin Present
#define SYSCTL_DC8_ADC0AIN0_B       0x00000001 // ADC Module 0 AIN0 Pin Present

/* Bit/Fields in Register PBORCTL of Module SYSCTL                            */
#define SYSCTL_PBORCTL_BOR0_B       0x00000004 // VDD under BOR0 Event Action
#define SYSCTL_PBORCTL_BOR1_B       0x00000002 // VDD under BOR1 Event Action

/* Bit/Fields in Register SRCR0 of Module SYSCTL                              */
#define SYSCTL_SRCR0_WDT1_B         0x10000000 // WDT1 Reset Control
#define SYSCTL_SRCR0_CAN1_B         0x02000000 // CAN1 Reset Control
#define SYSCTL_SRCR0_CAN0_B         0x01000000 // CAN0 Reset Control
#define SYSCTL_SRCR0_PWM0_B         0x00100000 // PWM Reset Control
#define SYSCTL_SRCR0_ADC1_B         0x00020000 // ADC1 Reset Control
#define SYSCTL_SRCR0_ADC0_B         0x00010000 // ADC0 Reset Control
#define SYSCTL_SRCR0_HIB_B          0x00000040 // HIB Reset Control
#define SYSCTL_SRCR0_WDT0_B         0x00000008 // WDT0 Reset Control

/* Bit/Fields in Register SRCR1 of Module SYSCTL                              */
#define SYSCTL_SRCR1_COMP1_B        0x02000000 // Analog Comp 1 Reset Control
#define SYSCTL_SRCR1_COMP0_B        0x01000000 // Analog Comp 0 Reset Control
#define SYSCTL_SRCR1_TIMER3_B       0x00080000 // Timer 3 Reset Control
#define SYSCTL_SRCR1_TIMER2_B       0x00040000 // Timer 2 Reset Control
#define SYSCTL_SRCR1_TIMER1_B       0x00020000 // Timer 1 Reset Control
#define SYSCTL_SRCR1_TIMER0_B       0x00010000 // Timer 0 Reset Control
#define SYSCTL_SRCR1_I2C1_B         0x00004000 // I2C1 Reset Control
#define SYSCTL_SRCR1_I2C0_B         0x00001000 // I2C0 Reset Control
#define SYSCTL_SRCR1_QEI1_B         0x00000200 // QEI1 Reset Control
#define SYSCTL_SRCR1_QEI0_B         0x00000100 // QEI0 Reset Control
#define SYSCTL_SRCR1_SSI1_B         0x00000020 // SSI1 Reset Control
#define SYSCTL_SRCR1_SSI0_B         0x00000010 // SSI0 Reset Control
#define SYSCTL_SRCR1_UART2_B        0x00000004 // UART2 Reset Control
#define SYSCTL_SRCR1_UART1_B        0x00000002 // UART1 Reset Control
#define SYSCTL_SRCR1_UART0_B        0x00000001 // UART0 Reset Control

/* Bit/Fields in Register SRCR2 of Module SYSCTL                              */
#define SYSCTL_SRCR2_USB0_B         0x00010000 // USB0 Reset Control
#define SYSCTL_SRCR2_UDMA_B         0x00002000 // Micro-DMA Reset Control
#define SYSCTL_SRCR2_GPIOF_B        0x00000020 // Port F Reset Control
#define SYSCTL_SRCR2_GPIOE_B        0x00000010 // Port E Reset Control
#define SYSCTL_SRCR2_GPIOD_B        0x00000008 // Port D Reset Control
#define SYSCTL_SRCR2_GPIOC_B        0x00000004 // Port C Reset Control
#define SYSCTL_SRCR2_GPIOB_B        0x00000002 // Port B Reset Control
#define SYSCTL_SRCR2_GPIOA_B        0x00000001 // Port A Reset Control

/* Bit/Fields in Register RIS of Module SYSCTL                                */
#define SYSCTL_RIS_BOR0RIS_B        0x00000800 // VDD under BOR0 Raw Interrupt
#define SYSCTL_RIS_VDDARIS_B        0x00000400 // VDDA Power OK Event Raw
#define SYSCTL_RIS_MOSCPUPRIS_B     0x00000100 // MOSC Power Up Raw Interrupt
#define SYSCTL_RIS_USBPLLLRIS_B     0x00000080 // USB PLL Lock Raw Interrupt
#define SYSCTL_RIS_PLLLRIS_B        0x00000040 // PLL Lock Raw Interrupt Status
#define SYSCTL_RIS_MOFRIS_B         0x00000008 // Main Oscillator Failure Raw
#define SYSCTL_RIS_BOR1RIS_B        0x00000002 // VDD under BOR1 Raw Interrupt

/* Bit/Fields in Register IMC of Module SYSCTL                                */
#define SYSCTL_IMC_BOR0IM_B         0x00000800 // VDD under BOR0 Interrupt Mask
#define SYSCTL_IMC_VDDAIM_B         0x00000400 // VDDA Power OK Interrupt Mask
#define SYSCTL_IMC_MOSCPUPIM_B      0x00000100 // MOSC Power Up Interrupt Mask
#define SYSCTL_IMC_USBPLLLIM_B      0x00000080 // USB PLL Lock Interrupt Mask
#define SYSCTL_IMC_PLLLIM_B         0x00000040 // PLL Lock Interrupt Mask
#define SYSCTL_IMC_MOFIM_B          0x00000008 // Main Oscillator Failure
#define SYSCTL_IMC_BOR1IM_B         0x00000002 // VDD under BOR1 Interrupt Mask

/* Bit/Fields in Register MISC of Module SYSCTL                               */
#define SYSCTL_MISC_BOR0MIS_B       0x00000800 // VDD under BOR0 Masked Interrupt
#define SYSCTL_MISC_VDDAMIS_B       0x00000400 // VDDA Power OK Masked Interrupt
#define SYSCTL_MISC_MOSCPUPMIS_B    0x00000100 // MOSC Power Up Masked Interrupt
#define SYSCTL_MISC_USBPLLLMIS_B    0x00000080 // USB PLL Lock Masked Interrupt
#define SYSCTL_MISC_PLLLMIS_B       0x00000040 // PLL Lock Masked Interrupt Status
#define SYSCTL_MISC_MOFMIS_B        0x00000008 // Main Oscillator Failure Masked
#define SYSCTL_MISC_BOR1MIS_B       0x00000002 // VDD under BOR1 Masked Interrupt

/* Bit/Fields in Register RESC of Module SYSCTL                               */
#define SYSCTL_RESC_MOSCFAIL_B      0x00010000 // MOSC Failure Reset
#define SYSCTL_RESC_WDT1_B          0x00000020 // Watchdog Timer 1 Reset
#define SYSCTL_RESC_SW_B            0x00000010 // Software Reset
#define SYSCTL_RESC_WDT0_B          0x00000008 // Watchdog Timer 0 Reset
#define SYSCTL_RESC_BOR_B           0x00000004 // Brown-Out Reset
#define SYSCTL_RESC_POR_B           0x00000002 // Power-On Reset
#define SYSCTL_RESC_EXT_B           0x00000001 // External Reset

/* Bit/Fields in Register RCC of Module SYSCTL                                */
#define SYSCTL_RCC_ACG_B            0x08000000 // Auto Clock Gating
#define SYSCTL_RCC_SYSDIV_M         0x07800000 // System Clock Divisor
#define SYSCTL_RCC_SYSDIV_S         23         // System Clock Divisor
#define SYSCTL_RCC_USESYSDIV_B      0x00400000 // Enable System Clock Divider
#define SYSCTL_RCC_USEPWMDIV_B      0x00100000 // Enable PWM Clock Divisor
#define SYSCTL_RCC_PWMDIV_M         0x000E0000 // PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_S         17         // PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_2_V       0x00000000 // PWM clock /2
#define SYSCTL_RCC_PWMDIV_4_V       0x00020000 // PWM clock /4
#define SYSCTL_RCC_PWMDIV_8_V       0x00040000 // PWM clock /8
#define SYSCTL_RCC_PWMDIV_16_V      0x00060000 // PWM clock /16
#define SYSCTL_RCC_PWMDIV_32_V      0x00080000 // PWM clock /32
#define SYSCTL_RCC_PWMDIV_64_V      0x000A0000 // PWM clock /64
#define SYSCTL_RCC_PWRDN_B          0x00002000 // PLL Power Down
#define SYSCTL_RCC_BYPASS_B         0x00000800 // PLL Bypass
#define SYSCTL_RCC_XTAL_M           0x000007C0 // Crystal Value
#define SYSCTL_RCC_XTAL_S           6          // Crystal Value
#define SYSCTL_RCC_XTAL_4MHZ_V      0x00000180 // 4 MHz
#define SYSCTL_RCC_XTAL_4_09MHZ_V   0x000001C0 // 4.096 MHz
#define SYSCTL_RCC_XTAL_4_91MHZ_V   0x00000200 // 4.9152 MHz
#define SYSCTL_RCC_XTAL_5MHZ_V      0x00000240 // 5 MHz
#define SYSCTL_RCC_XTAL_5_12MHZ_V   0x00000280 // 5.12 MHz
#define SYSCTL_RCC_XTAL_6MHZ_V      0x000002C0 // 6 MHz
#define SYSCTL_RCC_XTAL_6_14MHZ_V   0x00000300 // 6.144 MHz
#define SYSCTL_RCC_XTAL_7_37MHZ_V   0x00000340 // 7.3728 MHz
#define SYSCTL_RCC_XTAL_8MHZ_V      0x00000380 // 8 MHz
#define SYSCTL_RCC_XTAL_8_19MHZ_V   0x000003C0 // 8.192 MHz
#define SYSCTL_RCC_XTAL_10MHZ_V     0x00000400 // 10 MHz
#define SYSCTL_RCC_XTAL_12MHZ_V     0x00000440 // 12 MHz
#define SYSCTL_RCC_XTAL_12_2MHZ_V   0x00000480 // 12.288 MHz
#define SYSCTL_RCC_XTAL_13_5MHZ_V   0x000004C0 // 13.56 MHz
#define SYSCTL_RCC_XTAL_14_3MHZ_V   0x00000500 // 14.31818 MHz
#define SYSCTL_RCC_XTAL_16MHZ_V     0x00000540 // 16 MHz
#define SYSCTL_RCC_XTAL_16_3MHZ_V   0x00000580 // 16.384 MHz
#define SYSCTL_RCC_XTAL_18MHZ_V     0x000005C0 // 18.0 MHz (USB)
#define SYSCTL_RCC_XTAL_20MHZ_V     0x00000600 // 20.0 MHz (USB)
#define SYSCTL_RCC_XTAL_24MHZ_V     0x00000640 // 24.0 MHz (USB)
#define SYSCTL_RCC_XTAL_25MHZ_V     0x00000680 // 25.0 MHz (USB)
#define SYSCTL_RCC_OSCSRC_M         0x00000030 // Oscillator Source
#define SYSCTL_RCC_OSCSRC_S         4          // Oscillator Source
#define SYSCTL_RCC_OSCSRC_MOSC_V    0x00000000 // MOSC
#define SYSCTL_RCC_OSCSRC_IOSC_V    0x00000010 // IOSC
#define SYSCTL_RCC_OSCSRC_IOSC_4_V  0x00000020 // IOSC/4
#define SYSCTL_RCC_OSCSRC_LFIOSC_V  0x00000030 // LFIOSC
#define SYSCTL_RCC_MOSCDIS_B        0x00000001 // Main Oscillator Disable

/* Bit/Fields in Register GPIOHBCTL of Module SYSCTL                          */
#define SYSCTL_GPIOHBCTL_PORTF_B    0x00000020 // Port F Advanced High-Performance
#define SYSCTL_GPIOHBCTL_PORTE_B    0x00000010 // Port E Advanced High-Performance
#define SYSCTL_GPIOHBCTL_PORTD_B    0x00000008 // Port D Advanced High-Performance
#define SYSCTL_GPIOHBCTL_PORTC_B    0x00000004 // Port C Advanced High-Performance
#define SYSCTL_GPIOHBCTL_PORTB_B    0x00000002 // Port B Advanced High-Performance
#define SYSCTL_GPIOHBCTL_PORTA_B    0x00000001 // Port A Advanced High-Performance

/* Bit/Fields in Register RCC2 of Module SYSCTL                               */
#define SYSCTL_RCC2_USERCC2_B       0x80000000 // Use RCC2
#define SYSCTL_RCC2_DIV400_B        0x40000000 // Divide PLL as 400 MHz vs. 200
#define SYSCTL_RCC2_SYSDIV2_M       0x1F800000 // System Clock Divisor 2
#define SYSCTL_RCC2_SYSDIV2_S       23         // System Clock Divisor 2
#define SYSCTL_RCC2_SYSDIV2LSB_B    0x00400000 // Additional LSB for SYSDIV2
#define SYSCTL_RCC2_USBPWRDN_B      0x00004000 // Power-Down USB PLL
#define SYSCTL_RCC2_PWRDN2_B        0x00002000 // Power-Down PLL 2
#define SYSCTL_RCC2_BYPASS2_B       0x00000800 // PLL Bypass 2
#define SYSCTL_RCC2_OSCSRC2_M       0x00000070 // Oscillator Source 2
#define SYSCTL_RCC2_OSCSRC2_S       4          // Oscillator Source 2
#define SYSCTL_RCC2_OSCSRC2_MOSC_V   0x00000000 // MOSC
#define SYSCTL_RCC2_OSCSRC2_IOSC_V   0x00000010 // PIOSC
#define SYSCTL_RCC2_OSCSRC2_IOSC_4_V 0x00000020 // PIOSC/4
#define SYSCTL_RCC2_OSCSRC2_LFIOSC_V 0x00000030 // LFIOSC
#define SYSCTL_RCC2_OSCSRC2_HIBOSC_V 0x00000070 // 32.768 kHz

/* Bit/Fields in Register MOSCCTL of Module SYSCTL                            */
#define SYSCTL_MOSCCTL_NOXTAL_B     0x00000004 // No Crystal Connected
#define SYSCTL_MOSCCTL_MOSCIM_B     0x00000002 // MOSC Failure Action
#define SYSCTL_MOSCCTL_CVAL_B       0x00000001 // Clock Validation for MOSC

/* Bit/Fields in Register RCGC0 of Module SYSCTL                              */
#define SYSCTL_RCGC0_WDT1_B         0x10000000 // WDT1 Clock Gating Control
#define SYSCTL_RCGC0_CAN1_B         0x02000000 // CAN1 Clock Gating Control
#define SYSCTL_RCGC0_CAN0_B         0x01000000 // CAN0 Clock Gating Control
#define SYSCTL_RCGC0_PWM0_B         0x00100000 // PWM Clock Gating Control
#define SYSCTL_RCGC0_ADC1_B         0x00020000 // ADC1 Clock Gating Control
#define SYSCTL_RCGC0_ADC0_B         0x00010000 // ADC0 Clock Gating Control
#define SYSCTL_RCGC0_ADC1SPD_M      0x00000C00 // ADC1 Sample Speed
#define SYSCTL_RCGC0_ADC1SPD_S      10         // ADC1 Sample Speed
#define SYSCTL_RCGC0_ADC1SPD_1M_V   0x00000C00 // 1M samples/second
#define SYSCTL_RCGC0_ADC0SPD_M      0x00000300 // ADC0 Sample Speed
#define SYSCTL_RCGC0_ADC0SPD_S      8          // ADC0 Sample Speed
#define SYSCTL_RCGC0_ADC0SPD_1M_V   0x00000300 // 1M samples/second
#define SYSCTL_RCGC0_HIB_B          0x00000040 // HIB Clock Gating Control
#define SYSCTL_RCGC0_WDT0_B         0x00000008 // WDT0 Clock Gating Control

/* Bit/Fields in Register RCGC1 of Module SYSCTL                              */
#define SYSCTL_RCGC1_COMP1_B        0x02000000 // Analog Comparator 1 Clock Gating
#define SYSCTL_RCGC1_COMP0_B        0x01000000 // Analog Comparator 0 Clock Gating
#define SYSCTL_RCGC1_TIMER3_B       0x00080000 // Timer 3 Clock Gating Control
#define SYSCTL_RCGC1_TIMER2_B       0x00040000 // Timer 2 Clock Gating Control
#define SYSCTL_RCGC1_TIMER1_B       0x00020000 // Timer 1 Clock Gating Control
#define SYSCTL_RCGC1_TIMER0_B       0x00010000 // Timer 0 Clock Gating Control
#define SYSCTL_RCGC1_I2C1_B         0x00004000 // I2C1 Clock Gating Control
#define SYSCTL_RCGC1_I2C0_B         0x00001000 // I2C0 Clock Gating Control
#define SYSCTL_RCGC1_QEI1_B         0x00000200 // QEI1 Clock Gating Control
#define SYSCTL_RCGC1_QEI0_B         0x00000100 // QEI0 Clock Gating Control
#define SYSCTL_RCGC1_SSI1_B         0x00000020 // SSI1 Clock Gating Control
#define SYSCTL_RCGC1_SSI0_B         0x00000010 // SSI0 Clock Gating Control
#define SYSCTL_RCGC1_UART2_B        0x00000004 // UART2 Clock Gating Control
#define SYSCTL_RCGC1_UART1_B        0x00000002 // UART1 Clock Gating Control
#define SYSCTL_RCGC1_UART0_B        0x00000001 // UART0 Clock Gating Control

/* Bit/Fields in Register RCGC2 of Module SYSCTL                              */
#define SYSCTL_RCGC2_USB0_B         0x00010000 // USB0 Clock Gating Control
#define SYSCTL_RCGC2_UDMA_B         0x00002000 // Micro-DMA Clock Gating Control
#define SYSCTL_RCGC2_GPIOF_B        0x00000020 // Port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOE_B        0x00000010 // Port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOD_B        0x00000008 // Port D Clock Gating Control
#define SYSCTL_RCGC2_GPIOC_B        0x00000004 // Port C Clock Gating Control
#define SYSCTL_RCGC2_GPIOB_B        0x00000002 // Port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOA_B        0x00000001 // Port A Clock Gating Control

/* Bit/Fields in Register SCGC0 of Module SYSCTL                              */
#define SYSCTL_SCGC0_WDT1_B         0x10000000 // WDT1 Clock Gating Control
#define SYSCTL_SCGC0_CAN1_B         0x02000000 // CAN1 Clock Gating Control
#define SYSCTL_SCGC0_CAN0_B         0x01000000 // CAN0 Clock Gating Control
#define SYSCTL_SCGC0_PWM0_B         0x00100000 // PWM Clock Gating Control
#define SYSCTL_SCGC0_ADC1_B         0x00020000 // ADC1 Clock Gating Control
#define SYSCTL_SCGC0_ADC0_B         0x00010000 // ADC0 Clock Gating Control
#define SYSCTL_SCGC0_HIB_B          0x00000040 // HIB Clock Gating Control
#define SYSCTL_SCGC0_WDT0_B         0x00000008 // WDT0 Clock Gating Control

/* Bit/Fields in Register SCGC1 of Module SYSCTL                              */
#define SYSCTL_SCGC1_COMP1_B        0x02000000 // Analog Comparator 1 Clock Gating
#define SYSCTL_SCGC1_COMP0_B        0x01000000 // Analog Comparator 0 Clock Gating
#define SYSCTL_SCGC1_TIMER3_B       0x00080000 // Timer 3 Clock Gating Control
#define SYSCTL_SCGC1_TIMER2_B       0x00040000 // Timer 2 Clock Gating Control
#define SYSCTL_SCGC1_TIMER1_B       0x00020000 // Timer 1 Clock Gating Control
#define SYSCTL_SCGC1_TIMER0_B       0x00010000 // Timer 0 Clock Gating Control
#define SYSCTL_SCGC1_I2C1_B         0x00004000 // I2C1 Clock Gating Control
#define SYSCTL_SCGC1_I2C0_B         0x00001000 // I2C0 Clock Gating Control
#define SYSCTL_SCGC1_QEI1_B         0x00000200 // QEI1 Clock Gating Control
#define SYSCTL_SCGC1_QEI0_B         0x00000100 // QEI0 Clock Gating Control
#define SYSCTL_SCGC1_SSI1_B         0x00000020 // SSI1 Clock Gating Control
#define SYSCTL_SCGC1_SSI0_B         0x00000010 // SSI0 Clock Gating Control
#define SYSCTL_SCGC1_UART2_B        0x00000004 // UART2 Clock Gating Control
#define SYSCTL_SCGC1_UART1_B        0x00000002 // UART1 Clock Gating Control
#define SYSCTL_SCGC1_UART0_B        0x00000001 // UART0 Clock Gating Control

/* Bit/Fields in Register SCGC2 of Module SYSCTL                              */
#define SYSCTL_SCGC2_USB0_B         0x00010000 // USB0 Clock Gating Control
#define SYSCTL_SCGC2_UDMA_B         0x00002000 // Micro-DMA Clock Gating Control
#define SYSCTL_SCGC2_GPIOF_B        0x00000020 // Port F Clock Gating Control
#define SYSCTL_SCGC2_GPIOE_B        0x00000010 // Port E Clock Gating Control
#define SYSCTL_SCGC2_GPIOD_B        0x00000008 // Port D Clock Gating Control
#define SYSCTL_SCGC2_GPIOC_B        0x00000004 // Port C Clock Gating Control
#define SYSCTL_SCGC2_GPIOB_B        0x00000002 // Port B Clock Gating Control
#define SYSCTL_SCGC2_GPIOA_B        0x00000001 // Port A Clock Gating Control

/* Bit/Fields in Register DCGC0 of Module SYSCTL                              */
#define SYSCTL_DCGC0_WDT1_B         0x10000000 // WDT1 Clock Gating Control
#define SYSCTL_DCGC0_CAN1_B         0x02000000 // CAN1 Clock Gating Control
#define SYSCTL_DCGC0_CAN0_B         0x01000000 // CAN0 Clock Gating Control
#define SYSCTL_DCGC0_PWM0_B         0x00100000 // PWM Clock Gating Control
#define SYSCTL_DCGC0_ADC1_B         0x00020000 // ADC1 Clock Gating Control
#define SYSCTL_DCGC0_ADC0_B         0x00010000 // ADC0 Clock Gating Control
#define SYSCTL_DCGC0_HIB_B          0x00000040 // HIB Clock Gating Control
#define SYSCTL_DCGC0_WDT0_B         0x00000008 // WDT0 Clock Gating Control

/* Bit/Fields in Register DCGC1 of Module SYSCTL                              */
#define SYSCTL_DCGC1_COMP1_B        0x02000000 // Analog Comparator 1 Clock Gating
#define SYSCTL_DCGC1_COMP0_B        0x01000000 // Analog Comparator 0 Clock Gating
#define SYSCTL_DCGC1_TIMER3_B       0x00080000 // Timer 3 Clock Gating Control
#define SYSCTL_DCGC1_TIMER2_B       0x00040000 // Timer 2 Clock Gating Control
#define SYSCTL_DCGC1_TIMER1_B       0x00020000 // Timer 1 Clock Gating Control
#define SYSCTL_DCGC1_TIMER0_B       0x00010000 // Timer 0 Clock Gating Control
#define SYSCTL_DCGC1_I2C1_B         0x00004000 // I2C1 Clock Gating Control
#define SYSCTL_DCGC1_I2C0_B         0x00001000 // I2C0 Clock Gating Control
#define SYSCTL_DCGC1_QEI1_B         0x00000200 // QEI1 Clock Gating Control
#define SYSCTL_DCGC1_QEI0_B         0x00000100 // QEI0 Clock Gating Control
#define SYSCTL_DCGC1_SSI1_B         0x00000020 // SSI1 Clock Gating Control
#define SYSCTL_DCGC1_SSI0_B         0x00000010 // SSI0 Clock Gating Control
#define SYSCTL_DCGC1_UART2_B        0x00000004 // UART2 Clock Gating Control
#define SYSCTL_DCGC1_UART1_B        0x00000002 // UART1 Clock Gating Control
#define SYSCTL_DCGC1_UART0_B        0x00000001 // UART0 Clock Gating Control

/* Bit/Fields in Register DCGC2 of Module SYSCTL                              */
#define SYSCTL_DCGC2_USB0_B         0x00010000 // USB0 Clock Gating Control
#define SYSCTL_DCGC2_UDMA_B         0x00002000 // Micro-DMA Clock Gating Control
#define SYSCTL_DCGC2_GPIOF_B        0x00000020 // Port F Clock Gating Control
#define SYSCTL_DCGC2_GPIOE_B        0x00000010 // Port E Clock Gating Control
#define SYSCTL_DCGC2_GPIOD_B        0x00000008 // Port D Clock Gating Control
#define SYSCTL_DCGC2_GPIOC_B        0x00000004 // Port C Clock Gating Control
#define SYSCTL_DCGC2_GPIOB_B        0x00000002 // Port B Clock Gating Control
#define SYSCTL_DCGC2_GPIOA_B        0x00000001 // Port A Clock Gating Control

/* Bit/Fields in Register DSLPCLKCFG of Module SYSCTL                         */
#define SYSCTL_DSLPCLKCFG_D_M       0x1F800000 // Divider Field Override
#define SYSCTL_DSLPCLKCFG_D_S       23         // Divider Field Override
#define SYSCTL_DSLPCLKCFG_O_M       0x00000070 // Clock Source
#define SYSCTL_DSLPCLKCFG_O_S       4          // Clock Source
#define SYSCTL_DSLPCLKCFG_O_IGN_V   0x00000000 // MOSC
#define SYSCTL_DSLPCLKCFG_O_IO_V    0x00000010 // PIOSC
#define SYSCTL_DSLPCLKCFG_O_30_V    0x00000030 // LFIOSC
#define SYSCTL_DSLPCLKCFG_O_32_V    0x00000070 // 32.768 kHz

/* Bit/Fields in Register SYSPROP of Module SYSCTL                            */
#define SYSCTL_SYSPROP_FPU_B        0x00000001 // FPU Present

/* Bit/Fields in Register PIOSCCAL of Module SYSCTL                           */
#define SYSCTL_PIOSCCAL_UTEN_B      0x80000000 // Use User Trim Value
#define SYSCTL_PIOSCCAL_CAL_B       0x00000200 // Start Calibration
#define SYSCTL_PIOSCCAL_UPDATE_B    0x00000100 // Update Trim
#define SYSCTL_PIOSCCAL_UT_M        0x0000007F // User Trim Value
#define SYSCTL_PIOSCCAL_UT_S        0          // User Trim Value

/* Bit/Fields in Register PIOSCSTAT of Module SYSCTL                          */
#define SYSCTL_PIOSCSTAT_DT_M       0x007F0000 // Default Trim Value
#define SYSCTL_PIOSCSTAT_DT_S       16         // Default Trim Value
#define SYSCTL_PIOSCSTAT_CR_M       0x00000300 // Calibration Result
#define SYSCTL_PIOSCSTAT_CR_S       8          // Calibration Result
#define SYSCTL_PIOSCSTAT_CRNONE_V   0x00000000 // Calibration has not been
#define SYSCTL_PIOSCSTAT_CRPASS_V   0x00000100 // The last calibration operation
#define SYSCTL_PIOSCSTAT_CRFAIL_V   0x00000200 // The last calibration operation
#define SYSCTL_PIOSCSTAT_CT_M       0x0000007F // Calibration Trim Value
#define SYSCTL_PIOSCSTAT_CT_S       0          // Calibration Trim Value

/* Bit/Fields in Register PLLFREQ0 of Module SYSCTL                           */
#define SYSCTL_PLLFREQ0_MFRAC_M     0x000FFC00 // PLL M Fractional Value
#define SYSCTL_PLLFREQ0_MFRAC_S     10         // PLL M Fractional Value
#define SYSCTL_PLLFREQ0_MINT_M      0x000003FF // PLL M Integer Value
#define SYSCTL_PLLFREQ0_MINT_S      0          // PLL M Integer Value

/* Bit/Fields in Register PLLFREQ1 of Module SYSCTL                           */
#define SYSCTL_PLLFREQ1_Q_M         0x00001F00 // PLL Q Value
#define SYSCTL_PLLFREQ1_Q_S         8          // PLL Q Value
#define SYSCTL_PLLFREQ1_N_M         0x0000001F // PLL N Value
#define SYSCTL_PLLFREQ1_N_S         0          // PLL N Value

/* Bit/Fields in Register PLLSTAT of Module SYSCTL                            */
#define SYSCTL_PLLSTAT_LOCK_B       0x00000001 // PLL Lock

/* Bit/Fields in Register DC9 of Module SYSCTL                                */
#define SYSCTL_DC9_ADC1DC7_B        0x00800000 // ADC1 DC7 Present
#define SYSCTL_DC9_ADC1DC6_B        0x00400000 // ADC1 DC6 Present
#define SYSCTL_DC9_ADC1DC5_B        0x00200000 // ADC1 DC5 Present
#define SYSCTL_DC9_ADC1DC4_B        0x00100000 // ADC1 DC4 Present
#define SYSCTL_DC9_ADC1DC3_B        0x00080000 // ADC1 DC3 Present
#define SYSCTL_DC9_ADC1DC2_B        0x00040000 // ADC1 DC2 Present
#define SYSCTL_DC9_ADC1DC1_B        0x00020000 // ADC1 DC1 Present
#define SYSCTL_DC9_ADC1DC0_B        0x00010000 // ADC1 DC0 Present
#define SYSCTL_DC9_ADC0DC7_B        0x00000080 // ADC0 DC7 Present
#define SYSCTL_DC9_ADC0DC6_B        0x00000040 // ADC0 DC6 Present
#define SYSCTL_DC9_ADC0DC5_B        0x00000020 // ADC0 DC5 Present
#define SYSCTL_DC9_ADC0DC4_B        0x00000010 // ADC0 DC4 Present
#define SYSCTL_DC9_ADC0DC3_B        0x00000008 // ADC0 DC3 Present
#define SYSCTL_DC9_ADC0DC2_B        0x00000004 // ADC0 DC2 Present
#define SYSCTL_DC9_ADC0DC1_B        0x00000002 // ADC0 DC1 Present
#define SYSCTL_DC9_ADC0DC0_B        0x00000001 // ADC0 DC0 Present

/* Bit/Fields in Register NVMSTAT of Module SYSCTL                            */
#define SYSCTL_NVMSTAT_FWB_B        0x00000001 // 32 Word Flash Write Buffer

/* Bit/Fields in Register LDOSPCTL of Module SYSCTL                           */
#define SYSCTL_LDOSPCTL_VADJEN_B    0x80000000 // Voltage Adjust Enable
#define SYSCTL_LDOSPCTL_VLDO_M      0x000000FF // LDO Output Voltage
#define SYSCTL_LDOSPCTL_VLDO_S      0          // LDO Output Voltage

/* Bit/Fields in Register LDODPCTL of Module SYSCTL                           */
#define SYSCTL_LDODPCTL_VADJEN_B    0x80000000 // Voltage Adjust Enable
#define SYSCTL_LDODPCTL_VLDO_M      0x000000FF // LDO Output Voltage
#define SYSCTL_LDODPCTL_VLDO_S      0          // LDO Output Voltage

/* Bit/Fields in Register PPWD of Module SYSCTL                               */
#define SYSCTL_PPWD_P1_B            0x00000002 // Watchdog Timer 1 Present
#define SYSCTL_PPWD_P0_B            0x00000001 // Watchdog Timer 0 Present

/* Bit/Fields in Register PPTIMER of Module SYSCTL                            */
#define SYSCTL_PPTIMER_P5_B         0x00000020 // 16/32-Bit General-Purpose Timer
#define SYSCTL_PPTIMER_P4_B         0x00000010 // 16/32-Bit General-Purpose Timer
#define SYSCTL_PPTIMER_P3_B         0x00000008 // 16/32-Bit General-Purpose Timer
#define SYSCTL_PPTIMER_P2_B         0x00000004 // 16/32-Bit General-Purpose Timer
#define SYSCTL_PPTIMER_P1_B         0x00000002 // 16/32-Bit General-Purpose Timer
#define SYSCTL_PPTIMER_P0_B         0x00000001 // 16/32-Bit General-Purpose Timer

/* Bit/Fields in Register PPGPIO of Module SYSCTL                             */
#define SYSCTL_PPGPIO_P14_B         0x00004000 // GPIO Port Q Present
#define SYSCTL_PPGPIO_P13_B         0x00002000 // GPIO Port P Present
#define SYSCTL_PPGPIO_P12_B         0x00001000 // GPIO Port N Present
#define SYSCTL_PPGPIO_P11_B         0x00000800 // GPIO Port M Present
#define SYSCTL_PPGPIO_P10_B         0x00000400 // GPIO Port L Present
#define SYSCTL_PPGPIO_P9_B          0x00000200 // GPIO Port K Present
#define SYSCTL_PPGPIO_P8_B          0x00000100 // GPIO Port J Present
#define SYSCTL_PPGPIO_P7_B          0x00000080 // GPIO Port H Present
#define SYSCTL_PPGPIO_P6_B          0x00000040 // GPIO Port G Present
#define SYSCTL_PPGPIO_P5_B          0x00000020 // GPIO Port F Present
#define SYSCTL_PPGPIO_P4_B          0x00000010 // GPIO Port E Present
#define SYSCTL_PPGPIO_P3_B          0x00000008 // GPIO Port D Present
#define SYSCTL_PPGPIO_P2_B          0x00000004 // GPIO Port C Present
#define SYSCTL_PPGPIO_P1_B          0x00000002 // GPIO Port B Present
#define SYSCTL_PPGPIO_P0_B          0x00000001 // GPIO Port A Present

/* Bit/Fields in Register PPDMA of Module SYSCTL                              */
#define SYSCTL_PPDMA_P0_B           0x00000001 // uDMA Module Present

/* Bit/Fields in Register PPHIB of Module SYSCTL                              */
#define SYSCTL_PPHIB_P0_B           0x00000001 // Hibernation Module Present

/* Bit/Fields in Register PPUART of Module SYSCTL                             */
#define SYSCTL_PPUART_P7_B          0x00000080 // UART Module 7 Present
#define SYSCTL_PPUART_P6_B          0x00000040 // UART Module 6 Present
#define SYSCTL_PPUART_P5_B          0x00000020 // UART Module 5 Present
#define SYSCTL_PPUART_P4_B          0x00000010 // UART Module 4 Present
#define SYSCTL_PPUART_P3_B          0x00000008 // UART Module 3 Present
#define SYSCTL_PPUART_P2_B          0x00000004 // UART Module 2 Present
#define SYSCTL_PPUART_P1_B          0x00000002 // UART Module 1 Present
#define SYSCTL_PPUART_P0_B          0x00000001 // UART Module 0 Present

/* Bit/Fields in Register PPSSI of Module SYSCTL                              */
#define SYSCTL_PPSSI_P3_B           0x00000008 // SSI Module 3 Present
#define SYSCTL_PPSSI_P2_B           0x00000004 // SSI Module 2 Present
#define SYSCTL_PPSSI_P1_B           0x00000002 // SSI Module 1 Present
#define SYSCTL_PPSSI_P0_B           0x00000001 // SSI Module 0 Present

/* Bit/Fields in Register PPI2C of Module SYSCTL                              */
#define SYSCTL_PPI2C_P5_B           0x00000020 // I2C Module 5 Present
#define SYSCTL_PPI2C_P4_B           0x00000010 // I2C Module 4 Present
#define SYSCTL_PPI2C_P3_B           0x00000008 // I2C Module 3 Present
#define SYSCTL_PPI2C_P2_B           0x00000004 // I2C Module 2 Present
#define SYSCTL_PPI2C_P1_B           0x00000002 // I2C Module 1 Present
#define SYSCTL_PPI2C_P0_B           0x00000001 // I2C Module 0 Present

/* Bit/Fields in Register PPUSB of Module SYSCTL                              */
#define SYSCTL_PPUSB_P0_B           0x00000001 // USB Module Present

/* Bit/Fields in Register PPCAN of Module SYSCTL                              */
#define SYSCTL_PPCAN_P1_B           0x00000002 // CAN Module 1 Present
#define SYSCTL_PPCAN_P0_B           0x00000001 // CAN Module 0 Present

/* Bit/Fields in Register PPADC of Module SYSCTL                              */
#define SYSCTL_PPADC_P1_B           0x00000002 // ADC Module 1 Present
#define SYSCTL_PPADC_P0_B           0x00000001 // ADC Module 0 Present

/* Bit/Fields in Register PPACMP of Module SYSCTL                             */
#define SYSCTL_PPACMP_P0_B          0x00000001 // Analog Comparator Module Present

/* Bit/Fields in Register PPPWM of Module SYSCTL                              */
#define SYSCTL_PPPWM_P1_B           0x00000002 // PWM Module 1 Present
#define SYSCTL_PPPWM_P0_B           0x00000001 // PWM Module 0 Present

/* Bit/Fields in Register PPQEI of Module SYSCTL                              */
#define SYSCTL_PPQEI_P1_B           0x00000002 // QEI Module 1 Present
#define SYSCTL_PPQEI_P0_B           0x00000001 // QEI Module 0 Present

/* Bit/Fields in Register PPEEPROM of Module SYSCTL                           */
#define SYSCTL_PPEEPROM_P0_B        0x00000001 // EEPROM Module Present

/* Bit/Fields in Register PPWTIMER of Module SYSCTL                           */
#define SYSCTL_PPWTIMER_P5_B        0x00000020 // 32/64-Bit Wide General-Purpose
#define SYSCTL_PPWTIMER_P4_B        0x00000010 // 32/64-Bit Wide General-Purpose
#define SYSCTL_PPWTIMER_P3_B        0x00000008 // 32/64-Bit Wide General-Purpose
#define SYSCTL_PPWTIMER_P2_B        0x00000004 // 32/64-Bit Wide General-Purpose
#define SYSCTL_PPWTIMER_P1_B        0x00000002 // 32/64-Bit Wide General-Purpose
#define SYSCTL_PPWTIMER_P0_B        0x00000001 // 32/64-Bit Wide General-Purpose

/* Bit/Fields in Register SRWD of Module SYSCTL                               */
#define SYSCTL_SRWD_R1_B            0x00000002 // Watchdog Timer 1 Software Reset
#define SYSCTL_SRWD_R0_B            0x00000001 // Watchdog Timer 0 Software Reset

/* Bit/Fields in Register SRTIMER of Module SYSCTL                            */
#define SYSCTL_SRTIMER_R5_B         0x00000020 // 16/32-Bit General-Purpose Timer
#define SYSCTL_SRTIMER_R4_B         0x00000010 // 16/32-Bit General-Purpose Timer
#define SYSCTL_SRTIMER_R3_B         0x00000008 // 16/32-Bit General-Purpose Timer
#define SYSCTL_SRTIMER_R2_B         0x00000004 // 16/32-Bit General-Purpose Timer
#define SYSCTL_SRTIMER_R1_B         0x00000002 // 16/32-Bit General-Purpose Timer
#define SYSCTL_SRTIMER_R0_B         0x00000001 // 16/32-Bit General-Purpose Timer

/* Bit/Fields in Register SRGPIO of Module SYSCTL                             */
#define SYSCTL_SRGPIO_R5_B          0x00000020 // GPIO Port F Software Reset
#define SYSCTL_SRGPIO_R4_B          0x00000010 // GPIO Port E Software Reset
#define SYSCTL_SRGPIO_R3_B          0x00000008 // GPIO Port D Software Reset
#define SYSCTL_SRGPIO_R2_B          0x00000004 // GPIO Port C Software Reset
#define SYSCTL_SRGPIO_R1_B          0x00000002 // GPIO Port B Software Reset
#define SYSCTL_SRGPIO_R0_B          0x00000001 // GPIO Port A Software Reset

/* Bit/Fields in Register SRDMA of Module SYSCTL                              */
#define SYSCTL_SRDMA_R0_B           0x00000001 // uDMA Module Software Reset

/* Bit/Fields in Register SRHIB of Module SYSCTL                              */
#define SYSCTL_SRHIB_R0_B           0x00000001 // Hibernation Module Software

/* Bit/Fields in Register SRUART of Module SYSCTL                             */
#define SYSCTL_SRUART_R7_B          0x00000080 // UART Module 7 Software Reset
#define SYSCTL_SRUART_R6_B          0x00000040 // UART Module 6 Software Reset
#define SYSCTL_SRUART_R5_B          0x00000020 // UART Module 5 Software Reset
#define SYSCTL_SRUART_R4_B          0x00000010 // UART Module 4 Software Reset
#define SYSCTL_SRUART_R3_B          0x00000008 // UART Module 3 Software Reset
#define SYSCTL_SRUART_R2_B          0x00000004 // UART Module 2 Software Reset
#define SYSCTL_SRUART_R1_B          0x00000002 // UART Module 1 Software Reset
#define SYSCTL_SRUART_R0_B          0x00000001 // UART Module 0 Software Reset

/* Bit/Fields in Register SRSSI of Module SYSCTL                              */
#define SYSCTL_SRSSI_R3_B           0x00000008 // SSI Module 3 Software Reset
#define SYSCTL_SRSSI_R2_B           0x00000004 // SSI Module 2 Software Reset
#define SYSCTL_SRSSI_R1_B           0x00000002 // SSI Module 1 Software Reset
#define SYSCTL_SRSSI_R0_B           0x00000001 // SSI Module 0 Software Reset

/* Bit/Fields in Register SRI2C of Module SYSCTL                              */
#define SYSCTL_SRI2C_R3_B           0x00000008 // I2C Module 3 Software Reset
#define SYSCTL_SRI2C_R2_B           0x00000004 // I2C Module 2 Software Reset
#define SYSCTL_SRI2C_R1_B           0x00000002 // I2C Module 1 Software Reset
#define SYSCTL_SRI2C_R0_B           0x00000001 // I2C Module 0 Software Reset

/* Bit/Fields in Register SRUSB of Module SYSCTL                              */
#define SYSCTL_SRUSB_R0_B           0x00000001 // USB Module Software Reset

/* Bit/Fields in Register SRCAN of Module SYSCTL                              */
#define SYSCTL_SRCAN_R1_B           0x00000002 // CAN Module 1 Software Reset
#define SYSCTL_SRCAN_R0_B           0x00000001 // CAN Module 0 Software Reset

/* Bit/Fields in Register SRADC of Module SYSCTL                              */
#define SYSCTL_SRADC_R1_B           0x00000002 // ADC Module 1 Software Reset
#define SYSCTL_SRADC_R0_B           0x00000001 // ADC Module 0 Software Reset

/* Bit/Fields in Register SRACMP of Module SYSCTL                             */
#define SYSCTL_SRACMP_R0_B          0x00000001 // Analog Comparator Module 0

/* Bit/Fields in Register SRPWM of Module SYSCTL                              */
#define SYSCTL_SRPWM_R1_B           0x00000002 // PWM Module 1 Software Reset
#define SYSCTL_SRPWM_R0_B           0x00000001 // PWM Module 0 Software Reset

/* Bit/Fields in Register SRQEI of Module SYSCTL                              */
#define SYSCTL_SRQEI_R1_B           0x00000002 // QEI Module 1 Software Reset
#define SYSCTL_SRQEI_R0_B           0x00000001 // QEI Module 0 Software Reset

/* Bit/Fields in Register SREEPROM of Module SYSCTL                           */
#define SYSCTL_SREEPROM_R0_B        0x00000001 // EEPROM Module Software Reset

/* Bit/Fields in Register SRWTIMER of Module SYSCTL                           */
#define SYSCTL_SRWTIMER_R5_B        0x00000020 // 32/64-Bit Wide General-Purpose
#define SYSCTL_SRWTIMER_R4_B        0x00000010 // 32/64-Bit Wide General-Purpose
#define SYSCTL_SRWTIMER_R3_B        0x00000008 // 32/64-Bit Wide General-Purpose
#define SYSCTL_SRWTIMER_R2_B        0x00000004 // 32/64-Bit Wide General-Purpose
#define SYSCTL_SRWTIMER_R1_B        0x00000002 // 32/64-Bit Wide General-Purpose
#define SYSCTL_SRWTIMER_R0_B        0x00000001 // 32/64-Bit Wide General-Purpose

/* Bit/Fields in Register RCGCWD of Module SYSCTL                             */
#define SYSCTL_RCGCWD_R1_B          0x00000002 // Watchdog Timer 1 Run Mode Clock
#define SYSCTL_RCGCWD_R0_B          0x00000001 // Watchdog Timer 0 Run Mode Clock

/* Bit/Fields in Register RCGCTIMER of Module SYSCTL                          */
#define SYSCTL_RCGCTIMER_R5_B       0x00000020 // 16/32-Bit General-Purpose Timer
#define SYSCTL_RCGCTIMER_R4_B       0x00000010 // 16/32-Bit General-Purpose Timer
#define SYSCTL_RCGCTIMER_R3_B       0x00000008 // 16/32-Bit General-Purpose Timer
#define SYSCTL_RCGCTIMER_R2_B       0x00000004 // 16/32-Bit General-Purpose Timer
#define SYSCTL_RCGCTIMER_R1_B       0x00000002 // 16/32-Bit General-Purpose Timer
#define SYSCTL_RCGCTIMER_R0_B       0x00000001 // 16/32-Bit General-Purpose Timer

/* Bit/Fields in Register RCGCGPIO of Module SYSCTL                           */
#define SYSCTL_RCGCGPIO_R5_B        0x00000020 // GPIO Port F Run Mode Clock
#define SYSCTL_RCGCGPIO_R4_B        0x00000010 // GPIO Port E Run Mode Clock
#define SYSCTL_RCGCGPIO_R3_B        0x00000008 // GPIO Port D Run Mode Clock
#define SYSCTL_RCGCGPIO_R2_B        0x00000004 // GPIO Port C Run Mode Clock
#define SYSCTL_RCGCGPIO_R1_B        0x00000002 // GPIO Port B Run Mode Clock
#define SYSCTL_RCGCGPIO_R0_B        0x00000001 // GPIO Port A Run Mode Clock

/* Bit/Fields in Register RCGCDMA of Module SYSCTL                            */
#define SYSCTL_RCGCDMA_R0_B         0x00000001 // uDMA Module Run Mode Clock

/* Bit/Fields in Register RCGCHIB of Module SYSCTL                            */
#define SYSCTL_RCGCHIB_R0_B         0x00000001 // Hibernation Module Run Mode

/* Bit/Fields in Register RCGCUART of Module SYSCTL                           */
#define SYSCTL_RCGCUART_R7_B        0x00000080 // UART Module 7 Run Mode Clock
#define SYSCTL_RCGCUART_R6_B        0x00000040 // UART Module 6 Run Mode Clock
#define SYSCTL_RCGCUART_R5_B        0x00000020 // UART Module 5 Run Mode Clock
#define SYSCTL_RCGCUART_R4_B        0x00000010 // UART Module 4 Run Mode Clock
#define SYSCTL_RCGCUART_R3_B        0x00000008 // UART Module 3 Run Mode Clock
#define SYSCTL_RCGCUART_R2_B        0x00000004 // UART Module 2 Run Mode Clock
#define SYSCTL_RCGCUART_R1_B        0x00000002 // UART Module 1 Run Mode Clock
#define SYSCTL_RCGCUART_R0_B        0x00000001 // UART Module 0 Run Mode Clock

/* Bit/Fields in Register RCGCSSI of Module SYSCTL                            */
#define SYSCTL_RCGCSSI_R3_B         0x00000008 // SSI Module 3 Run Mode Clock
#define SYSCTL_RCGCSSI_R2_B         0x00000004 // SSI Module 2 Run Mode Clock
#define SYSCTL_RCGCSSI_R1_B         0x00000002 // SSI Module 1 Run Mode Clock
#define SYSCTL_RCGCSSI_R0_B         0x00000001 // SSI Module 0 Run Mode Clock

/* Bit/Fields in Register RCGCI2C of Module SYSCTL                            */
#define SYSCTL_RCGCI2C_R3_B         0x00000008 // I2C Module 3 Run Mode Clock
#define SYSCTL_RCGCI2C_R2_B         0x00000004 // I2C Module 2 Run Mode Clock
#define SYSCTL_RCGCI2C_R1_B         0x00000002 // I2C Module 1 Run Mode Clock
#define SYSCTL_RCGCI2C_R0_B         0x00000001 // I2C Module 0 Run Mode Clock

/* Bit/Fields in Register RCGCUSB of Module SYSCTL                            */
#define SYSCTL_RCGCUSB_R0_B         0x00000001 // USB Module Run Mode Clock Gating

/* Bit/Fields in Register RCGCCAN of Module SYSCTL                            */
#define SYSCTL_RCGCCAN_R1_B         0x00000002 // CAN Module 1 Run Mode Clock
#define SYSCTL_RCGCCAN_R0_B         0x00000001 // CAN Module 0 Run Mode Clock

/* Bit/Fields in Register RCGCADC of Module SYSCTL                            */
#define SYSCTL_RCGCADC_R1_B         0x00000002 // ADC Module 1 Run Mode Clock
#define SYSCTL_RCGCADC_R0_B         0x00000001 // ADC Module 0 Run Mode Clock

/* Bit/Fields in Register RCGCACMP of Module SYSCTL                           */
#define SYSCTL_RCGCACMP_R0_B        0x00000001 // Analog Comparator Module 0 Run

/* Bit/Fields in Register RCGCPWM of Module SYSCTL                            */
#define SYSCTL_RCGCPWM_R1_B         0x00000002 // PWM Module 1 Run Mode Clock
#define SYSCTL_RCGCPWM_R0_B         0x00000001 // PWM Module 0 Run Mode Clock

/* Bit/Fields in Register RCGCQEI of Module SYSCTL                            */
#define SYSCTL_RCGCQEI_R1_B         0x00000002 // QEI Module 1 Run Mode Clock
#define SYSCTL_RCGCQEI_R0_B         0x00000001 // QEI Module 0 Run Mode Clock

/* Bit/Fields in Register RCGCEEPROM of Module SYSCTL                         */
#define SYSCTL_RCGCEEPROM_R0_B      0x00000001 // EEPROM Module Run Mode Clock

/* Bit/Fields in Register RCGCWTIMER of Module SYSCTL                         */
#define SYSCTL_RCGCWTIMER_R5_B      0x00000020 // 32/64-Bit Wide General-Purpose
#define SYSCTL_RCGCWTIMER_R4_B      0x00000010 // 32/64-Bit Wide General-Purpose
#define SYSCTL_RCGCWTIMER_R3_B      0x00000008 // 32/64-Bit Wide General-Purpose
#define SYSCTL_RCGCWTIMER_R2_B      0x00000004 // 32/64-Bit Wide General-Purpose
#define SYSCTL_RCGCWTIMER_R1_B      0x00000002 // 32/64-Bit Wide General-Purpose
#define SYSCTL_RCGCWTIMER_R0_B      0x00000001 // 32/64-Bit Wide General-Purpose

/* Bit/Fields in Register SCGCWD of Module SYSCTL                             */
#define SYSCTL_SCGCWD_S1_B          0x00000002 // Watchdog Timer 1 Sleep Mode
#define SYSCTL_SCGCWD_S0_B          0x00000001 // Watchdog Timer 0 Sleep Mode

/* Bit/Fields in Register SCGCTIMER of Module SYSCTL                          */
#define SYSCTL_SCGCTIMER_S5_B       0x00000020 // 16/32-Bit General-Purpose Timer
#define SYSCTL_SCGCTIMER_S4_B       0x00000010 // 16/32-Bit General-Purpose Timer
#define SYSCTL_SCGCTIMER_S3_B       0x00000008 // 16/32-Bit General-Purpose Timer
#define SYSCTL_SCGCTIMER_S2_B       0x00000004 // 16/32-Bit General-Purpose Timer
#define SYSCTL_SCGCTIMER_S1_B       0x00000002 // 16/32-Bit General-Purpose Timer
#define SYSCTL_SCGCTIMER_S0_B       0x00000001 // 16/32-Bit General-Purpose Timer

/* Bit/Fields in Register SCGCGPIO of Module SYSCTL                           */
#define SYSCTL_SCGCGPIO_S5_B        0x00000020 // GPIO Port F Sleep Mode Clock
#define SYSCTL_SCGCGPIO_S4_B        0x00000010 // GPIO Port E Sleep Mode Clock
#define SYSCTL_SCGCGPIO_S3_B        0x00000008 // GPIO Port D Sleep Mode Clock
#define SYSCTL_SCGCGPIO_S2_B        0x00000004 // GPIO Port C Sleep Mode Clock
#define SYSCTL_SCGCGPIO_S1_B        0x00000002 // GPIO Port B Sleep Mode Clock
#define SYSCTL_SCGCGPIO_S0_B        0x00000001 // GPIO Port A Sleep Mode Clock

/* Bit/Fields in Register SCGCDMA of Module SYSCTL                            */
#define SYSCTL_SCGCDMA_S0_B         0x00000001 // uDMA Module Sleep Mode Clock

/* Bit/Fields in Register SCGCHIB of Module SYSCTL                            */
#define SYSCTL_SCGCHIB_S0_B         0x00000001 // Hibernation Module Sleep Mode

/* Bit/Fields in Register SCGCUART of Module SYSCTL                           */
#define SYSCTL_SCGCUART_S7_B        0x00000080 // UART Module 7 Sleep Mode Clock
#define SYSCTL_SCGCUART_S6_B        0x00000040 // UART Module 6 Sleep Mode Clock
#define SYSCTL_SCGCUART_S5_B        0x00000020 // UART Module 5 Sleep Mode Clock
#define SYSCTL_SCGCUART_S4_B        0x00000010 // UART Module 4 Sleep Mode Clock
#define SYSCTL_SCGCUART_S3_B        0x00000008 // UART Module 3 Sleep Mode Clock
#define SYSCTL_SCGCUART_S2_B        0x00000004 // UART Module 2 Sleep Mode Clock
#define SYSCTL_SCGCUART_S1_B        0x00000002 // UART Module 1 Sleep Mode Clock
#define SYSCTL_SCGCUART_S0_B        0x00000001 // UART Module 0 Sleep Mode Clock

/* Bit/Fields in Register SCGCSSI of Module SYSCTL                            */
#define SYSCTL_SCGCSSI_S3_B         0x00000008 // SSI Module 3 Sleep Mode Clock
#define SYSCTL_SCGCSSI_S2_B         0x00000004 // SSI Module 2 Sleep Mode Clock
#define SYSCTL_SCGCSSI_S1_B         0x00000002 // SSI Module 1 Sleep Mode Clock
#define SYSCTL_SCGCSSI_S0_B         0x00000001 // SSI Module 0 Sleep Mode Clock

/* Bit/Fields in Register SCGCI2C of Module SYSCTL                            */
#define SYSCTL_SCGCI2C_S3_B         0x00000008 // I2C Module 3 Sleep Mode Clock
#define SYSCTL_SCGCI2C_S2_B         0x00000004 // I2C Module 2 Sleep Mode Clock
#define SYSCTL_SCGCI2C_S1_B         0x00000002 // I2C Module 1 Sleep Mode Clock
#define SYSCTL_SCGCI2C_S0_B         0x00000001 // I2C Module 0 Sleep Mode Clock

/* Bit/Fields in Register SCGCUSB of Module SYSCTL                            */
#define SYSCTL_SCGCUSB_S0_B         0x00000001 // USB Module Sleep Mode Clock

/* Bit/Fields in Register SCGCCAN of Module SYSCTL                            */
#define SYSCTL_SCGCCAN_S1_B         0x00000002 // CAN Module 1 Sleep Mode Clock
#define SYSCTL_SCGCCAN_S0_B         0x00000001 // CAN Module 0 Sleep Mode Clock

/* Bit/Fields in Register SCGCADC of Module SYSCTL                            */
#define SYSCTL_SCGCADC_S1_B         0x00000002 // ADC Module 1 Sleep Mode Clock
#define SYSCTL_SCGCADC_S0_B         0x00000001 // ADC Module 0 Sleep Mode Clock

/* Bit/Fields in Register SCGCACMP of Module SYSCTL                           */
#define SYSCTL_SCGCACMP_S0_B        0x00000001 // Analog Comparator Module 0 Sleep

/* Bit/Fields in Register SCGCPWM of Module SYSCTL                            */
#define SYSCTL_SCGCPWM_S1_B         0x00000002 // PWM Module 1 Sleep Mode Clock
#define SYSCTL_SCGCPWM_S0_B         0x00000001 // PWM Module 0 Sleep Mode Clock

/* Bit/Fields in Register SCGCQEI of Module SYSCTL                            */
#define SYSCTL_SCGCQEI_S1_B         0x00000002 // QEI Module 1 Sleep Mode Clock
#define SYSCTL_SCGCQEI_S0_B         0x00000001 // QEI Module 0 Sleep Mode Clock

/* Bit/Fields in Register SCGCEEPROM of Module SYSCTL                         */
#define SYSCTL_SCGCEEPROM_S0_B      0x00000001 // EEPROM Module Sleep Mode Clock

/* Bit/Fields in Register SCGCWTIMER of Module SYSCTL                         */
#define SYSCTL_SCGCWTIMER_S5_B      0x00000020 // 32/64-Bit Wide General-Purpose
#define SYSCTL_SCGCWTIMER_S4_B      0x00000010 // 32/64-Bit Wide General-Purpose
#define SYSCTL_SCGCWTIMER_S3_B      0x00000008 // 32/64-Bit Wide General-Purpose
#define SYSCTL_SCGCWTIMER_S2_B      0x00000004 // 32/64-Bit Wide General-Purpose
#define SYSCTL_SCGCWTIMER_S1_B      0x00000002 // 32/64-Bit Wide General-Purpose
#define SYSCTL_SCGCWTIMER_S0_B      0x00000001 // 32/64-Bit Wide General-Purpose

/* Bit/Fields in Register DCGCWD of Module SYSCTL                             */
#define SYSCTL_DCGCWD_D1_B          0x00000002 // Watchdog Timer 1 Deep-Sleep Mode
#define SYSCTL_DCGCWD_D0_B          0x00000001 // Watchdog Timer 0 Deep-Sleep Mode

/* Bit/Fields in Register DCGCTIMER of Module SYSCTL                          */
#define SYSCTL_DCGCTIMER_D5_B       0x00000020 // 16/32-Bit General-Purpose Timer
#define SYSCTL_DCGCTIMER_D4_B       0x00000010 // 16/32-Bit General-Purpose Timer
#define SYSCTL_DCGCTIMER_D3_B       0x00000008 // 16/32-Bit General-Purpose Timer
#define SYSCTL_DCGCTIMER_D2_B       0x00000004 // 16/32-Bit General-Purpose Timer
#define SYSCTL_DCGCTIMER_D1_B       0x00000002 // 16/32-Bit General-Purpose Timer
#define SYSCTL_DCGCTIMER_D0_B       0x00000001 // 16/32-Bit General-Purpose Timer

/* Bit/Fields in Register DCGCGPIO of Module SYSCTL                           */
#define SYSCTL_DCGCGPIO_D5_B        0x00000020 // GPIO Port F Deep-Sleep Mode
#define SYSCTL_DCGCGPIO_D4_B        0x00000010 // GPIO Port E Deep-Sleep Mode
#define SYSCTL_DCGCGPIO_D3_B        0x00000008 // GPIO Port D Deep-Sleep Mode
#define SYSCTL_DCGCGPIO_D2_B        0x00000004 // GPIO Port C Deep-Sleep Mode
#define SYSCTL_DCGCGPIO_D1_B        0x00000002 // GPIO Port B Deep-Sleep Mode
#define SYSCTL_DCGCGPIO_D0_B        0x00000001 // GPIO Port A Deep-Sleep Mode

/* Bit/Fields in Register DCGCDMA of Module SYSCTL                            */
#define SYSCTL_DCGCDMA_D0_B         0x00000001 // uDMA Module Deep-Sleep Mode

/* Bit/Fields in Register DCGCHIB of Module SYSCTL                            */
#define SYSCTL_DCGCHIB_D0_B         0x00000001 // Hibernation Module Deep-Sleep

/* Bit/Fields in Register DCGCUART of Module SYSCTL                           */
#define SYSCTL_DCGCUART_D7_B        0x00000080 // UART Module 7 Deep-Sleep Mode
#define SYSCTL_DCGCUART_D6_B        0x00000040 // UART Module 6 Deep-Sleep Mode
#define SYSCTL_DCGCUART_D5_B        0x00000020 // UART Module 5 Deep-Sleep Mode
#define SYSCTL_DCGCUART_D4_B        0x00000010 // UART Module 4 Deep-Sleep Mode
#define SYSCTL_DCGCUART_D3_B        0x00000008 // UART Module 3 Deep-Sleep Mode
#define SYSCTL_DCGCUART_D2_B        0x00000004 // UART Module 2 Deep-Sleep Mode
#define SYSCTL_DCGCUART_D1_B        0x00000002 // UART Module 1 Deep-Sleep Mode
#define SYSCTL_DCGCUART_D0_B        0x00000001 // UART Module 0 Deep-Sleep Mode

/* Bit/Fields in Register DCGCSSI of Module SYSCTL                            */
#define SYSCTL_DCGCSSI_D3_B         0x00000008 // SSI Module 3 Deep-Sleep Mode
#define SYSCTL_DCGCSSI_D2_B         0x00000004 // SSI Module 2 Deep-Sleep Mode
#define SYSCTL_DCGCSSI_D1_B         0x00000002 // SSI Module 1 Deep-Sleep Mode
#define SYSCTL_DCGCSSI_D0_B         0x00000001 // SSI Module 0 Deep-Sleep Mode

/* Bit/Fields in Register DCGCI2C of Module SYSCTL                            */
#define SYSCTL_DCGCI2C_D3_B         0x00000008 // I2C Module 3 Deep-Sleep Mode
#define SYSCTL_DCGCI2C_D2_B         0x00000004 // I2C Module 2 Deep-Sleep Mode
#define SYSCTL_DCGCI2C_D1_B         0x00000002 // I2C Module 1 Deep-Sleep Mode
#define SYSCTL_DCGCI2C_D0_B         0x00000001 // I2C Module 0 Deep-Sleep Mode

/* Bit/Fields in Register DCGCUSB of Module SYSCTL                            */
#define SYSCTL_DCGCUSB_D0_B         0x00000001 // USB Module Deep-Sleep Mode Clock

/* Bit/Fields in Register DCGCCAN of Module SYSCTL                            */
#define SYSCTL_DCGCCAN_D1_B         0x00000002 // CAN Module 1 Deep-Sleep Mode
#define SYSCTL_DCGCCAN_D0_B         0x00000001 // CAN Module 0 Deep-Sleep Mode

/* Bit/Fields in Register DCGCADC of Module SYSCTL                            */
#define SYSCTL_DCGCADC_D1_B         0x00000002 // ADC Module 1 Deep-Sleep Mode
#define SYSCTL_DCGCADC_D0_B         0x00000001 // ADC Module 0 Deep-Sleep Mode

/* Bit/Fields in Register DCGCACMP of Module SYSCTL                           */
#define SYSCTL_DCGCACMP_D0_B        0x00000001 // Analog Comparator Module 0

/* Bit/Fields in Register DCGCPWM of Module SYSCTL                            */
#define SYSCTL_DCGCPWM_D1_B         0x00000002 // PWM Module 1 Deep-Sleep Mode
#define SYSCTL_DCGCPWM_D0_B         0x00000001 // PWM Module 0 Deep-Sleep Mode

/* Bit/Fields in Register DCGCQEI of Module SYSCTL                            */
#define SYSCTL_DCGCQEI_D1_B         0x00000002 // QEI Module 1 Deep-Sleep Mode
#define SYSCTL_DCGCQEI_D0_B         0x00000001 // QEI Module 0 Deep-Sleep Mode

/* Bit/Fields in Register DCGCEEPROM of Module SYSCTL                         */
#define SYSCTL_DCGCEEPROM_D0_B      0x00000001 // EEPROM Module Deep-Sleep Mode

/* Bit/Fields in Register DCGCWTIMER of Module SYSCTL                         */
#define SYSCTL_DCGCWTIMER_D5_B      0x00000020 // 32/64-Bit Wide General-Purpose
#define SYSCTL_DCGCWTIMER_D4_B      0x00000010 // 32/64-Bit Wide General-Purpose
#define SYSCTL_DCGCWTIMER_D3_B      0x00000008 // 32/64-Bit Wide General-Purpose
#define SYSCTL_DCGCWTIMER_D2_B      0x00000004 // 32/64-Bit Wide General-Purpose
#define SYSCTL_DCGCWTIMER_D1_B      0x00000002 // 32/64-Bit Wide General-Purpose
#define SYSCTL_DCGCWTIMER_D0_B      0x00000001 // 32/64-Bit Wide General-Purpose

/* Bit/Fields in Register PRWD of Module SYSCTL                               */
#define SYSCTL_PRWD_R1_B            0x00000002 // Watchdog Timer 1 Peripheral
#define SYSCTL_PRWD_R0_B            0x00000001 // Watchdog Timer 0 Peripheral

/* Bit/Fields in Register PRTIMER of Module SYSCTL                            */
#define SYSCTL_PRTIMER_R5_B         0x00000020 // 16/32-Bit General-Purpose Timer
#define SYSCTL_PRTIMER_R4_B         0x00000010 // 16/32-Bit General-Purpose Timer
#define SYSCTL_PRTIMER_R3_B         0x00000008 // 16/32-Bit General-Purpose Timer
#define SYSCTL_PRTIMER_R2_B         0x00000004 // 16/32-Bit General-Purpose Timer
#define SYSCTL_PRTIMER_R1_B         0x00000002 // 16/32-Bit General-Purpose Timer
#define SYSCTL_PRTIMER_R0_B         0x00000001 // 16/32-Bit General-Purpose Timer

/* Bit/Fields in Register PRGPIO of Module SYSCTL                             */
#define SYSCTL_PRGPIO_R5_B          0x00000020 // GPIO Port F Peripheral Ready
#define SYSCTL_PRGPIO_R4_B          0x00000010 // GPIO Port E Peripheral Ready
#define SYSCTL_PRGPIO_R3_B          0x00000008 // GPIO Port D Peripheral Ready
#define SYSCTL_PRGPIO_R2_B          0x00000004 // GPIO Port C Peripheral Ready
#define SYSCTL_PRGPIO_R1_B          0x00000002 // GPIO Port B Peripheral Ready
#define SYSCTL_PRGPIO_R0_B          0x00000001 // GPIO Port A Peripheral Ready

/* Bit/Fields in Register PRDMA of Module SYSCTL                              */
#define SYSCTL_PRDMA_R0_B           0x00000001 // uDMA Module Peripheral Ready

/* Bit/Fields in Register PRHIB of Module SYSCTL                              */
#define SYSCTL_PRHIB_R0_B           0x00000001 // Hibernation Module Peripheral

/* Bit/Fields in Register PRUART of Module SYSCTL                             */
#define SYSCTL_PRUART_R7_B          0x00000080 // UART Module 7 Peripheral Ready
#define SYSCTL_PRUART_R6_B          0x00000040 // UART Module 6 Peripheral Ready
#define SYSCTL_PRUART_R5_B          0x00000020 // UART Module 5 Peripheral Ready
#define SYSCTL_PRUART_R4_B          0x00000010 // UART Module 4 Peripheral Ready
#define SYSCTL_PRUART_R3_B          0x00000008 // UART Module 3 Peripheral Ready
#define SYSCTL_PRUART_R2_B          0x00000004 // UART Module 2 Peripheral Ready
#define SYSCTL_PRUART_R1_B          0x00000002 // UART Module 1 Peripheral Ready
#define SYSCTL_PRUART_R0_B          0x00000001 // UART Module 0 Peripheral Ready

/* Bit/Fields in Register PRSSI of Module SYSCTL                              */
#define SYSCTL_PRSSI_R3_B           0x00000008 // SSI Module 3 Peripheral Ready
#define SYSCTL_PRSSI_R2_B           0x00000004 // SSI Module 2 Peripheral Ready
#define SYSCTL_PRSSI_R1_B           0x00000002 // SSI Module 1 Peripheral Ready
#define SYSCTL_PRSSI_R0_B           0x00000001 // SSI Module 0 Peripheral Ready

/* Bit/Fields in Register PRI2C of Module SYSCTL                              */
#define SYSCTL_PRI2C_R3_B           0x00000008 // I2C Module 3 Peripheral Ready
#define SYSCTL_PRI2C_R2_B           0x00000004 // I2C Module 2 Peripheral Ready
#define SYSCTL_PRI2C_R1_B           0x00000002 // I2C Module 1 Peripheral Ready
#define SYSCTL_PRI2C_R0_B           0x00000001 // I2C Module 0 Peripheral Ready

/* Bit/Fields in Register PRUSB of Module SYSCTL                              */
#define SYSCTL_PRUSB_R0_B           0x00000001 // USB Module Peripheral Ready

/* Bit/Fields in Register PRCAN of Module SYSCTL                              */
#define SYSCTL_PRCAN_R1_B           0x00000002 // CAN Module 1 Peripheral Ready
#define SYSCTL_PRCAN_R0_B           0x00000001 // CAN Module 0 Peripheral Ready

/* Bit/Fields in Register PRADC of Module SYSCTL                              */
#define SYSCTL_PRADC_R1_B           0x00000002 // ADC Module 1 Peripheral Ready
#define SYSCTL_PRADC_R0_B           0x00000001 // ADC Module 0 Peripheral Ready

/* Bit/Fields in Register PRACMP of Module SYSCTL                             */
#define SYSCTL_PRACMP_R0_B          0x00000001 // Analog Comparator Module 0

/* Bit/Fields in Register PRPWM of Module SYSCTL                              */
#define SYSCTL_PRPWM_R1_B           0x00000002 // PWM Module 1 Peripheral Ready
#define SYSCTL_PRPWM_R0_B           0x00000001 // PWM Module 0 Peripheral Ready

/* Bit/Fields in Register PRQEI of Module SYSCTL                              */
#define SYSCTL_PRQEI_R1_B           0x00000002 // QEI Module 1 Peripheral Ready
#define SYSCTL_PRQEI_R0_B           0x00000001 // QEI Module 0 Peripheral Ready

/* Bit/Fields in Register PREEPROM of Module SYSCTL                           */
#define SYSCTL_PREEPROM_R0_B        0x00000001 // EEPROM Module Peripheral Ready

/* Bit/Fields in Register PRWTIMER of Module SYSCTL                           */
#define SYSCTL_PRWTIMER_R5_B        0x00000020 // 32/64-Bit Wide General-Purpose
#define SYSCTL_PRWTIMER_R4_B        0x00000010 // 32/64-Bit Wide General-Purpose
#define SYSCTL_PRWTIMER_R3_B        0x00000008 // 32/64-Bit Wide General-Purpose
#define SYSCTL_PRWTIMER_R2_B        0x00000004 // 32/64-Bit Wide General-Purpose
#define SYSCTL_PRWTIMER_R1_B        0x00000002 // 32/64-Bit Wide General-Purpose
#define SYSCTL_PRWTIMER_R0_B        0x00000001 // 32/64-Bit Wide General-Purpose


/******************************************************************************/
/*                                                                            */
/*                      UDMA                                                  */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register STAT of Module UDMA                                 */
#define UDMA_STAT_DMACHANS_M        0x001F0000 // Available uDMA Channels Minus 1
#define UDMA_STAT_DMACHANS_S        16         // Available uDMA Channels Minus 1
#define UDMA_STAT_STATE_M           0x000000F0 // Control State Machine Status
#define UDMA_STAT_STATE_S           4          // Control State Machine Status
#define UDMA_STAT_STATE_IDLE_V      0x00000000 // Idle
#define UDMA_STAT_STATE_RD_CTRL_V   0x00000010 // Reading channel controller data
#define UDMA_STAT_STATE_WAIT_V      0x00000060 // Waiting for uDMA request to
#define UDMA_STAT_STATE_WR_CTRL_V   0x00000070 // Writing channel controller data
#define UDMA_STAT_STATE_STALL_V     0x00000080 // Stalled
#define UDMA_STAT_STATE_DONE_V      0x00000090 // Done
#define UDMA_STAT_STATE_UNDEF_V     0x000000A0 // Undefined
#define UDMA_STAT_MASTEN_B          0x00000001 // Master Enable Status

/* Bit/Fields in Register CFG of Module UDMA                                  */
#define UDMA_CFG_MASTEN_B           0x00000001 // Controller Master Enable

/* Bit/Fields in Register CTLBASE of Module UDMA                              */
#define UDMA_CTLBASE_ADDR_M         0xFFFFFC00 // Channel Control Base Address
#define UDMA_CTLBASE_ADDR_S         10         // Channel Control Base Address

/* Bit/Fields in Register ALTBASE of Module UDMA                              */
#define UDMA_ALTBASE_ADDR_M         0xFFFFFFFF // Alternate Channel Address
#define UDMA_ALTBASE_ADDR_S         0          // Alternate Channel Address

/* Bit/Fields in Register WAITSTAT of Module UDMA                             */
#define UDMA_WAITSTAT_WAITREQ_M     0xFFFFFFFF // Channel [n] Wait Status
#define UDMA_WAITSTAT_WAITREQ_S     0          // Channel [n] Wait Status

/* Bit/Fields in Register SWREQ of Module UDMA                                */
#define UDMA_SWREQ_M                0xFFFFFFFF // Channel [n] Software Request
#define UDMA_SWREQ_S                0          // Channel [n] Software Request

/* Bit/Fields in Register USEBURSTSET of Module UDMA                          */
#define UDMA_USEBURSTSET_SET_M      0xFFFFFFFF // Channel [n] Useburst Set
#define UDMA_USEBURSTSET_SET_S      0          // Channel [n] Useburst Set

/* Bit/Fields in Register USEBURSTCLR of Module UDMA                          */
#define UDMA_USEBURSTCLR_CLR_M      0xFFFFFFFF // Channel [n] Useburst Clear
#define UDMA_USEBURSTCLR_CLR_S      0          // Channel [n] Useburst Clear

/* Bit/Fields in Register REQMASKSET of Module UDMA                           */
#define UDMA_REQMASKSET_SET_M       0xFFFFFFFF // Channel [n] Request Mask Set
#define UDMA_REQMASKSET_SET_S       0          // Channel [n] Request Mask Set

/* Bit/Fields in Register REQMASKCLR of Module UDMA                           */
#define UDMA_REQMASKCLR_CLR_M       0xFFFFFFFF // Channel [n] Request Mask Clear
#define UDMA_REQMASKCLR_CLR_S       0          // Channel [n] Request Mask Clear

/* Bit/Fields in Register ENASET of Module UDMA                               */
#define UDMA_ENASET_SET_M           0xFFFFFFFF // Channel [n] Enable Set
#define UDMA_ENASET_SET_S           0          // Channel [n] Enable Set

/* Bit/Fields in Register ENACLR of Module UDMA                               */
#define UDMA_ENACLR_CLR_M           0xFFFFFFFF // Clear Channel [n] Enable Clear
#define UDMA_ENACLR_CLR_S           0          // Clear Channel [n] Enable Clear

/* Bit/Fields in Register ALTSET of Module UDMA                               */
#define UDMA_ALTSET_SET_M           0xFFFFFFFF // Channel [n] Alternate Set
#define UDMA_ALTSET_SET_S           0          // Channel [n] Alternate Set

/* Bit/Fields in Register ALTCLR of Module UDMA                               */
#define UDMA_ALTCLR_CLR_M           0xFFFFFFFF // Channel [n] Alternate Clear
#define UDMA_ALTCLR_CLR_S           0          // Channel [n] Alternate Clear

/* Bit/Fields in Register PRIOSET of Module UDMA                              */
#define UDMA_PRIOSET_SET_M          0xFFFFFFFF // Channel [n] Priority Set
#define UDMA_PRIOSET_SET_S          0          // Channel [n] Priority Set

/* Bit/Fields in Register PRIOCLR of Module UDMA                              */
#define UDMA_PRIOCLR_CLR_M          0xFFFFFFFF // Channel [n] Priority Clear
#define UDMA_PRIOCLR_CLR_S          0          // Channel [n] Priority Clear

/* Bit/Fields in Register ERRCLR of Module UDMA                               */
#define UDMA_ERRCLR_ERRCLR_B        0x00000001 // uDMA Bus Error Status

/* Bit/Fields in Register CHASGN of Module UDMA                               */
#define UDMA_CHASGN_M               0xFFFFFFFF // Channel [n] Assignment Select
#define UDMA_CHASGN_S               0          // Channel [n] Assignment Select
#define UDMA_CHASGN_PRIMARY_V       0x00000000 // Use the primary channel
#define UDMA_CHASGN_SECONDARY_V     0x00000001 // Use the secondary channel

/* Bit/Fields in Register CHIS of Module UDMA                                 */
#define UDMA_CHIS_M                 0xFFFFFFFF // Channel [n] Interrupt Status
#define UDMA_CHIS_S                 0          // Channel [n] Interrupt Status

/* Bit/Fields in Register CHMAP0 of Module UDMA                               */
#define UDMA_CHMAP0_CH7SEL_M        0xF0000000 // uDMA Channel 7 Source Select
#define UDMA_CHMAP0_CH7SEL_S        28         // uDMA Channel 7 Source Select
#define UDMA_CHMAP0_CH6SEL_M        0x0F000000 // uDMA Channel 6 Source Select
#define UDMA_CHMAP0_CH6SEL_S        24         // uDMA Channel 6 Source Select
#define UDMA_CHMAP0_CH5SEL_M        0x00F00000 // uDMA Channel 5 Source Select
#define UDMA_CHMAP0_CH5SEL_S        20         // uDMA Channel 5 Source Select
#define UDMA_CHMAP0_CH4SEL_M        0x000F0000 // uDMA Channel 4 Source Select
#define UDMA_CHMAP0_CH4SEL_S        16         // uDMA Channel 4 Source Select
#define UDMA_CHMAP0_CH3SEL_M        0x0000F000 // uDMA Channel 3 Source Select
#define UDMA_CHMAP0_CH3SEL_S        12         // uDMA Channel 3 Source Select
#define UDMA_CHMAP0_CH2SEL_M        0x00000F00 // uDMA Channel 2 Source Select
#define UDMA_CHMAP0_CH2SEL_S        8          // uDMA Channel 2 Source Select
#define UDMA_CHMAP0_CH1SEL_M        0x000000F0 // uDMA Channel 1 Source Select
#define UDMA_CHMAP0_CH1SEL_S        4          // uDMA Channel 1 Source Select
#define UDMA_CHMAP0_CH0SEL_M        0x0000000F // uDMA Channel 0 Source Select
#define UDMA_CHMAP0_CH0SEL_S        0          // uDMA Channel 0 Source Select

/* Bit/Fields in Register CHMAP1 of Module UDMA                               */
#define UDMA_CHMAP1_CH15SEL_M       0xF0000000 // uDMA Channel 15 Source Select
#define UDMA_CHMAP1_CH15SEL_S       28         // uDMA Channel 15 Source Select
#define UDMA_CHMAP1_CH14SEL_M       0x0F000000 // uDMA Channel 14 Source Select
#define UDMA_CHMAP1_CH14SEL_S       24         // uDMA Channel 14 Source Select
#define UDMA_CHMAP1_CH13SEL_M       0x00F00000 // uDMA Channel 13 Source Select
#define UDMA_CHMAP1_CH13SEL_S       20         // uDMA Channel 13 Source Select
#define UDMA_CHMAP1_CH12SEL_M       0x000F0000 // uDMA Channel 12 Source Select
#define UDMA_CHMAP1_CH12SEL_S       16         // uDMA Channel 12 Source Select
#define UDMA_CHMAP1_CH11SEL_M       0x0000F000 // uDMA Channel 11 Source Select
#define UDMA_CHMAP1_CH11SEL_S       12         // uDMA Channel 11 Source Select
#define UDMA_CHMAP1_CH10SEL_M       0x00000F00 // uDMA Channel 10 Source Select
#define UDMA_CHMAP1_CH10SEL_S       8          // uDMA Channel 10 Source Select
#define UDMA_CHMAP1_CH9SEL_M        0x000000F0 // uDMA Channel 9 Source Select
#define UDMA_CHMAP1_CH9SEL_S        4          // uDMA Channel 9 Source Select
#define UDMA_CHMAP1_CH8SEL_M        0x0000000F // uDMA Channel 8 Source Select
#define UDMA_CHMAP1_CH8SEL_S        0          // uDMA Channel 8 Source Select

/* Bit/Fields in Register CHMAP2 of Module UDMA                               */
#define UDMA_CHMAP2_CH23SEL_M       0xF0000000 // uDMA Channel 23 Source Select
#define UDMA_CHMAP2_CH23SEL_S       28         // uDMA Channel 23 Source Select
#define UDMA_CHMAP2_CH22SEL_M       0x0F000000 // uDMA Channel 22 Source Select
#define UDMA_CHMAP2_CH22SEL_S       24         // uDMA Channel 22 Source Select
#define UDMA_CHMAP2_CH21SEL_M       0x00F00000 // uDMA Channel 21 Source Select
#define UDMA_CHMAP2_CH21SEL_S       20         // uDMA Channel 21 Source Select
#define UDMA_CHMAP2_CH20SEL_M       0x000F0000 // uDMA Channel 20 Source Select
#define UDMA_CHMAP2_CH20SEL_S       16         // uDMA Channel 20 Source Select
#define UDMA_CHMAP2_CH19SEL_M       0x0000F000 // uDMA Channel 19 Source Select
#define UDMA_CHMAP2_CH19SEL_S       12         // uDMA Channel 19 Source Select
#define UDMA_CHMAP2_CH18SEL_M       0x00000F00 // uDMA Channel 18 Source Select
#define UDMA_CHMAP2_CH18SEL_S       8          // uDMA Channel 18 Source Select
#define UDMA_CHMAP2_CH17SEL_M       0x000000F0 // uDMA Channel 17 Source Select
#define UDMA_CHMAP2_CH17SEL_S       4          // uDMA Channel 17 Source Select
#define UDMA_CHMAP2_CH16SEL_M       0x0000000F // uDMA Channel 16 Source Select
#define UDMA_CHMAP2_CH16SEL_S       0          // uDMA Channel 16 Source Select

/* Bit/Fields in Register CHMAP3 of Module UDMA                               */
#define UDMA_CHMAP3_CH31SEL_M       0xF0000000 // uDMA Channel 31 Source Select
#define UDMA_CHMAP3_CH31SEL_S       28         // uDMA Channel 31 Source Select
#define UDMA_CHMAP3_CH30SEL_M       0x0F000000 // uDMA Channel 30 Source Select
#define UDMA_CHMAP3_CH30SEL_S       24         // uDMA Channel 30 Source Select
#define UDMA_CHMAP3_CH29SEL_M       0x00F00000 // uDMA Channel 29 Source Select
#define UDMA_CHMAP3_CH29SEL_S       20         // uDMA Channel 29 Source Select
#define UDMA_CHMAP3_CH28SEL_M       0x000F0000 // uDMA Channel 28 Source Select
#define UDMA_CHMAP3_CH28SEL_S       16         // uDMA Channel 28 Source Select
#define UDMA_CHMAP3_CH27SEL_M       0x0000F000 // uDMA Channel 27 Source Select
#define UDMA_CHMAP3_CH27SEL_S       12         // uDMA Channel 27 Source Select
#define UDMA_CHMAP3_CH26SEL_M       0x00000F00 // uDMA Channel 26 Source Select
#define UDMA_CHMAP3_CH26SEL_S       8          // uDMA Channel 26 Source Select
#define UDMA_CHMAP3_CH25SEL_M       0x000000F0 // uDMA Channel 25 Source Select
#define UDMA_CHMAP3_CH25SEL_S       4          // uDMA Channel 25 Source Select
#define UDMA_CHMAP3_CH24SEL_M       0x0000000F // uDMA Channel 24 Source Select
#define UDMA_CHMAP3_CH24SEL_S       0          // uDMA Channel 24 Source Select

/* Bit/Fields in Register SRCENDP of Module UDMA                              */
#define UDMA_SRCENDP_ADDR_M         0xFFFFFFFF // Source Address End Pointer
#define UDMA_SRCENDP_ADDR_S         0          // Source Address End Pointer

/* Bit/Fields in Register DSTENDP of Module UDMA                              */
#define UDMA_DSTENDP_ADDR_M         0xFFFFFFFF // Destination Address End Pointer
#define UDMA_DSTENDP_ADDR_S         0          // Destination Address End Pointer

/* Bit/Fields in Register CHCTL of Module UDMA                                */
#define UDMA_CHCTL_DSTINC_M         0xC0000000 // Destination Address Increment
#define UDMA_CHCTL_DSTINC_S         30         // Destination Address Increment
#define UDMA_CHCTL_DSTINC_8_V       0x00000000 // Byte
#define UDMA_CHCTL_DSTINC_16_V      0x40000000 // Half-word
#define UDMA_CHCTL_DSTINC_32_V      0x80000000 // Word
#define UDMA_CHCTL_DSTINC_NONE_V    0xC0000000 // No increment
#define UDMA_CHCTL_DSTSIZE_M        0x30000000 // Destination Data Size
#define UDMA_CHCTL_DSTSIZE_S        28         // Destination Data Size
#define UDMA_CHCTL_DSTSIZE_8_V      0x00000000 // Byte
#define UDMA_CHCTL_DSTSIZE_16_V     0x10000000 // Half-word
#define UDMA_CHCTL_DSTSIZE_32_V     0x20000000 // Word
#define UDMA_CHCTL_SRCINC_M         0x0C000000 // Source Address Increment
#define UDMA_CHCTL_SRCINC_S         26         // Source Address Increment
#define UDMA_CHCTL_SRCINC_8_V       0x00000000 // Byte
#define UDMA_CHCTL_SRCINC_16_V      0x04000000 // Half-word
#define UDMA_CHCTL_SRCINC_32_V      0x08000000 // Word
#define UDMA_CHCTL_SRCINC_NONE_V    0x0C000000 // No increment
#define UDMA_CHCTL_SRCSIZE_M        0x03000000 // Source Data Size
#define UDMA_CHCTL_SRCSIZE_S        24         // Source Data Size
#define UDMA_CHCTL_SRCSIZE_8_V      0x00000000 // Byte
#define UDMA_CHCTL_SRCSIZE_16_V     0x01000000 // Half-word
#define UDMA_CHCTL_SRCSIZE_32_V     0x02000000 // Word
#define UDMA_CHCTL_ARBSIZE_M        0x0003C000 // Arbitration Size
#define UDMA_CHCTL_ARBSIZE_S        14         // Arbitration Size
#define UDMA_CHCTL_ARBSIZE_1_V      0x00000000 // 1 Transfer
#define UDMA_CHCTL_ARBSIZE_2_V      0x00004000 // 2 Transfers
#define UDMA_CHCTL_ARBSIZE_4_V      0x00008000 // 4 Transfers
#define UDMA_CHCTL_ARBSIZE_8_V      0x0000C000 // 8 Transfers
#define UDMA_CHCTL_ARBSIZE_16_V     0x00010000 // 16 Transfers
#define UDMA_CHCTL_ARBSIZE_32_V     0x00014000 // 32 Transfers
#define UDMA_CHCTL_ARBSIZE_64_V     0x00018000 // 64 Transfers
#define UDMA_CHCTL_ARBSIZE_128_V    0x0001C000 // 128 Transfers
#define UDMA_CHCTL_ARBSIZE_256_V    0x00020000 // 256 Transfers
#define UDMA_CHCTL_ARBSIZE_512_V    0x00024000 // 512 Transfers
#define UDMA_CHCTL_ARBSIZE_1024_V   0x00028000 // 1024 Transfers
#define UDMA_CHCTL_XFERSIZE_M       0x00003FF0 // Transfer Size (minus 1)
#define UDMA_CHCTL_XFERSIZE_S       4          // Transfer Size (minus 1)
#define UDMA_CHCTL_NXTUSEBURST_B    0x00000008 // Next Useburst
#define UDMA_CHCTL_XFERMODE_M       0x00000007 // uDMA Transfer Mode
#define UDMA_CHCTL_XFERMODE_S       0          // uDMA Transfer Mode


/******************************************************************************/
/*                                                                            */
/*                      NVIC                                                  */
/*                                                                            */
/******************************************************************************/


/* Bit/Fields in Register ACTLR of Module NVIC                                */
#define NVIC_ACTLR_DISOOFP_B        0x00000200 // Disable Out-Of-Order Floating
#define NVIC_ACTLR_DISFPCA_B        0x00000100 // Disable CONTROL
#define NVIC_ACTLR_DISFOLD_B        0x00000004 // Disable IT Folding
#define NVIC_ACTLR_DISWBUF_B        0x00000002 // Disable Write Buffer
#define NVIC_ACTLR_DISMCYC_B        0x00000001 // Disable Interrupts of Multiple

/* Bit/Fields in Register ST of Module NVIC                                   */
#define NVIC_ST_CTRL_COUNT_B        0x00010000 // Count Flag
#define NVIC_ST_CTRL_CLK_SRC_B      0x00000004 // Clock Source
#define NVIC_ST_CTRL_INTEN_B        0x00000002 // Interrupt Enable
#define NVIC_ST_CTRL_ENABLE_B       0x00000001 // Enable
#define NVIC_ST_RELOAD_M            0x00FFFFFF // Reload Value
#define NVIC_ST_RELOAD_S            0          // Reload Value
#define NVIC_ST_CURRENT_M           0x00FFFFFF // Current Value
#define NVIC_ST_CURRENT_S           0          // Current Value

/* Bit/Fields in Register EN0 of Module NVIC                                  */
#define NVIC_EN0_INT_M              0xFFFFFFFF // Interrupt Enable
#define NVIC_EN0_INT_S              0          // Interrupt Enable

/* Bit/Fields in Register EN1 of Module NVIC                                  */
#define NVIC_EN1_INT_M              0xFFFFFFFF // Interrupt Enable
#define NVIC_EN1_INT_S              0          // Interrupt Enable

/* Bit/Fields in Register EN2 of Module NVIC                                  */
#define NVIC_EN2_INT_M              0xFFFFFFFF // Interrupt Enable
#define NVIC_EN2_INT_S              0          // Interrupt Enable

/* Bit/Fields in Register EN3 of Module NVIC                                  */
#define NVIC_EN3_INT_M              0xFFFFFFFF // Interrupt Enable
#define NVIC_EN3_INT_S              0          // Interrupt Enable

/* Bit/Fields in Register EN4 of Module NVIC                                  */
#define NVIC_EN4_INT_M              0x000007FF // Interrupt Enable
#define NVIC_EN4_INT_S              0          // Interrupt Enable

/* Bit/Fields in Register DIS0 of Module NVIC                                 */
#define NVIC_DIS0_INT_M             0xFFFFFFFF // Interrupt Disable
#define NVIC_DIS0_INT_S             0          // Interrupt Disable

/* Bit/Fields in Register DIS1 of Module NVIC                                 */
#define NVIC_DIS1_INT_M             0xFFFFFFFF // Interrupt Disable
#define NVIC_DIS1_INT_S             0          // Interrupt Disable

/* Bit/Fields in Register DIS2 of Module NVIC                                 */
#define NVIC_DIS2_INT_M             0xFFFFFFFF // Interrupt Disable
#define NVIC_DIS2_INT_S             0          // Interrupt Disable

/* Bit/Fields in Register DIS3 of Module NVIC                                 */
#define NVIC_DIS3_INT_M             0xFFFFFFFF // Interrupt Disable
#define NVIC_DIS3_INT_S             0          // Interrupt Disable

/* Bit/Fields in Register DIS4 of Module NVIC                                 */
#define NVIC_DIS4_INT_M             0x000007FF // Interrupt Disable
#define NVIC_DIS4_INT_S             0          // Interrupt Disable

/* Bit/Fields in Register PEND0 of Module NVIC                                */
#define NVIC_PEND0_INT_M            0xFFFFFFFF // Interrupt Set Pending
#define NVIC_PEND0_INT_S            0          // Interrupt Set Pending

/* Bit/Fields in Register PEND1 of Module NVIC                                */
#define NVIC_PEND1_INT_M            0xFFFFFFFF // Interrupt Set Pending
#define NVIC_PEND1_INT_S            0          // Interrupt Set Pending

/* Bit/Fields in Register PEND2 of Module NVIC                                */
#define NVIC_PEND2_INT_M            0xFFFFFFFF // Interrupt Set Pending
#define NVIC_PEND2_INT_S            0          // Interrupt Set Pending

/* Bit/Fields in Register PEND3 of Module NVIC                                */
#define NVIC_PEND3_INT_M            0xFFFFFFFF // Interrupt Set Pending
#define NVIC_PEND3_INT_S            0          // Interrupt Set Pending

/* Bit/Fields in Register PEND4 of Module NVIC                                */
#define NVIC_PEND4_INT_M            0x000007FF // Interrupt Set Pending
#define NVIC_PEND4_INT_S            0          // Interrupt Set Pending

/* Bit/Fields in Register UNPEND0 of Module NVIC                              */
#define NVIC_UNPEND0_INT_M          0xFFFFFFFF // Interrupt Clear Pending
#define NVIC_UNPEND0_INT_S          0          // Interrupt Clear Pending

/* Bit/Fields in Register UNPEND1 of Module NVIC                              */
#define NVIC_UNPEND1_INT_M          0xFFFFFFFF // Interrupt Clear Pending
#define NVIC_UNPEND1_INT_S          0          // Interrupt Clear Pending

/* Bit/Fields in Register UNPEND2 of Module NVIC                              */
#define NVIC_UNPEND2_INT_M          0xFFFFFFFF // Interrupt Clear Pending
#define NVIC_UNPEND2_INT_S          0          // Interrupt Clear Pending

/* Bit/Fields in Register UNPEND3 of Module NVIC                              */
#define NVIC_UNPEND3_INT_M          0xFFFFFFFF // Interrupt Clear Pending
#define NVIC_UNPEND3_INT_S          0          // Interrupt Clear Pending

/* Bit/Fields in Register UNPEND4 of Module NVIC                              */
#define NVIC_UNPEND4_INT_M          0x000007FF // Interrupt Clear Pending
#define NVIC_UNPEND4_INT_S          0          // Interrupt Clear Pending

/* Bit/Fields in Register ACTIVE0 of Module NVIC                              */
#define NVIC_ACTIVE0_INT_M          0xFFFFFFFF // Interrupt Active
#define NVIC_ACTIVE0_INT_S          0          // Interrupt Active

/* Bit/Fields in Register ACTIVE1 of Module NVIC                              */
#define NVIC_ACTIVE1_INT_M          0xFFFFFFFF // Interrupt Active
#define NVIC_ACTIVE1_INT_S          0          // Interrupt Active

/* Bit/Fields in Register ACTIVE2 of Module NVIC                              */
#define NVIC_ACTIVE2_INT_M          0xFFFFFFFF // Interrupt Active
#define NVIC_ACTIVE2_INT_S          0          // Interrupt Active

/* Bit/Fields in Register ACTIVE3 of Module NVIC                              */
#define NVIC_ACTIVE3_INT_M          0xFFFFFFFF // Interrupt Active
#define NVIC_ACTIVE3_INT_S          0          // Interrupt Active

/* Bit/Fields in Register ACTIVE4 of Module NVIC                              */
#define NVIC_ACTIVE4_INT_M          0x000007FF // Interrupt Active
#define NVIC_ACTIVE4_INT_S          0          // Interrupt Active

/* Bit/Fields in Register PRI0 of Module NVIC                                 */
#define NVIC_PRI0_INT3_M            0xE0000000 // Interrupt 3 Priority Mask
#define NVIC_PRI0_INT3_S            29         // Interrupt 3 Priority Mask
#define NVIC_PRI0_INT2_M            0x00E00000 // Interrupt 2 Priority Mask
#define NVIC_PRI0_INT2_S            21         // Interrupt 2 Priority Mask
#define NVIC_PRI0_INT1_M            0x0000E000 // Interrupt 1 Priority Mask
#define NVIC_PRI0_INT1_S            13         // Interrupt 1 Priority Mask
#define NVIC_PRI0_INT0_M            0x000000E0 // Interrupt 0 Priority Mask
#define NVIC_PRI0_INT0_S            5          // Interrupt 0 Priority Mask

/* Bit/Fields in Register PRI1 of Module NVIC                                 */
#define NVIC_PRI1_INT7_M            0xE0000000 // Interrupt 7 Priority Mask
#define NVIC_PRI1_INT7_S            29         // Interrupt 7 Priority Mask
#define NVIC_PRI1_INT6_M            0x00E00000 // Interrupt 6 Priority Mask
#define NVIC_PRI1_INT6_S            21         // Interrupt 6 Priority Mask
#define NVIC_PRI1_INT5_M            0x0000E000 // Interrupt 5 Priority Mask
#define NVIC_PRI1_INT5_S            13         // Interrupt 5 Priority Mask
#define NVIC_PRI1_INT4_M            0x000000E0 // Interrupt 4 Priority Mask
#define NVIC_PRI1_INT4_S            5          // Interrupt 4 Priority Mask

/* Bit/Fields in Register PRI2 of Module NVIC                                 */
#define NVIC_PRI2_INT11_M           0xE0000000 // Interrupt 11 Priority Mask
#define NVIC_PRI2_INT11_S           29         // Interrupt 11 Priority Mask
#define NVIC_PRI2_INT10_M           0x00E00000 // Interrupt 10 Priority Mask
#define NVIC_PRI2_INT10_S           21         // Interrupt 10 Priority Mask
#define NVIC_PRI2_INT9_M            0x0000E000 // Interrupt 9 Priority Mask
#define NVIC_PRI2_INT9_S            13         // Interrupt 9 Priority Mask
#define NVIC_PRI2_INT8_M            0x000000E0 // Interrupt 8 Priority Mask
#define NVIC_PRI2_INT8_S            5          // Interrupt 8 Priority Mask

/* Bit/Fields in Register PRI3 of Module NVIC                                 */
#define NVIC_PRI3_INT15_M           0xE0000000 // Interrupt 15 Priority Mask
#define NVIC_PRI3_INT15_S           29         // Interrupt 15 Priority Mask
#define NVIC_PRI3_INT14_M           0x00E00000 // Interrupt 14 Priority Mask
#define NVIC_PRI3_INT14_S           21         // Interrupt 14 Priority Mask
#define NVIC_PRI3_INT13_M           0x0000E000 // Interrupt 13 Priority Mask
#define NVIC_PRI3_INT13_S           13         // Interrupt 13 Priority Mask
#define NVIC_PRI3_INT12_M           0x000000E0 // Interrupt 12 Priority Mask
#define NVIC_PRI3_INT12_S           5          // Interrupt 12 Priority Mask

/* Bit/Fields in Register PRI4 of Module NVIC                                 */
#define NVIC_PRI4_INT19_M           0xE0000000 // Interrupt 19 Priority Mask
#define NVIC_PRI4_INT19_S           29         // Interrupt 19 Priority Mask
#define NVIC_PRI4_INT18_M           0x00E00000 // Interrupt 18 Priority Mask
#define NVIC_PRI4_INT18_S           21         // Interrupt 18 Priority Mask
#define NVIC_PRI4_INT17_M           0x0000E000 // Interrupt 17 Priority Mask
#define NVIC_PRI4_INT17_S           13         // Interrupt 17 Priority Mask
#define NVIC_PRI4_INT16_M           0x000000E0 // Interrupt 16 Priority Mask
#define NVIC_PRI4_INT16_S           5          // Interrupt 16 Priority Mask

/* Bit/Fields in Register PRI5 of Module NVIC                                 */
#define NVIC_PRI5_INT23_M           0xE0000000 // Interrupt 23 Priority Mask
#define NVIC_PRI5_INT23_S           29         // Interrupt 23 Priority Mask
#define NVIC_PRI5_INT22_M           0x00E00000 // Interrupt 22 Priority Mask
#define NVIC_PRI5_INT22_S           21         // Interrupt 22 Priority Mask
#define NVIC_PRI5_INT21_M           0x0000E000 // Interrupt 21 Priority Mask
#define NVIC_PRI5_INT21_S           13         // Interrupt 21 Priority Mask
#define NVIC_PRI5_INT20_M           0x000000E0 // Interrupt 20 Priority Mask
#define NVIC_PRI5_INT20_S           5          // Interrupt 20 Priority Mask

/* Bit/Fields in Register PRI6 of Module NVIC                                 */
#define NVIC_PRI6_INT27_M           0xE0000000 // Interrupt 27 Priority Mask
#define NVIC_PRI6_INT27_S           29         // Interrupt 27 Priority Mask
#define NVIC_PRI6_INT26_M           0x00E00000 // Interrupt 26 Priority Mask
#define NVIC_PRI6_INT26_S           21         // Interrupt 26 Priority Mask
#define NVIC_PRI6_INT25_M           0x0000E000 // Interrupt 25 Priority Mask
#define NVIC_PRI6_INT25_S           13         // Interrupt 25 Priority Mask
#define NVIC_PRI6_INT24_M           0x000000E0 // Interrupt 24 Priority Mask
#define NVIC_PRI6_INT24_S           5          // Interrupt 24 Priority Mask

/* Bit/Fields in Register PRI7 of Module NVIC                                 */
#define NVIC_PRI7_INT31_M           0xE0000000 // Interrupt 31 Priority Mask
#define NVIC_PRI7_INT31_S           29         // Interrupt 31 Priority Mask
#define NVIC_PRI7_INT30_M           0x00E00000 // Interrupt 30 Priority Mask
#define NVIC_PRI7_INT30_S           21         // Interrupt 30 Priority Mask
#define NVIC_PRI7_INT29_M           0x0000E000 // Interrupt 29 Priority Mask
#define NVIC_PRI7_INT29_S           13         // Interrupt 29 Priority Mask
#define NVIC_PRI7_INT28_M           0x000000E0 // Interrupt 28 Priority Mask
#define NVIC_PRI7_INT28_S           5          // Interrupt 28 Priority Mask

/* Bit/Fields in Register PRI8 of Module NVIC                                 */
#define NVIC_PRI8_INT35_M           0xE0000000 // Interrupt 35 Priority Mask
#define NVIC_PRI8_INT35_S           29         // Interrupt 35 Priority Mask
#define NVIC_PRI8_INT34_M           0x00E00000 // Interrupt 34 Priority Mask
#define NVIC_PRI8_INT34_S           21         // Interrupt 34 Priority Mask
#define NVIC_PRI8_INT33_M           0x0000E000 // Interrupt 33 Priority Mask
#define NVIC_PRI8_INT33_S           13         // Interrupt 33 Priority Mask
#define NVIC_PRI8_INT32_M           0x000000E0 // Interrupt 32 Priority Mask
#define NVIC_PRI8_INT32_S           5          // Interrupt 32 Priority Mask

/* Bit/Fields in Register PRI9 of Module NVIC                                 */
#define NVIC_PRI9_INT39_M           0xE0000000 // Interrupt 39 Priority Mask
#define NVIC_PRI9_INT39_S           29         // Interrupt 39 Priority Mask
#define NVIC_PRI9_INT38_M           0x00E00000 // Interrupt 38 Priority Mask
#define NVIC_PRI9_INT38_S           21         // Interrupt 38 Priority Mask
#define NVIC_PRI9_INT37_M           0x0000E000 // Interrupt 37 Priority Mask
#define NVIC_PRI9_INT37_S           13         // Interrupt 37 Priority Mask
#define NVIC_PRI9_INT36_M           0x000000E0 // Interrupt 36 Priority Mask
#define NVIC_PRI9_INT36_S           5          // Interrupt 36 Priority Mask

/* Bit/Fields in Register PRI10 of Module NVIC                                */
#define NVIC_PRI10_INT43_M          0xE0000000 // Interrupt 43 Priority Mask
#define NVIC_PRI10_INT43_S          29         // Interrupt 43 Priority Mask
#define NVIC_PRI10_INT42_M          0x00E00000 // Interrupt 42 Priority Mask
#define NVIC_PRI10_INT42_S          21         // Interrupt 42 Priority Mask
#define NVIC_PRI10_INT41_M          0x0000E000 // Interrupt 41 Priority Mask
#define NVIC_PRI10_INT41_S          13         // Interrupt 41 Priority Mask
#define NVIC_PRI10_INT40_M          0x000000E0 // Interrupt 40 Priority Mask
#define NVIC_PRI10_INT40_S          5          // Interrupt 40 Priority Mask

/* Bit/Fields in Register PRI11 of Module NVIC                                */
#define NVIC_PRI11_INT47_M          0xE0000000 // Interrupt 47 Priority Mask
#define NVIC_PRI11_INT47_S          29         // Interrupt 47 Priority Mask
#define NVIC_PRI11_INT46_M          0x00E00000 // Interrupt 46 Priority Mask
#define NVIC_PRI11_INT46_S          21         // Interrupt 46 Priority Mask
#define NVIC_PRI11_INT45_M          0x0000E000 // Interrupt 45 Priority Mask
#define NVIC_PRI11_INT45_S          13         // Interrupt 45 Priority Mask
#define NVIC_PRI11_INT44_M          0x000000E0 // Interrupt 44 Priority Mask
#define NVIC_PRI11_INT44_S          5          // Interrupt 44 Priority Mask

/* Bit/Fields in Register PRI12 of Module NVIC                                */
#define NVIC_PRI12_INT51_M          0xE0000000 // Interrupt 51 Priority Mask
#define NVIC_PRI12_INT51_S          29         // Interrupt 51 Priority Mask
#define NVIC_PRI12_INT50_M          0x00E00000 // Interrupt 50 Priority Mask
#define NVIC_PRI12_INT50_S          21         // Interrupt 50 Priority Mask
#define NVIC_PRI12_INT49_M          0x0000E000 // Interrupt 49 Priority Mask
#define NVIC_PRI12_INT49_S          13         // Interrupt 49 Priority Mask
#define NVIC_PRI12_INT48_M          0x000000E0 // Interrupt 48 Priority Mask
#define NVIC_PRI12_INT48_S          5          // Interrupt 48 Priority Mask

/* Bit/Fields in Register PRI13 of Module NVIC                                */
#define NVIC_PRI13_INT55_M          0xE0000000 // Interrupt 55 Priority Mask
#define NVIC_PRI13_INT55_S          29         // Interrupt 55 Priority Mask
#define NVIC_PRI13_INT54_M          0x00E00000 // Interrupt 54 Priority Mask
#define NVIC_PRI13_INT54_S          21         // Interrupt 54 Priority Mask
#define NVIC_PRI13_INT53_M          0x0000E000 // Interrupt 53 Priority Mask
#define NVIC_PRI13_INT53_S          13         // Interrupt 53 Priority Mask
#define NVIC_PRI13_INT52_M          0x000000E0 // Interrupt 52 Priority Mask
#define NVIC_PRI13_INT52_S          5          // Interrupt 52 Priority Mask

/* Bit/Fields in Register PRI14 of Module NVIC                                */
#define NVIC_PRI14_INTD_M           0xE0000000 // Interrupt 59 Priority Mask
#define NVIC_PRI14_INTD_S           29         // Interrupt 59 Priority Mask
#define NVIC_PRI14_INTC_M           0x00E00000 // Interrupt 58 Priority Mask
#define NVIC_PRI14_INTC_S           21         // Interrupt 58 Priority Mask
#define NVIC_PRI14_INTB_M           0x0000E000 // Interrupt 57 Priority Mask
#define NVIC_PRI14_INTB_S           13         // Interrupt 57 Priority Mask
#define NVIC_PRI14_INTA_M           0x000000E0 // Interrupt 56 Priority Mask
#define NVIC_PRI14_INTA_S           5          // Interrupt 56 Priority Mask

/* Bit/Fields in Register PRI15 of Module NVIC                                */
#define NVIC_PRI15_INTD_M           0xE0000000 // Interrupt 63 Priority Mask
#define NVIC_PRI15_INTD_S           29         // Interrupt 63 Priority Mask
#define NVIC_PRI15_INTC_M           0x00E00000 // Interrupt 62 Priority Mask
#define NVIC_PRI15_INTC_S           21         // Interrupt 62 Priority Mask
#define NVIC_PRI15_INTB_M           0x0000E000 // Interrupt 61 Priority Mask
#define NVIC_PRI15_INTB_S           13         // Interrupt 61 Priority Mask
#define NVIC_PRI15_INTA_M           0x000000E0 // Interrupt 60 Priority Mask
#define NVIC_PRI15_INTA_S           5          // Interrupt 60 Priority Mask

/* Bit/Fields in Register PRI16 of Module NVIC                                */
#define NVIC_PRI16_INTD_M           0xE0000000 // Interrupt 67 Priority Mask
#define NVIC_PRI16_INTD_S           29         // Interrupt 67 Priority Mask
#define NVIC_PRI16_INTC_M           0x00E00000 // Interrupt 66 Priority Mask
#define NVIC_PRI16_INTC_S           21         // Interrupt 66 Priority Mask
#define NVIC_PRI16_INTB_M           0x0000E000 // Interrupt 65 Priority Mask
#define NVIC_PRI16_INTB_S           13         // Interrupt 65 Priority Mask
#define NVIC_PRI16_INTA_M           0x000000E0 // Interrupt 64 Priority Mask
#define NVIC_PRI16_INTA_S           5          // Interrupt 64 Priority Mask

/* Bit/Fields in Register PRI17 of Module NVIC                                */
#define NVIC_PRI17_INTD_M           0xE0000000 // Interrupt 71 Priority Mask
#define NVIC_PRI17_INTD_S           29         // Interrupt 71 Priority Mask
#define NVIC_PRI17_INTC_M           0x00E00000 // Interrupt 70 Priority Mask
#define NVIC_PRI17_INTC_S           21         // Interrupt 70 Priority Mask
#define NVIC_PRI17_INTB_M           0x0000E000 // Interrupt 69 Priority Mask
#define NVIC_PRI17_INTB_S           13         // Interrupt 69 Priority Mask
#define NVIC_PRI17_INTA_M           0x000000E0 // Interrupt 68 Priority Mask
#define NVIC_PRI17_INTA_S           5          // Interrupt 68 Priority Mask

/* Bit/Fields in Register PRI18 of Module NVIC                                */
#define NVIC_PRI18_INTD_M           0xE0000000 // Interrupt 75 Priority Mask
#define NVIC_PRI18_INTD_S           29         // Interrupt 75 Priority Mask
#define NVIC_PRI18_INTC_M           0x00E00000 // Interrupt 74 Priority Mask
#define NVIC_PRI18_INTC_S           21         // Interrupt 74 Priority Mask
#define NVIC_PRI18_INTB_M           0x0000E000 // Interrupt 73 Priority Mask
#define NVIC_PRI18_INTB_S           13         // Interrupt 73 Priority Mask
#define NVIC_PRI18_INTA_M           0x000000E0 // Interrupt 72 Priority Mask
#define NVIC_PRI18_INTA_S           5          // Interrupt 72 Priority Mask

/* Bit/Fields in Register PRI19 of Module NVIC                                */
#define NVIC_PRI19_INTD_M           0xE0000000 // Interrupt 79 Priority Mask
#define NVIC_PRI19_INTD_S           29         // Interrupt 79 Priority Mask
#define NVIC_PRI19_INTC_M           0x00E00000 // Interrupt 78 Priority Mask
#define NVIC_PRI19_INTC_S           21         // Interrupt 78 Priority Mask
#define NVIC_PRI19_INTB_M           0x0000E000 // Interrupt 77 Priority Mask
#define NVIC_PRI19_INTB_S           13         // Interrupt 77 Priority Mask
#define NVIC_PRI19_INTA_M           0x000000E0 // Interrupt 76 Priority Mask
#define NVIC_PRI19_INTA_S           5          // Interrupt 76 Priority Mask

/* Bit/Fields in Register PRI20 of Module NVIC                                */
#define NVIC_PRI20_INTD_M           0xE0000000 // Interrupt 83 Priority Mask
#define NVIC_PRI20_INTD_S           29         // Interrupt 83 Priority Mask
#define NVIC_PRI20_INTC_M           0x00E00000 // Interrupt 82 Priority Mask
#define NVIC_PRI20_INTC_S           21         // Interrupt 82 Priority Mask
#define NVIC_PRI20_INTB_M           0x0000E000 // Interrupt 81 Priority Mask
#define NVIC_PRI20_INTB_S           13         // Interrupt 81 Priority Mask
#define NVIC_PRI20_INTA_M           0x000000E0 // Interrupt 80 Priority Mask
#define NVIC_PRI20_INTA_S           5          // Interrupt 80 Priority Mask

/* Bit/Fields in Register PRI21 of Module NVIC                                */
#define NVIC_PRI21_INTD_M           0xE0000000 // Interrupt 87 Priority Mask
#define NVIC_PRI21_INTD_S           29         // Interrupt 87 Priority Mask
#define NVIC_PRI21_INTC_M           0x00E00000 // Interrupt 86 Priority Mask
#define NVIC_PRI21_INTC_S           21         // Interrupt 86 Priority Mask
#define NVIC_PRI21_INTB_M           0x0000E000 // Interrupt 85 Priority Mask
#define NVIC_PRI21_INTB_S           13         // Interrupt 85 Priority Mask
#define NVIC_PRI21_INTA_M           0x000000E0 // Interrupt 84 Priority Mask
#define NVIC_PRI21_INTA_S           5          // Interrupt 84 Priority Mask

/* Bit/Fields in Register PRI22 of Module NVIC                                */
#define NVIC_PRI22_INTD_M           0xE0000000 // Interrupt 91 Priority Mask
#define NVIC_PRI22_INTD_S           29         // Interrupt 91 Priority Mask
#define NVIC_PRI22_INTC_M           0x00E00000 // Interrupt 90 Priority Mask
#define NVIC_PRI22_INTC_S           21         // Interrupt 90 Priority Mask
#define NVIC_PRI22_INTB_M           0x0000E000 // Interrupt 89 Priority Mask
#define NVIC_PRI22_INTB_S           13         // Interrupt 89 Priority Mask
#define NVIC_PRI22_INTA_M           0x000000E0 // Interrupt 88 Priority Mask
#define NVIC_PRI22_INTA_S           5          // Interrupt 88 Priority Mask

/* Bit/Fields in Register PRI23 of Module NVIC                                */
#define NVIC_PRI23_INTD_M           0xE0000000 // Interrupt 95 Priority Mask
#define NVIC_PRI23_INTD_S           29         // Interrupt 95 Priority Mask
#define NVIC_PRI23_INTC_M           0x00E00000 // Interrupt 94 Priority Mask
#define NVIC_PRI23_INTC_S           21         // Interrupt 94 Priority Mask
#define NVIC_PRI23_INTB_M           0x0000E000 // Interrupt 93 Priority Mask
#define NVIC_PRI23_INTB_S           13         // Interrupt 93 Priority Mask
#define NVIC_PRI23_INTA_M           0x000000E0 // Interrupt 92 Priority Mask
#define NVIC_PRI23_INTA_S           5          // Interrupt 92 Priority Mask

/* Bit/Fields in Register PRI24 of Module NVIC                                */
#define NVIC_PRI24_INTD_M           0xE0000000 // Interrupt 99 Priority Mask
#define NVIC_PRI24_INTD_S           29         // Interrupt 99 Priority Mask
#define NVIC_PRI24_INTC_M           0x00E00000 // Interrupt 98 Priority Mask
#define NVIC_PRI24_INTC_S           21         // Interrupt 98 Priority Mask
#define NVIC_PRI24_INTB_M           0x0000E000 // Interrupt 97 Priority Mask
#define NVIC_PRI24_INTB_S           13         // Interrupt 97 Priority Mask
#define NVIC_PRI24_INTA_M           0x000000E0 // Interrupt 96 Priority Mask
#define NVIC_PRI24_INTA_S           5          // Interrupt 96 Priority Mask

/* Bit/Fields in Register PRI25 of Module NVIC                                */
#define NVIC_PRI25_INTD_M           0xE0000000 // Interrupt 103 Priority Mask
#define NVIC_PRI25_INTD_S           29         // Interrupt 103 Priority Mask
#define NVIC_PRI25_INTC_M           0x00E00000 // Interrupt 102 Priority Mask
#define NVIC_PRI25_INTC_S           21         // Interrupt 102 Priority Mask
#define NVIC_PRI25_INTB_M           0x0000E000 // Interrupt 101 Priority Mask
#define NVIC_PRI25_INTB_S           13         // Interrupt 101 Priority Mask
#define NVIC_PRI25_INTA_M           0x000000E0 // Interrupt 100 Priority Mask
#define NVIC_PRI25_INTA_S           5          // Interrupt 100 Priority Mask

/* Bit/Fields in Register PRI26 of Module NVIC                                */
#define NVIC_PRI26_INTD_M           0xE0000000 // Interrupt 107 Priority Mask
#define NVIC_PRI26_INTD_S           29         // Interrupt 107 Priority Mask
#define NVIC_PRI26_INTC_M           0x00E00000 // Interrupt 106 Priority Mask
#define NVIC_PRI26_INTC_S           21         // Interrupt 106 Priority Mask
#define NVIC_PRI26_INTB_M           0x0000E000 // Interrupt 105 Priority Mask
#define NVIC_PRI26_INTB_S           13         // Interrupt 105 Priority Mask
#define NVIC_PRI26_INTA_M           0x000000E0 // Interrupt 104 Priority Mask
#define NVIC_PRI26_INTA_S           5          // Interrupt 104 Priority Mask

/* Bit/Fields in Register PRI27 of Module NVIC                                */
#define NVIC_PRI27_INTD_M           0xE0000000 // Interrupt 111 Priority Mask
#define NVIC_PRI27_INTD_S           29         // Interrupt 111 Priority Mask
#define NVIC_PRI27_INTC_M           0x00E00000 // Interrupt 110 Priority Mask
#define NVIC_PRI27_INTC_S           21         // Interrupt 110 Priority Mask
#define NVIC_PRI27_INTB_M           0x0000E000 // Interrupt 109 Priority Mask
#define NVIC_PRI27_INTB_S           13         // Interrupt 109 Priority Mask
#define NVIC_PRI27_INTA_M           0x000000E0 // Interrupt 108 Priority Mask
#define NVIC_PRI27_INTA_S           5          // Interrupt 108 Priority Mask

/* Bit/Fields in Register PRI28 of Module NVIC                                */
#define NVIC_PRI28_INTD_M           0xE0000000 // Interrupt 115 Priority Mask
#define NVIC_PRI28_INTD_S           29         // Interrupt 115 Priority Mask
#define NVIC_PRI28_INTC_M           0x00E00000 // Interrupt 114 Priority Mask
#define NVIC_PRI28_INTC_S           21         // Interrupt 114 Priority Mask
#define NVIC_PRI28_INTB_M           0x0000E000 // Interrupt 113 Priority Mask
#define NVIC_PRI28_INTB_S           13         // Interrupt 113 Priority Mask
#define NVIC_PRI28_INTA_M           0x000000E0 // Interrupt 112 Priority Mask
#define NVIC_PRI28_INTA_S           5          // Interrupt 112 Priority Mask

/* Bit/Fields in Register PRI29 of Module NVIC                                */
#define NVIC_PRI29_INTD_M           0xE0000000 // Interrupt 119 Priority Mask
#define NVIC_PRI29_INTD_S           29         // Interrupt 119 Priority Mask
#define NVIC_PRI29_INTC_M           0x00E00000 // Interrupt 118 Priority Mask
#define NVIC_PRI29_INTC_S           21         // Interrupt 118 Priority Mask
#define NVIC_PRI29_INTB_M           0x0000E000 // Interrupt 117 Priority Mask
#define NVIC_PRI29_INTB_S           13         // Interrupt 117 Priority Mask
#define NVIC_PRI29_INTA_M           0x000000E0 // Interrupt 116 Priority Mask
#define NVIC_PRI29_INTA_S           5          // Interrupt 116 Priority Mask

/* Bit/Fields in Register PRI30 of Module NVIC                                */
#define NVIC_PRI30_INTD_M           0xE0000000 // Interrupt 123 Priority Mask
#define NVIC_PRI30_INTD_S           29         // Interrupt 123 Priority Mask
#define NVIC_PRI30_INTC_M           0x00E00000 // Interrupt 122 Priority Mask
#define NVIC_PRI30_INTC_S           21         // Interrupt 122 Priority Mask
#define NVIC_PRI30_INTB_M           0x0000E000 // Interrupt 121 Priority Mask
#define NVIC_PRI30_INTB_S           13         // Interrupt 121 Priority Mask
#define NVIC_PRI30_INTA_M           0x000000E0 // Interrupt 120 Priority Mask
#define NVIC_PRI30_INTA_S           5          // Interrupt 120 Priority Mask

/* Bit/Fields in Register PRI31 of Module NVIC                                */
#define NVIC_PRI31_INTD_M           0xE0000000 // Interrupt 127 Priority Mask
#define NVIC_PRI31_INTD_S           29         // Interrupt 127 Priority Mask
#define NVIC_PRI31_INTC_M           0x00E00000 // Interrupt 126 Priority Mask
#define NVIC_PRI31_INTC_S           21         // Interrupt 126 Priority Mask
#define NVIC_PRI31_INTB_M           0x0000E000 // Interrupt 125 Priority Mask
#define NVIC_PRI31_INTB_S           13         // Interrupt 125 Priority Mask
#define NVIC_PRI31_INTA_M           0x000000E0 // Interrupt 124 Priority Mask
#define NVIC_PRI31_INTA_S           5          // Interrupt 124 Priority Mask

/* Bit/Fields in Register PRI32 of Module NVIC                                */
#define NVIC_PRI32_INTD_M           0xE0000000 // Interrupt 131 Priority Mask
#define NVIC_PRI32_INTD_S           29         // Interrupt 131 Priority Mask
#define NVIC_PRI32_INTC_M           0x00E00000 // Interrupt 130 Priority Mask
#define NVIC_PRI32_INTC_S           21         // Interrupt 130 Priority Mask
#define NVIC_PRI32_INTB_M           0x0000E000 // Interrupt 129 Priority Mask
#define NVIC_PRI32_INTB_S           13         // Interrupt 129 Priority Mask
#define NVIC_PRI32_INTA_M           0x000000E0 // Interrupt 128 Priority Mask
#define NVIC_PRI32_INTA_S           5          // Interrupt 128 Priority Mask

/* Bit/Fields in Register PRI33 of Module NVIC                                */
#define NVIC_PRI33_INTD_M           0xE0000000 // Interrupt Priority for Interrupt
#define NVIC_PRI33_INTD_S           29         // Interrupt Priority for Interrupt
#define NVIC_PRI33_INTC_M           0x00E00000 // Interrupt Priority for Interrupt
#define NVIC_PRI33_INTC_S           21         // Interrupt Priority for Interrupt
#define NVIC_PRI33_INTB_M           0x0000E000 // Interrupt Priority for Interrupt
#define NVIC_PRI33_INTB_S           13         // Interrupt Priority for Interrupt
#define NVIC_PRI33_INTA_M           0x000000E0 // Interrupt Priority for Interrupt
#define NVIC_PRI33_INTA_S           5          // Interrupt Priority for Interrupt

/* Bit/Fields in Register PRI34 of Module NVIC                                */
#define NVIC_PRI34_INTD_M           0xE0000000 // Interrupt Priority for Interrupt
#define NVIC_PRI34_INTD_S           29         // Interrupt Priority for Interrupt
#define NVIC_PRI34_INTC_M           0x00E00000 // Interrupt Priority for Interrupt
#define NVIC_PRI34_INTC_S           21         // Interrupt Priority for Interrupt
#define NVIC_PRI34_INTB_M           0x0000E000 // Interrupt Priority for Interrupt
#define NVIC_PRI34_INTB_S           13         // Interrupt Priority for Interrupt
#define NVIC_PRI34_INTA_M           0x000000E0 // Interrupt Priority for Interrupt
#define NVIC_PRI34_INTA_S           5          // Interrupt Priority for Interrupt

/* Bit/Fields in Register SWTRIG of Module NVIC                               */
#define NVIC_SWTRIG_INTID_M         0x000000FF // Implementer Code
#define NVIC_SWTRIG_INTID_S         0



//
// SCB
//

/* Bit/Fields in Register CPUID of Module SCB                                 */
#define SCB_CPUID_IMP_M            0xFF000000 // Implementer Code
#define SCB_CPUID_IMP_S            24         // Implementer Code
#define SCB_CPUID_IMP_ARM_V        0x41000000 // ARM
#define SCB_CPUID_VAR_M            0x00F00000 // Variant Number
#define SCB_CPUID_VAR_S            20         // Variant Number
#define SCB_CPUID_CON_M            0x000F0000 // Constant
#define SCB_CPUID_CON_S            16         // Constant
#define SCB_CPUID_PARTNO_M         0x0000FFF0 // Part Number
#define SCB_CPUID_PARTNO_S         4          // Part Number
#define SCB_CPUID_PARTNO_CM4_V     0x0000C240 // Cortex-M4 processor
#define SCB_CPUID_REV_M            0x0000000F // Revision Number
#define SCB_CPUID_REV_S            0          // Revision Number




/* Bit/Fields in Register ICSR of Module SCB                                  */
#define SCB_INT_ICSR_NMI_SET_B     0x80000000 // NMI Set Pending
#define SCB_INT_ICSR_PEND_SV_B     0x10000000 // PendSV Set Pending
#define SCB_INT_ICSR_UNPEND_SV_B   0x08000000 // PendSV Clear Pending
#define SCB_INT_ICSR_PENDSTSET_B   0x04000000 // SysTick Set Pending
#define SCB_INT_ICSR_PENDSTCLR_B   0x02000000 // SysTick Clear Pending
#define SCB_INT_ICSR_ISR_PRE_B     0x00800000 // Debug Interrupt Handling
#define SCB_INT_ICSR_ISR_PEND_B    0x00400000 // Interrupt Pending
#define SCB_INT_ICSR_VEC_PEN_M     0x000FF000 // Interrupt Pending Vector Number
#define SCB_INT_ICSR_VEC_PEN_S     12         // Interrupt Pending Vector Number
#define SCB_INT_ICSR_RET_BASE_B    0x00000800 // Return to Base
#define SCB_INT_ICSR_VEC_ACT_M     0x000000FF // Interrupt Pending Vector Number
#define SCB_INT_ICSR_VEC_ACT_S     0          // Interrupt Pending Vector Number

/* Bit/Fields in Register VTABLE of Module SCB                               */
#define SCB_VTABLE_OFFSET_M        0xFFFFFC00 // Vector Table Offset
#define SCB_VTABLE_OFFSET_S        10         // Vector Table Offset

/* Bit/Fields in Register AIRCR of Module SCB                                */
#define SCB_AIRCR_VECTKEY_M        0xFFFF0000 // Register Key
#define SCB_AIRCR_VECTKEY_S        16         // Register Key
#define SCB_AIRCR_VECTKEY_V        0x05FA0000 // Vector key
#define SCB_AIRCR_ENDIANESS_B      0x00008000 // Data Endianess
#define SCB_AIRCR_PRIGROUP_M       0x00000700 // Interrupt Priority Grouping
#define SCB_AIRCR_PRIGROUP_S       8          // Interrupt Priority Grouping
#define SCB_AIRCR_PRIGROUP_7_1_V   0x00000000 // Priority group 7.1 split
#define SCB_AIRCR_PRIGROUP_6_2_V   0x00000100 // Priority group 6.2 split
#define SCB_AIRCR_PRIGROUP_5_3_V   0x00000200 // Priority group 5.3 split
#define SCB_AIRCR_PRIGROUP_4_4_V   0x00000300 // Priority group 4.4 split
#define SCB_AIRCR_PRIGROUP_3_5_V   0x00000400 // Priority group 3.5 split
#define SCB_AIRCR_PRIGROUP_2_6_V   0x00000500 // Priority group 2.6 split
#define SCB_AIRCR_PRIGROUP_1_7_V   0x00000600 // Priority group 1.7 split
#define SCB_AIRCR_PRIGROUP_0_8_V   0x00000700 // Priority group 0.8 split
#define SCB_AIRCR_SYSRESETREQ_B    0x00000004 // System Reset Request
#define SCB_AIRCR_VECT_CLR_ACT_B   0x00000002 // Clear Active NMI / Fault
#define SCB_AIRCR_VECT_RESET_B     0x00000001 // System Reset

/* Bit/Fields in Register SYSCTRL of Module NVIC                                  */
#define SCB_SCR_SEVONPEND_B   0x00000010 // Wake Up on Pending
#define SCB_SCR_SLEEPDEEP_B   0x00000004 // Deep Sleep Enable
#define SCB_SCR_SLEEPEXIT_B   0x00000002 // Sleep on ISR Exit

/* Bit/Fields in Register CFGCTRL of Module NVIC                                  */
#define SCB_CCR_CTRL_STKALIGN_B    0x00000200 // Stack Alignment on Exception
#define SCB_CCR_CTRL_BFHFNMIGN_B   0x00000100 // Ignore Bus Fault in NMI and
#define SCB_CCR_CTRL_DIV0_B        0x00000010 // Trap on Divide by 0
#define SCB_CCR_CTRL_UNALIGNED_B   0x00000008 // Trap on Unaligned Access
#define SCB_CCR_CTRL_MAIN_PEND_B   0x00000002 // Allow Main Interrupt Trigger
#define SCB_CCR_CTRL_BASE_THR_B    0x00000001 // Thread State Control

/* Bit/Fields in Register SYS of Module SCB                                  */
#define SCB_SHP_PRI1_USAGE_M       0x00E00000 // Usage Fault Priority
#define SCB_SHP_PRI1_USAGE_S       21         // Usage Fault Priority
#define SCB_SHP_PRI1_BUS_M         0x0000E000 // Bus Fault Priority
#define SCB_SHP_PRI1_BUS_S         13         // Bus Fault Priority
#define SCB_SHP_PRI1_MEM_M         0x000000E0 // Memory Management Fault Priority
#define SCB_SHP_PRI1_MEM_S         5          // Memory Management Fault Priority
#define SCB_SHP_PRI2_SVC_M         0xE0000000 // SVCall Priority
#define SCB_SHP_PRI2_SVC_S         29         // SVCall Priority
#define SCB_SHP_PRI3_TICK_M        0xE0000000 // SysTick Exception Priority
#define SCB_SHP_PRI3_TICK_S        29         // SysTick Exception Priority
#define SCB_SHP_PRI3_PENDSV_M      0x00E00000 // PendSV Priority
#define SCB_SHP_PRI3_PENDSV_S      21         // PendSV Priority
#define SCB_SHP_PRI3_DEBUG_M       0x000000E0 // Debug Priority
#define SCB_SHP_PRI3_DEBUG_S       5          // Debug Priority
#define SCB_SHP_HND_CTRL_USAGE_B   0x00040000 // Usage Fault Enable
#define SCB_SHP_HND_CTRL_BUS_B     0x00020000 // Bus Fault Enable
#define SCB_SHP_HND_CTRL_MEM_B     0x00010000 // Memory Management Fault Enable
#define SCB_SHP_HND_CTRL_SVC_B     0x00008000 // SVC Call Pending
#define SCB_SHP_HND_CTRL_BUSP_B    0x00004000 // Bus Fault Pending
#define SCB_SHP_HND_CTRL_MEMP_B    0x00002000 // Memory Management Fault Pending
#define SCB_SHP_HND_CTRL_TICK_B    0x00000800 // SysTick Exception Active
#define SCB_SHP_HND_CTRL_PNDSV_B   0x00000400 // PendSV Exception Active
#define SCB_SHP_HND_CTRL_MON_B     0x00000100 // Debug Monitor Active
#define SCB_SHP_HND_CTRL_SVCA_B    0x00000080 // SVC Call Active
#define SCB_SHP_HND_CTRL_USGA_B    0x00000008 // Usage Fault Active
#define SCB_SHP_HND_CTRL_BUSA_B    0x00000002 // Bus Fault Active
#define SCB_SHP_HND_CTRL_MEMA_B    0x00000001 // Memory Management Fault Active

/* Bit/Fields in Register FAULT of Module SCB                                */
#define SCB_CFSR_STAT_DIV0_B      0x02000000 // Divide-by-Zero Usage Fault
#define SCB_CFSR_STAT_UNALIGN_B   0x01000000 // Unaligned Access Usage Fault
#define SCB_CFSR_STAT_NOCP_B      0x00080000 // No Coprocessor Usage Fault
#define SCB_CFSR_STAT_INVPC_B     0x00040000 // Invalid PC Load Usage Fault
#define SCB_CFSR_STAT_INVSTAT_B   0x00020000 // Invalid State Usage Fault
#define SCB_CFSR_STAT_UNDEF_B     0x00010000 // Undefined Instruction Usage
#define SCB_CFSR_STAT_BFARV_B     0x00008000 // Bus Fault Address Register Valid
#define SCB_CFSR_STAT_BLSPERR_B   0x00002000 // Bus Fault on Floating-Point Lazy
#define SCB_CFSR_STAT_BSTKE_B     0x00001000 // Stack Bus Fault
#define SCB_CFSR_STAT_BUSTKE_B    0x00000800 // Unstack Bus Fault
#define SCB_CFSR_STAT_IMPRE_B     0x00000400 // Imprecise Data Bus Error
#define SCB_CFSR_STAT_PRECISE_B   0x00000200 // Precise Data Bus Error
#define SCB_CFSR_STAT_IBUS_B      0x00000100 // Instruction Bus Error
#define SCB_CFSR_STAT_MMARV_B     0x00000080 // Memory Management Fault Address
#define SCB_CFSR_STAT_MLSPERR_B   0x00000020 // Memory Management Fault on
#define SCB_CFSR_STAT_MSTKE_B     0x00000010 // Stack Access Violation
#define SCB_CFSR_STAT_MUSTKE_B    0x00000008 // Unstack Access Violation
#define SCB_CFSR_STAT_DERR_B      0x00000002 // Data Access Violation
#define SCB_CFSR_STAT_IERR_B      0x00000001 // Instruction Access Violation

/* Bit/Fields in Register HFAULT of Module NVIC                               */
#define SCB_HFSR_STAT_DBG_B      0x80000000 // Debug Event
#define SCB_HFSR_STAT_FORCED_B   0x40000000 // Forced Hard Fault
#define SCB_HFSR_STAT_VECT_B     0x00000002 // Vector Table Read Fault

/* Bit/Fields in Register DEBUG of Module NVIC                                */
#define SCB_DFSR_STAT_EXTRNL_B    0x00000010 // EDBGRQ asserted
#define SCB_DFSR_STAT_VCATCH_B    0x00000008 // Vector catch
#define SCB_DFSR_STAT_DWTTRAP_B   0x00000004 // DWT match
#define SCB_DFSR_STAT_BKPT_B      0x00000002 // Breakpoint instruction
#define SCB_DFSR_STAT_HALTED_B    0x00000001 // Halt request

/* Bit/Fields in Register MM of Module NVIC                                   */
#define NVIC_MM_ADDR_M              0xFFFFFFFF // Fault Address
#define NVIC_MM_ADDR_S              0          // Fault Address

/* Bit/Fields in Register FAULT of Module NVIC                                */
#define NVIC_FAULT_ADDR_M           0xFFFFFFFF // Fault Address
#define NVIC_FAULT_ADDR_S           0          // Fault Address

/* Bit/Fields in Register CPAC of Module NVIC                                 */
#define SCB_CPACR_CP11_M             0x00C00000 // CP11 Coprocessor Access
#define SCB_CPACR_CP11_S             22         // CP11 Coprocessor Access
#define SCB_CPACR_CP11_DIS_V         0x00000000 // Access Denied
#define SCB_CPACR_CP11_PRIV_V        0x00400000 // Privileged Access Only
#define SCB_CPACR_CP11_FULL_V        0x00C00000 // Full Access
#define SCB_CPACR_CP10_M             0x00300000 // CP10 Coprocessor Access
#define SCB_CPACR_CP10_S             20         // CP10 Coprocessor Access
#define SCB_CPACR_CP10_DIS_V         0x00000000 // Access Denied
#define SCB_CPACR_CP10_PRIV_V        0x00100000 // Privileged Access Only
#define SCB_CPACR_CP10_FULL_V        0x00300000 // Full Access

/* Bit/Fields in Register MPU of Module NVIC                                  */
#define NVIC_MPU_TYPE_IREGION_M     0x00FF0000 // Number of I Regions
#define NVIC_MPU_TYPE_IREGION_S     16         // Number of I Regions
#define NVIC_MPU_TYPE_DREGION_M     0x0000FF00 // Number of D Regions
#define NVIC_MPU_TYPE_DREGION_S     8          // Number of D Regions
#define NVIC_MPU_TYPE_SEPARATE_B    0x00000001 // Separate or Unified MPU
#define NVIC_MPU_CTRL_PRIVDEFEN_B   0x00000004 // MPU Default Region
#define NVIC_MPU_CTRL_HFNMIENA_B    0x00000002 // MPU Enabled During Faults
#define NVIC_MPU_CTRL_ENABLE_B      0x00000001 // MPU Enable
#define NVIC_MPU_NUMBER_M           0x00000007 // MPU Region to Access
#define NVIC_MPU_NUMBER_S           0          // MPU Region to Access
#define NVIC_MPU_BASE_ADDR_M        0xFFFFFFE0 // Base Address Mask
#define NVIC_MPU_BASE_ADDR_S        5          // Base Address Mask
#define NVIC_MPU_BASE_VALID_B       0x00000010 // Region Number Valid
#define NVIC_MPU_BASE_REGION_M      0x00000007 // Region Number
#define NVIC_MPU_BASE_REGION_S      0          // Region Number
#define NVIC_MPU_ATTR_XN_B          0x10000000 // Instruction Access Disable
#define NVIC_MPU_ATTR_AP_M          0x07000000 // Access Privilege
#define NVIC_MPU_ATTR_AP_S          24         // Access Privilege
#define NVIC_MPU_ATTR_TEX_M         0x00380000 // Type Extension Mask
#define NVIC_MPU_ATTR_TEX_S         19         // Type Extension Mask
#define NVIC_MPU_ATTR_SHAREABLE_B   0x00040000 // Shareable
#define NVIC_MPU_ATTR_CACHEABLE_B   0x00020000 // Cacheable
#define NVIC_MPU_ATTR_BUFFRABLE_B   0x00010000 // Bufferable
#define NVIC_MPU_ATTR_SRD_M         0x0000FF00 // Subregion Disable Bits
#define NVIC_MPU_ATTR_SRD_S         8          // Subregion Disable Bits
#define NVIC_MPU_ATTR_SIZE_M        0x0000003E // Region Size Mask
#define NVIC_MPU_ATTR_SIZE_S        1          // Region Size Mask
#define NVIC_MPU_ATTR_ENABLE_B      0x00000001 // Region Enable
#define NVIC_MPU_BASE1_ADDR_M       0xFFFFFFE0 // Base Address Mask
#define NVIC_MPU_BASE1_ADDR_S       5          // Base Address Mask
#define NVIC_MPU_BASE1_VALID_B      0x00000010 // Region Number Valid
#define NVIC_MPU_BASE1_REGION_M     0x00000007 // Region Number
#define NVIC_MPU_BASE1_REGION_S     0          // Region Number
#define NVIC_MPU_ATTR1_XN_B         0x10000000 // Instruction Access Disable
#define NVIC_MPU_ATTR1_AP_M         0x07000000 // Access Privilege
#define NVIC_MPU_ATTR1_AP_S         24         // Access Privilege
#define NVIC_MPU_ATTR1_TEX_M        0x00380000 // Type Extension Mask
#define NVIC_MPU_ATTR1_TEX_S        19         // Type Extension Mask
#define NVIC_MPU_ATTR1_SRD_M        0x0000FF00 // Subregion Disable Bits
#define NVIC_MPU_ATTR1_SRD_S        8          // Subregion Disable Bits
#define NVIC_MPU_ATTR1_SIZE_M       0x0000003E // Region Size Mask
#define NVIC_MPU_ATTR1_SIZE_S       1          // Region Size Mask
#define NVIC_MPU_ATTR1_ENABLE_B     0x00000001 // Region Enable
#define NVIC_MPU_BASE2_ADDR_M       0xFFFFFFE0 // Base Address Mask
#define NVIC_MPU_BASE2_ADDR_S       5          // Base Address Mask
#define NVIC_MPU_BASE2_VALID_B      0x00000010 // Region Number Valid
#define NVIC_MPU_BASE2_REGION_M     0x00000007 // Region Number
#define NVIC_MPU_BASE2_REGION_S     0          // Region Number
#define NVIC_MPU_ATTR2_XN_B         0x10000000 // Instruction Access Disable
#define NVIC_MPU_ATTR2_AP_M         0x07000000 // Access Privilege
#define NVIC_MPU_ATTR2_AP_S         24         // Access Privilege
#define NVIC_MPU_ATTR2_TEX_M        0x00380000 // Type Extension Mask
#define NVIC_MPU_ATTR2_TEX_S        19         // Type Extension Mask
#define NVIC_MPU_ATTR2_SRD_M        0x0000FF00 // Subregion Disable Bits
#define NVIC_MPU_ATTR2_SRD_S        8          // Subregion Disable Bits
#define NVIC_MPU_ATTR2_SIZE_M       0x0000003E // Region Size Mask
#define NVIC_MPU_ATTR2_SIZE_S       1          // Region Size Mask
#define NVIC_MPU_ATTR2_ENABLE_B     0x00000001 // Region Enable
#define NVIC_MPU_BASE3_ADDR_M       0xFFFFFFE0 // Base Address Mask
#define NVIC_MPU_BASE3_ADDR_S       5          // Base Address Mask
#define NVIC_MPU_BASE3_VALID_B      0x00000010 // Region Number Valid
#define NVIC_MPU_BASE3_REGION_M     0x00000007 // Region Number
#define NVIC_MPU_BASE3_REGION_S     0          // Region Number
#define NVIC_MPU_ATTR3_XN_B         0x10000000 // Instruction Access Disable
#define NVIC_MPU_ATTR3_AP_M         0x07000000 // Access Privilege
#define NVIC_MPU_ATTR3_AP_S         24         // Access Privilege
#define NVIC_MPU_ATTR3_TEX_M        0x00380000 // Type Extension Mask
#define NVIC_MPU_ATTR3_TEX_S        19         // Type Extension Mask
#define NVIC_MPU_ATTR3_SRD_M        0x0000FF00 // Subregion Disable Bits
#define NVIC_MPU_ATTR3_SRD_S        8          // Subregion Disable Bits
#define NVIC_MPU_ATTR3_SIZE_M       0x0000003E // Region Size Mask
#define NVIC_MPU_ATTR3_SIZE_S       1          // Region Size Mask
#define NVIC_MPU_ATTR3_ENABLE_B     0x00000001 // Region Enable

/* Bit/Fields in Register DBG of Module NVIC                                  */
#define NVIC_DBG_CTRL_DBGKEY_M      0xFFFF0000 // Debug key mask
#define NVIC_DBG_CTRL_DBGKEY_S      16         // Debug key mask
#define NVIC_DBG_CTRL_DBGKEY_V      0xA05F0000 // Debug key
#define NVIC_DBG_CTRL_S_LOCKUP_V    0x00080000 // Core is locked up
#define NVIC_DBG_CTRL_S_SLEEP_V     0x00040000 // Core is sleeping
#define NVIC_DBG_CTRL_S_HALT_V      0x00020000 // Core status on halt
#define NVIC_DBG_CTRL_S_REGRDY_V    0x00010000 // Register read/write available
#define NVIC_DBG_CTRL_C_MASKINT_B   0x00000008 // Mask interrupts when stepping
#define NVIC_DBG_CTRL_C_STEP_B      0x00000004 // Step the core
#define NVIC_DBG_CTRL_C_HALT_B      0x00000002 // Halt the core
#define NVIC_DBG_CTRL_C_DEBUGEN_B   0x00000001 // Enable debug
#define NVIC_DBG_XFER_REG_WNR_B     0x00010000 // Write or not read
#define NVIC_DBG_XFER_REG_SEL_M     0x0000001F // Register
#define NVIC_DBG_XFER_REG_SEL_S     0          // Register
#define NVIC_DBG_XFER_REG_R0_V      0x00000000 // Register R0
#define NVIC_DBG_XFER_REG_R1_V      0x00000001 // Register R1
#define NVIC_DBG_XFER_REG_R2_V      0x00000002 // Register R2
#define NVIC_DBG_XFER_REG_R3_V      0x00000003 // Register R3
#define NVIC_DBG_XFER_REG_R4_V      0x00000004 // Register R4
#define NVIC_DBG_XFER_REG_R5_V      0x00000005 // Register R5
#define NVIC_DBG_XFER_REG_R6_V      0x00000006 // Register R6
#define NVIC_DBG_XFER_REG_R7_V      0x00000007 // Register R7
#define NVIC_DBG_XFER_REG_R8_V      0x00000008 // Register R8
#define NVIC_DBG_XFER_REG_R9_V      0x00000009 // Register R9
#define NVIC_DBG_XFER_REG_R10_V     0x0000000A // Register R10
#define NVIC_DBG_XFER_REG_R11_V     0x0000000B // Register R11
#define NVIC_DBG_XFER_REG_R12_V     0x0000000C // Register R12
#define NVIC_DBG_XFER_REG_R13_V     0x0000000D // Register R13
#define NVIC_DBG_XFER_REG_R14_V     0x0000000E // Register R14
#define NVIC_DBG_XFER_REG_R15_V     0x0000000F // Register R15
#define NVIC_DBG_XFER_REG_FLAGS_V   0x00000010 // xPSR/Flags register
#define NVIC_DBG_XFER_REG_MSP_V     0x00000011 // Main SP
#define NVIC_DBG_XFER_REG_PSP_V     0x00000012 // Process SP
#define NVIC_DBG_XFER_REG_DSP_V     0x00000013 // Deep SP
#define NVIC_DBG_XFER_REG_CFBP_V    0x00000014 // Control/Fault/BasePri/PriMask
#define NVIC_DBG_DATA_M             0xFFFFFFFF // Data temporary cache
#define NVIC_DBG_DATA_S             0          // Data temporary cache
#define NVIC_DBG_INT_HARDERR_V      0x00000400 // Debug trap on hard fault
#define NVIC_DBG_INT_INTERR_V       0x00000200 // Debug trap on interrupt errors
#define NVIC_DBG_INT_BUSERR_V       0x00000100 // Debug trap on bus error
#define NVIC_DBG_INT_STATERR_V      0x00000080 // Debug trap on usage fault state
#define NVIC_DBG_INT_CHKERR_V       0x00000040 // Debug trap on usage fault check
#define NVIC_DBG_INT_NOCPERR_V      0x00000020 // Debug trap on coprocessor error
#define NVIC_DBG_INT_MMERR_V        0x00000010 // Debug trap on mem manage fault
#define NVIC_DBG_INT_RESET_V        0x00000008 // Core reset status
#define NVIC_DBG_INT_RSTPENDCLR_V   0x00000004 // Clear pending core reset
#define NVIC_DBG_INT_RSTPENDING_V   0x00000002 // Core reset is pending
#define NVIC_DBG_INT_RSTVCATCH_V    0x00000001 // Reset vector catch

/* Bit/Fields in Register SW of Module NVIC                                   */
#define NVIC_SW_TRIG_INTID_M        0x000000FF // Interrupt ID
#define NVIC_SW_TRIG_INTID_S        0          // Interrupt ID

/* Bit/Fields in Register FPCC of Module NVIC                                 */
#define NVIC_FPCC_ASPEN_B           0x80000000 // Automatic State Preservation
#define NVIC_FPCC_LSPEN_B           0x40000000 // Lazy State Preservation Enable
#define NVIC_FPCC_MONRDY_B          0x00000100 // Monitor Ready
#define NVIC_FPCC_BFRDY_B           0x00000040 // Bus Fault Ready
#define NVIC_FPCC_MMRDY_B           0x00000020 // Memory Management Fault Ready
#define NVIC_FPCC_HFRDY_B           0x00000010 // Hard Fault Ready
#define NVIC_FPCC_THREAD_B          0x00000008 // Thread Mode
#define NVIC_FPCC_USER_B            0x00000002 // User Privilege Level
#define NVIC_FPCC_LSPACT_B          0x00000001 // Lazy State Preservation Active

/* Bit/Fields in Register FPCA of Module NVIC                                 */
#define NVIC_FPCA_ADDRESS_M         0xFFFFFFF8 // Address
#define NVIC_FPCA_ADDRESS_S         3          // Address

/* Bit/Fields in Register FPDSC of Module NVIC                                */
#define NVIC_FPDSC_AHP_B            0x04000000 // AHP Bit Default
#define NVIC_FPDSC_DN_B             0x02000000 // DN Bit Default
#define NVIC_FPDSC_FZ_B             0x01000000 // FZ Bit Default
#define NVIC_FPDSC_RMODE_M          0x00C00000 // RMODE Bit Default
#define NVIC_FPDSC_RMODE_S          22         // RMODE Bit Default
#define NVIC_FPDSC_RMODE_RN_V       0x00000000 // Round to Nearest (RN) mode
#define NVIC_FPDSC_RMODE_RP_V       0x00400000 // Round towards Plus Infinity (RP)
#define NVIC_FPDSC_RMODE_RM_V       0x00800000 // Round towards Minus Infinity
#define NVIC_FPDSC_RMODE_RZ_V       0x00C00000 // Round towards Zero (RZ) mode

#endif // TM4C123GH6PM_FIELDS
