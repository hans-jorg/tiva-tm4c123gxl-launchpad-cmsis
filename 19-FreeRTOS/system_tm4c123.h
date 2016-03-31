#ifndef _SYSTEM_TM4C123_H
#define _SYSTEM_TM4C123_H
/**
 * @file     system_tm4c123.h
 * @brief    Routines and variables defined in system_tm4c123
 * @version  V1.0
 * @date     23/01/2016
 *
 **/

/*
 *  According CMSIS, a project must have:
 *    A device-specific system configuration function, SystemInit().
 *    A global variable that contains the system frequency, SystemCoreClock.
 *    A function to update the variable above.
 *
 *  The file configures the device and, typically, initializes the oscillator
 *    (PLL) that is part of the microcontroller device.
 *  This file might export other functions or variables that provide a more
 *    flexible configuration of the microcontroller system.
 *
 *  NOTE
 *  The files Startup File startup_<device>.s and System Configuration Files
 *    system_<device>.c and system_<device>.h may require application specific
 *    adaptations and therefore should be copied into the application project
 *    folder prior configuration. The Device Header File <device.h> is
 *    included in all source files that need device access and can be
 *    stored on a central include folder that is generic for all projects.
 */

/**
 *
 * @brief Mandated by CMSIS
 **/

//@{
// Variable to hold the system core clock value
extern uint32_t SystemCoreClock;
//Function to update the variable SystemCoreClock
extern void SystemCoreClockUpdate(void);
// Function to Initialize the system.
extern void SystemInit(void);
//@}

/**
 * @brief Clock sources
 *
 **/
//@{
#define CLOCK_SRC_MOSC                    1   /* external crystal or oscillator */
#define CLOCK_SRC_PIOSC                   2   /* 16 MHz   */
#define CLOCK_SRC_PIOSC_4                 3   /* 4 MHz    */
#define CLOCK_SRC_LFIOSC                  4   /* 33 KHz   */
#define CLOCK_SRC_HIBOSC                  5   /* 32768 Hz */
#define CLOCK_SRC_PLL_MOSC                6   /* 400 MHz  */
#define CLOCK_SRC_PLL_2_MOSC              7   /* 200 MHz  */
#define CLOCK_SRC_PLL_PIOSC               8   /* 400 MHz  */
#define CLOCK_SRC_PLL_2_PIOSC             9   /* 200 MHz  */
//@}

/**
 * @brief Crystal values
 *
 * @note  See Description on register RCC)
 * @note  Use only values in the description
 **/
#define XTAL_4MHZ                (4000000UL)    // 4 MHz
#define XTAL_4_09MHZ             (4096000UL)    // 4.096 MHz
#define XTAL_4_91MHZ             (4915200UL)    // 4.9152 MHz
#define XTAL_5MHZ                (5000000UL)    // 5 MHz        (USB)
#define XTAL_5_12MHZ             (5120000UL)    // 5.12 MHz
#define XTAL_6MHZ                (6000000UL)    // 6 MHz        (USB)
#define XTAL_6_14MHZ             (6144000UL)    // 6.144 MHz
#define XTAL_7_37MHZ             (7372800UL)    // 7.3728 MHz
#define XTAL_8MHZ                (8000000UL)    // 8 MHz        (USB)
#define XTAL_8_19MHZ             (8129200UL)    // 8.192 MHz
#define XTAL_10MHZ              (10000000UL)    // 10 MHz       (USB)
#define XTAL_12MHZ              (12000000UL)    // 12 MHz       (USB)
#define XTAL_12_2MHZ            (12288000UL)    // 12.288 MHz
#define XTAL_13_5MHZ            (13560000UL)    // 13.56 MHz
#define XTAL_14_3MHZ            (14318180UL)    // 14.31818 MHz
#define XTAL_16MHZ              (16000000UL)    // 16 MHz       (USB)
#define XTAL_16_3MHZ            (16384000UL)    // 16.384 MHz
#define XTAL_18MHZ              (18000000UL)    // 18.0 MHz     (USB)
#define XTAL_20MHZ              (20000000UL)    // 20.0 MHz     (USB)
#define XTAL_24MHZ              (24000000UL)    // 24.0 MHz     (USB)
#define XTAL_25MHZ              (25000000UL)    // 25.0 MHz     (USB)

/**
 * @brief Crystal in Launchpad Board
 *
 * @note  Can be defined in compiler parameter, e.g.,  -DXTAL_FREQ=4000000UL
 **/
#ifndef XTAL_FREQ
#define XTAL_FREQ              (XTAL_16MHZ)
#endif

/**
 * @brief Clock frequencies of diferent sources
 *
 **/
//@{
#define CLOCK_FREQ_MOSC                (XTAL_FREQ) /* External Main Oscillator Frequency: Crystal       */
#define CLOCK_FREQ_PIOSC              (16000000UL) /* Internal Oscillator Frequency: 16 MHz             */
#define CLOCK_FREQ_PIOSC_4             (4000000UL) /* PIOSC divided by 4 : 4 MHz                        */
#define CLOCK_FREQ_LFIOSC                (30000UL) /* Low Frequency Internal Oscillator: 30 KHz         */
#define CLOCK_FREQ_HIBOSC                (32768UL) /* External Hibernate Module Oscillator: 32768 Hz    */
#define CLOCK_FREQ_PLLOSC            (400000000UL) /* PLL Oscillator : 400 MHz                          */
#define CLOCK_FREQ_PLLOSC_2          (200000000UL) /* PLL Oscillator divided by 2 : 200 MHz             */
#define CLOCK_FREQ_MAX                (80000000UL) /* Maximum processo clock : 80 MHz                   */
//@}

/**
 *
 * @brief Extensions to CMSIS
 *
 */
//@{
int SystemCoreClockSet(uint8_t source, uint32_t div);
uint32_t SystemCoreClockGet(void);
int SystemCoreClockSet(uint8_t source, uint32_t div);
//@}


#endif
