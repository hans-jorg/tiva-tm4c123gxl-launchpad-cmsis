/**
 * @file     system_tm4c123.c
 * @brief    Initialization of device and/or board
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note
 * The frequency of the system clock can be up to 80 MHz.
 *
 * The system clock can be configured to use many sources. See Section 5.2.5
 * of datasheet for more information.
 * The sources for the sytem clock can be:
 *   Main Oscillator (MOSC)
 *           It can use an internal oscillator with an external crystal or
 *           an external oscillator. The frequency can be between 4 and 25 MHz.
 *           When USB or PLL is used only certain frequencies are allowed.
 *           It is the only source for USB module!
 *   Precision Internal Oscillator (PIOSC)
 *           A factory callibrated 16 MHz RC oscillator. It has 3% precision.
 *           It varies with voltage and temperature. When special calibration
 *           the precisin can be as low as 1%.
 *   Precision Internal Oscillator divided by 4(PIOSC_4)
 *           Use the PIOSC signal but divides it. So it is a 4 MHz signal.
 *   Low Frequency Internal Oscillator (LFIOSC).
 *           Uses an internal RC oscillator with a 33 KHz nominal frequency,
 *           but can vary between 10 and 90 KHz!!
 *   Hibernation Oscillator (HIBOSC)
 *           Uses an external 32768 Hz crystal.
 *   Phase Locked Loop Oscillator (PLL)
 *           This oscillator is configured to generate a 400 MHz clock signal.
 *           There are two dividers used to lower this frequency to allowed
 *           values. A divide-by-two circuit generates a 200 MHz signal.
 *           To enable PLL the BYPASS bit must be set to zero. To work
 *           properly, the value of crystal frequency must be set using
 *           field XTAL in RCC.
 *
 *
 *   System Divider
 *           The SYSDIV divider can use the 400 MHz, the 200 MHz (as
 *           configured by the DIV400 bit) or any of the above sources (in
 *           this case, BYPASS must be 1) to generate the desired system
 *           clock. It can be bypassed too when USESYSDIV is set to 0.
 *           There are actually two ways to set SYSDIV. One with the
 *           SYSDIV field in RCC register. It has 4 bits and can set the
 *           divider to a value between 1 to 16. The other way is to use
 *           the SYSDIV2 field in RCC2. It is 6 bits wide and can set the
 *           divider to a value between 1 and 64.
 *
 *  Care must be taken not to set the system clock to an invalid value!
 *
 *  To use 80 MHz, SYSDIV must be to 5 and DIV400 to 1.
 *  To use 40 MHz, SYSDIV can be 10 with DIV400 set to 1 or
 *      5 with DIV400 set to 0.
 *
 *  PLL takes time to set. The bit LOCK in PLLSTAT register must be
 *  tested before is this signal is used to generate the system clock.
 *
 */

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "system_tm4c123.h"
#include "tm4c123gh6pm-fields.h"

/**
 * * @brief Variable to hold current clock frequency
 */
uint32_t SystemCoreClock = CLOCK_FREQ_PIOSC; // Default = PIOSC

/**
 * @brief Crystal table
 *
 * @note See Description of RCC Register
 */
static uint32_t crystaltable[] = {
/*  0 */        0,
/*  1 */        0,
/*  2 */        0,
/*  3 */        0,
/*  4 */        0,
/*  5 */        0,
/*  6 */  4000000,
/*  7 */  4096000,
/*  8 */  4915200,
/*  9 */  5000000,
/* 10 */  5120000,
/* 11 */  6000000,
/* 12 */  6144000,
/* 13 */  7372800,
/* 14 */  8000000,
/* 15 */  8192000,
/* 16 */ 10000000,
/* 17 */ 12000000,
/* 18 */ 12288000,
/* 19 */ 13560000,
/* 20 */ 13560000,
/* 21 */ 14318180,
/* 22 */ 16000000,
/* 23 */ 16384000,
/* 24 */ 18000000,
/* 25 */ 20000000,
/* 26 */ 25000000,
/* 27 */        0,
/* 28 */        0,
/* 29 */        0,
/* 30 */        0,
/* 31 */        0,
};

static const int crystaltablesize=sizeof(crystaltable)/sizeof(uint32_t);

static int FindCrystalEncoding(uint32_t freq) {
int i;

    for(i=0;i<crystaltablesize;i++) {
        if( crystaltable[i]==freq ) return i<<6;
    }
    return -1;
}


/***************************************************************************
 *                                                                         *
 *              Clock control routines                                     *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief SystemCoreClockSet
 *
 * @param source: one of CLOCK_SRC_xxxx
 * @param div   : value of divisor
 *
 * @note  Does not avoid overclocking!
 * @note  When div is 0, the SYSDIV circuitry is not used
 * @note  Write RCC prior to writing RCC2 (See Note in 5.2.5.2)
 * @note  if div < 16, only RCC is used
 * @note  if div >= 16 and div < 64, SYSDIV2 (and RCC2) must be used
 * @note  if div >= 64, SYSDIV2 and SYSDIVLSB (and RCC2) must be used
 **/

int SystemCoreClockSet(uint8_t source, uint32_t div) {
uint32_t rcc,rcc2;
int32_t  xtal;
int delaycount;

    // Enable required peripherals
    switch(source) {
    case CLOCK_SRC_MOSC:
    case CLOCK_SRC_PLL_2_MOSC:
    case CLOCK_SRC_PLL_MOSC:
        if( SYSCTL->RCC & SYSCTL_RCC_MOSCDIS_B ) {  // MOSC disabled?
            /* Find Encoding of External Crystal */
            xtal = FindCrystalEncoding(XTAL_FREQ);
            if( xtal <= 0 )
                return -1;
            /* Configure MOSC */
            SYSCTL->MISC = SYSCTL_MISC_MOSCPUPMIS_B;
            rcc = SYSCTL->RCC;
            rcc = (rcc &~SYSCTL_RCC_XTAL_M) | (xtal);
            rcc &= ~SYSCTL_RCC_MOSCDIS_B; /* Enable MOSC */
            SYSCTL->RCC = rcc;

            /* Verify */
            delaycount = 524288; // WTF
            while( (delaycount != 0) && (SYSCTL->RIS & SYSCTL_RIS_MOSCPUPRIS_B) == 0 )
                delaycount--;
            if( delaycount == 0 ) return -1;
        }
        break;
    case CLOCK_SRC_HIBOSC:
        if( (HIB->CTL & HIB_CTL_CLK32EN_B) == 0 ) { // HIBOSC disabled?
            HIB->IM  |= HIB_IM_WC_B;
            HIB->CTL |= HIB_CTL_CLK32EN_B;
            /* Verify */
            delaycount = 100000; // WTF
            while( (delaycount != 0) && ((HIB->MIS&HIB_IM_WC_B) == 0) )
                delaycount--;
            if( delaycount == 0 ) return -1;
        }
        break;
    }
    // Get a copy of RCC and RCC2 registers
    rcc = SYSCTL->RCC;
    rcc2 = SYSCTL->RCC2;

    // Clear fields in RCC: SYSDIV, OSCSRC, USESYSDIV, USERCC2, BYPASS
    rcc  &= ~(SYSCTL_RCC_SYSDIV_M|SYSCTL_RCC_OSCSRC_M|SYSCTL_RCC_USESYSDIV_B
             |SYSCTL_RCC_BYPASS_B);
    // Clear fields in RCC2: USERCC2, SYSDIV2, OSCSRC2, USESYSDIV2, BYPASS2, etc
    rcc2 &= ~(SYSCTL_RCC2_USERCC2_B|SYSCTL_RCC2_SYSDIV2_M|SYSCTL_RCC2_OSCSRC2_M
             |SYSCTL_RCC2_BYPASS2_B|SYSCTL_RCC2_SYSDIV2LSB_B|SYSCTL_RCC2_DIV400_B);

#ifndef ALLOW_OVERCLOCKING
    // To avoid overclocking
    if( (source == CLOCK_SRC_PLL_MOSC) || (source == CLOCK_SRC_PLL_PIOSC) ) {
        if( div == 0 || CLOCK_FREQ_PLLOSC/div > CLOCK_FREQ_MAX ) {
            div = (CLOCK_FREQ_PLLOSC+CLOCK_FREQ_MAX-1)/CLOCK_FREQ_MAX;
        }
    } else if(  (source == CLOCK_SRC_PLL_2_MOSC) || (source == CLOCK_SRC_PLL_2_PIOSC) ) {
        if( div == 0 || CLOCK_FREQ_PLLOSC_2/div > CLOCK_FREQ_MAX ) {
            div = (CLOCK_FREQ_PLLOSC_2+CLOCK_FREQ_MAX-1)/CLOCK_FREQ_MAX;
        }
    }
#endif
    // Adjust divisor fields
    if( div != 0 ) {
        div--; /* field is one less than divisor */
        rcc  |= SYSCTL_RCC_USESYSDIV_B;
        if( (div >= 64) || (source == CLOCK_SRC_PLL_MOSC) || (source == CLOCK_SRC_PLL_PIOSC) ) {
            rcc2 |= SYSCTL_RCC2_USERCC2_B|SYSCTL_RCC2_DIV400_B;
            if( div&1 ) rcc2 |= SYSCTL_RCC2_SYSDIV2LSB_B;
            div>>=1;
            rcc2 |= (div<<SYSCTL_RCC2_SYSDIV2_S)&SYSCTL_RCC2_SYSDIV2_M;
       } else if( div >= 16 ) {
            rcc2 |= SYSCTL_RCC2_USERCC2_B;
            rcc2 |= (div<<SYSCTL_RCC2_SYSDIV2_S)&SYSCTL_RCC2_SYSDIV2_M;
       } else {
            rcc |= (div<<SYSCTL_RCC_SYSDIV_S)&SYSCTL_RCC_SYSDIV_M;
            rcc2 |= (div<<SYSCTL_RCC2_SYSDIV2_S)&SYSCTL_RCC2_SYSDIV2_M;
        }
    }

    // Configure according give source
    switch(source) {
    case CLOCK_SRC_MOSC:
        rcc  |= SYSCTL_RCC_OSCSRC_MOSC_V|SYSCTL_RCC_BYPASS_B;
        rcc2 |= SYSCTL_RCC2_OSCSRC2_MOSC_V|SYSCTL_RCC2_BYPASS2_B;
        break;
    case CLOCK_SRC_PIOSC:
        rcc  |= (SYSCTL_RCC_OSCSRC_IOSC_V|SYSCTL_RCC_BYPASS_B);
        rcc2 |= SYSCTL_RCC2_OSCSRC2_MOSC_V|SYSCTL_RCC2_BYPASS2_B;
        break;
    case CLOCK_SRC_PIOSC_4:
        rcc  |= (SYSCTL_RCC_OSCSRC_IOSC_4_V|SYSCTL_RCC_BYPASS_B);
        rcc2 |= (SYSCTL_RCC2_OSCSRC2_IOSC_4_V|SYSCTL_RCC2_BYPASS2_B);
        break;
    case CLOCK_SRC_LFIOSC:
        rcc  |= SYSCTL_RCC_OSCSRC_LFIOSC_V|SYSCTL_RCC_BYPASS_B;
        rcc2 |= SYSCTL_RCC2_OSCSRC2_LFIOSC_V|SYSCTL_RCC2_BYPASS2_B;
        break;
    case CLOCK_SRC_HIBOSC:
        rcc2 |= SYSCTL_RCC2_USERCC2_B;
        rcc  |= SYSCTL_RCC_OSCSRC_IOSC_V|SYSCTL_RCC_BYPASS_B;// Just in case
        rcc2 |= SYSCTL_RCC2_OSCSRC2_HIBOSC_V|SYSCTL_RCC2_BYPASS2_B;
        break;
    case CLOCK_SRC_PLL_MOSC: /* 400 MHz */
        // Configure PLL
        rcc  |= SYSCTL_RCC_OSCSRC_MOSC_V|SYSCTL_RCC_BYPASS_B;
        rcc2 |= SYSCTL_RCC2_OSCSRC2_MOSC_V|SYSCTL_RCC2_BYPASS2_B;
        rcc2 |= SYSCTL_RCC2_USERCC2_B;
        rcc2 |= SYSCTL_RCC2_DIV400_B;
        rcc &= ~SYSCTL_RCC_PWRDN_B;
        rcc2 &= ~SYSCTL_RCC2_PWRDN2_B;
        SYSCTL->RCC  = rcc;
        SYSCTL->RCC2 = rcc2;
        // Wait for PLL
        while((SYSCTL->RIS&SYSCTL_RIS_PLLLRIS_B)==0)      {}
        while((SYSCTL->PLLSTAT&SYSCTL_PLLSTAT_LOCK_B)==0) {}
        // Activate PLL
        rcc  &= ~SYSCTL_RCC_BYPASS_B;
        rcc2 &= ~SYSCTL_RCC2_BYPASS2_B;
        break;
    case CLOCK_SRC_PLL_2_MOSC: /* 200 MHz */
        // Configure PLL
        rcc  |= SYSCTL_RCC_OSCSRC_MOSC_V|SYSCTL_RCC_BYPASS_B;
        rcc2 |= SYSCTL_RCC2_OSCSRC2_MOSC_V|SYSCTL_RCC2_BYPASS2_B;
        rcc &= ~SYSCTL_RCC_PWRDN_B;
        rcc2 &= ~SYSCTL_RCC2_PWRDN2_B;
        SYSCTL->RCC  = rcc;
        SYSCTL->RCC2 = rcc2;
        // Wait for PLL
        while((SYSCTL->RIS&SYSCTL_RIS_PLLLRIS_B)==0)     {}
        while((SYSCTL->PLLSTAT&SYSCTL_PLLSTAT_LOCK_B)==0){}
        // Activate PLL
        rcc  &= ~SYSCTL_RCC_BYPASS_B;
        rcc2 &= ~SYSCTL_RCC2_BYPASS2_B;
        break;
    case CLOCK_SRC_PLL_PIOSC: /* 400 MHz */
        // Configure PLL
        rcc  |= SYSCTL_RCC_OSCSRC_IOSC_V|SYSCTL_RCC_BYPASS_B;
        rcc2 |= SYSCTL_RCC2_OSCSRC2_IOSC_V|SYSCTL_RCC2_BYPASS2_B;
        rcc2 |= SYSCTL_RCC2_USERCC2_B;
        rcc2 |= SYSCTL_RCC2_DIV400_B;
        rcc &= ~SYSCTL_RCC_PWRDN_B;
        rcc2 &= ~SYSCTL_RCC2_PWRDN2_B;
        SYSCTL->RCC  = rcc;
        SYSCTL->RCC2 = rcc2;
        // Wait for PLL
        while((SYSCTL->RIS&SYSCTL_RIS_PLLLRIS_B)==0)      {}
        while((SYSCTL->PLLSTAT&SYSCTL_PLLSTAT_LOCK_B)==0) {}
        // Activate PLL
        rcc  &= ~SYSCTL_RCC_BYPASS_B;
        rcc2 &= ~SYSCTL_RCC2_BYPASS2_B;
        break;
    case CLOCK_SRC_PLL_2_PIOSC: /* 200 MHz */
        // Configure PLL
        rcc  |= SYSCTL_RCC_OSCSRC_IOSC_V|SYSCTL_RCC_BYPASS_B;
        rcc2 |= SYSCTL_RCC2_OSCSRC2_IOSC_V|SYSCTL_RCC2_BYPASS2_B;
        rcc &= ~SYSCTL_RCC_PWRDN_B;
        rcc2 &= ~SYSCTL_RCC2_PWRDN2_B;
        SYSCTL->RCC  = rcc;
        SYSCTL->RCC2 = rcc2;
        // Wait for PLL
        while((SYSCTL->RIS&SYSCTL_RIS_PLLLRIS_B)==0)      {}
        while((SYSCTL->PLLSTAT&SYSCTL_PLLSTAT_LOCK_B)==0) {}
        // Activate PLL
        rcc  &= ~SYSCTL_RCC_BYPASS_B;
        rcc2 &= ~SYSCTL_RCC2_BYPASS2_B;
        break;
    }
    SYSCTL->RCC  = rcc;
    SYSCTL->RCC2 = rcc2;
    return 0;
}

/**
 * @brief GetOscilladorFrequency
 *
 * @param xtal  : xtal field in RCC
 * @param src   : clock source as OSCSRC field in RCC or RCC2 register
 *
 * @note  Does not avoid overclocking!
 * @note  When div is 0, the SYSDIV circuitry is not used
 **/
static uint32_t GetOscillatorFrequency(uint32_t xtal, uint32_t src) {
uint32_t oscClk = CLOCK_FREQ_PIOSC;

  switch (src) {
    case 0:                         /* CLOCK_FREQ_MOSC Main oscillator */
      oscClk = crystaltable[xtal&0x1F];
      break;
    case 1:                         /* IOSC Internal Oscillator */
      oscClk = CLOCK_FREQ_PIOSC;
      break;
    case 2:                         /* IOSC/4 Internal Oscillator/4 */
      oscClk = CLOCK_FREQ_PIOSC/4;
      break;
    case 3:                         /* Low Frequency Internal Oscillator (30kHz+-50%) */
      oscClk = CLOCK_FREQ_LFIOSC;
      break;
    case 7:                         /* Hibernate Module Oscillator (32768 Hz) */
      oscClk = CLOCK_FREQ_HIBOSC;
  }
  return oscClk;
}

/* RCC */
#define GETXTAL(RCC)      ((((RCC)&(SYSCTL_RCC_XTAL_M))>>6))
#define GETSYSDIV(RCC)    ((((RCC)&(SYSCTL_RCC_SYSDIV_M))>>SYSCTL_RCC_SYSDIV_S))
#define GETOSCSRC(RCC)    ((((RCC)&(SYSCTL_RCC_OSCSRC_M))>>4))
/* RCC2 */
#define GETSYSDIV2(RCC2)  ((((RCC2)&(SYSCTL_RCC2_SYSDIV2_M))>>SYSCTL_RCC2_SYSDIV2_S))
#define GETOSCSRC2(RCC2)  ((((RCC2)&(SYSCTL_RCC2_OSCSRC2_M))>>4))

/*
 * SystemCoreClockUpdate sets the global variable to correct frequency of
 * the system clock
 *
 */
void SystemCoreClockUpdate(void) {
    SystemCoreClock = SystemCoreClockGet();
}


/*
 * SystemCoreClockGet returns the current system clock frequency
 */

uint32_t SystemCoreClockGet (void) {
uint32_t rcc, rcc2, div, clockfreq;

    /* Determine clock frequency according to clock register values */
    rcc  = SYSCTL->RCC;
    rcc2 = SYSCTL->RCC2;

    if (rcc2 & SYSCTL_RCC2_USERCC2_B ) {
        /* if RCC2 is used then info is in RCC2 except XTAL in RCC */
        if (rcc2 & SYSCTL_RCC2_BYPASS2_B ) {    /* check BYPASS2 */
            clockfreq = GetOscillatorFrequency(GETXTAL(rcc),GETOSCSRC2(rcc2));
        } else {
            if ( rcc2 & SYSCTL_RCC2_DIV400_B )
                clockfreq = CLOCK_FREQ_PLLOSC;
            else
                clockfreq = CLOCK_FREQ_PLLOSC/2;
        }

        div = GETSYSDIV2(rcc2);
        if (rcc2 & SYSCTL_RCC2_DIV400_B ) {
            div = div<<1;
            if( rcc2&SYSCTL_RCC2_SYSDIV2LSB_B )
                div++;
        }
        div++;
        if (rcc & SYSCTL_RCC_USESYSDIV_B ) { /* check USESYSDIV */
            clockfreq = clockfreq / div;
        }
    } else {
      /* if RCC2 not used then all info is in RCC */
      clockfreq = CLOCK_FREQ_PIOSC;
      if (rcc & SYSCTL_RCC_BYPASS_B ) {                             /* check BYPASS */
        clockfreq = GetOscillatorFrequency (GETXTAL(rcc),GETOSCSRC(rcc));
      } else {
        clockfreq = CLOCK_FREQ_PLLOSC/2;
      }
      if (rcc & SYSCTL_RCC_USESYSDIV_B ) {                         /* check USESYSDIV */
         clockfreq = clockfreq / (GETSYSDIV(rcc) + 1);
      }
    }
    return clockfreq;
}

/*
 * @brief SystemInit
 *
 * @note  System initialization
 */
void SystemInit(void) {
    SystemCoreClock = CLOCK_FREQ_PIOSC;
}
