/**
 * @file     startup_tm4c123.c
 * @brief    Startup code for tm4c123 according CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Defines interrupt vector
 * @note     Implements Reset Handler
 * @note     Initializes and calls main
 * @note     CMSIS library used
 *
 **/

#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm-fields.h"

/* main : codigo do usuario */
extern void main(void);

/* _main: inicializacao da biblioteca (newlib?) */
void __attribute__((weak)) _main(void);
void __attribute__((weak)) SystemInit(void);
void __attribute__((weak)) Default_Handler(void);

/* Rotinas para tratamento de excecoes definidas em CMSIS */
/* Devem poder ser redefinidos */
void __attribute__((weak))                          Reset_Handler(void);             /* M0/M0+/M3/M4 */
void __attribute__((weak,alias("Default_Handler"))) NMI_Handler(void);               /* M0/M0+/M3/M4 */
void __attribute__((weak,alias("Default_Handler"))) HardFault_Handler(void);         /* M0/M0+/M3/M4 */
void __attribute__((weak,alias("Default_Handler"))) SVC_Handler(void);               /* M0/M0+/M3/M4 */
void __attribute__((weak,alias("Default_Handler"))) PendSV_Handler(void);            /* M0/M0+/M3/M4 */
void __attribute__((weak,alias("Default_Handler"))) SysTick_Handler(void);           /* M0/M0+/M3/M4 */
void __attribute__((weak,alias("Default_Handler"))) MemManage_Handler(void);         /* M3/M4 */
void __attribute__((weak,alias("Default_Handler"))) BusFault_Handler(void);          /* M3/M4 */
void __attribute__((weak,alias("Default_Handler"))) UsageFault_Handler(void);        /* M3/M4 */
void __attribute__((weak,alias("Default_Handler"))) DebugMon_Handler(void);          /* M3/M4 */

/*
 * Rotinas para tratamento de interrupcoes
 * Variam com implementacao
 */
void GPIOA_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void GPIOB_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void GPIOC_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void GPIOD_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void GPIOE_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void SSI0_IRQHandler(void)           __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler(void)           __attribute__ ((weak, alias("Default_Handler")));
void ADC0Seq0_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void ADC0Seq1_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void ADC0Seq2_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void ADC0Seq3_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void WDT_IRQHandler(void)            __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_0A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_0B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_1A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_1B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_2A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_2B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void ACOMP0_IRQHandler(void)         __attribute__ ((weak, alias("Default_Handler")));
void ACOMP1_IRQHandler(void)         __attribute__ ((weak, alias("Default_Handler")));
void SysCtl_IRQHandler(void)         __attribute__ ((weak, alias("Default_Handler")));
void Flash_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void GPIOF_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void UART2_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void SSI1_IRQHandler(void)           __attribute__ ((weak, alias("Default_Handler")));
void LPTimer_IRQHandler(void)        __attribute__ ((weak, alias("Default_Handler")));
void PORTA_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void PORTD_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_3A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_3B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_4A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_4B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_5A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER16_5B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler(void)           __attribute__ ((weak, alias("Default_Handler")));
void CAN0_IRQHandler(void)           __attribute__ ((weak, alias("Default_Handler")));
void Hibernation_IRQHandler(void)    __attribute__ ((weak, alias("Default_Handler")));
void USB_IRQHandler(void)            __attribute__ ((weak, alias("Default_Handler")));
void UDMASoft_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void UDMAError_IRQHandler(void)      __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq0_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq1_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq2_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq3_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void UART3_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void UART6_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void UART7_IRQHandler(void)          __attribute__ ((weak, alias("Default_Handler")));
void I2C2_IRQHandler(void)           __attribute__ ((weak, alias("Default_Handler")));
void I2C3_IRQHandler(void)           __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_0A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_0B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_1A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_1B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_2A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_2B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_3A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_3B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_4A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_4B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_5A_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void TIMER64_5B_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void System_IRQHandler(void)         __attribute__ ((weak, alias("Default_Handler")));

/*
 * Simbolos definidos no script de ligacao
 * Sao usados para delimitar areas
 */
extern unsigned long _text_start;
extern unsigned long _text_end;
extern unsigned long _data_start;
extern unsigned long _data_end;
extern unsigned long _bss_start;
extern unsigned long _bss_end;
extern unsigned long _StackLimit;
//extern unsigned long _stack_end;
extern void(_StackTop)(void);

/*
 * Tabela de excecoes/interrupcoes
 * Deve ficar na secao nvic para ser armazenada na flash antes de tudo
 *
 * E uma tabela de ponteiro para funcoes  void (*pFunc)(void)
 * Deve poder ser sobreescrito (weak)
 */

__attribute__ ((weak,section(".nvictable")))
void(*nvictable[])(void) = {
    _StackTop,             /* 0 = SP = endereco de stack_top */
    Reset_Handler,			/* 1 = PC = endereco execucao     */
    NMI_Handler,			/* 2 = NMI Handler Exception      */
    HardFault_Handler,		/* 3 = Hard Fault Exception       */
#if __CORTEX_M == 0x03 || __CORTEX_M == 0x04
    MemManage_Handler,      /* 4 = Memory Management Exception*/
    BusFault_Handler,       /* 5 = Bus Fault Exception        */
    UsageFault_Handler,     /* 6 = Usage Fault Exception      */
#else
    0,                      /* 4 = reserved                   */
    0,                      /* 5 = reserved                   */
    0,                      /* 6 = reserved                   */
#endif
    0,                      /* 7 = reserved                   */
    0,                      /* 8 = reserved                   */
    0,                      /* 9 = reserved                   */
    0,                      /*10 = reserved                   */
    SVC_Handler,            /*11 = Software Interrupt         */
    DebugMon_Handler,       /*12 = Debug Monitor              */
    0,                      /*13 = reserved                   */
    PendSV_Handler,         /*14 = PendSV                     */
    SysTick_Handler,        /*15 = SysTick                    */

   /* IRQ  (diferentes para cada implementacao                */
   /* M0:  ? elementos                                        */
   /* M0+: ? elementos                                        */
   /* M4:  138 elementos                                      */
    GPIOA_IRQHandler,       /* IRQ = 0 : GPIO Port A */
    GPIOB_IRQHandler,       /* IRQ = 1 : GPIO Port B */
    GPIOC_IRQHandler,       /* IRQ = 2 : GPIO Port C */
    GPIOD_IRQHandler,       /* IRQ = 3 : GPIO Port D */
    GPIOE_IRQHandler,       /* IRQ = 4 : GPIO Port E */
    UART0_IRQHandler,       /* IRQ = 5 : UART 0      */
    UART1_IRQHandler,       /* IRQ = 6 : UART 1      */
    SSI0_IRQHandler,        /* IRQ = 7 : SSI 0       */
    I2C0_IRQHandler,        /* IRQ = 8 : I2C 0       */
    0,                      /* IRQ = 9 : Reserved  */
    0,                      /* IRQ = 10 : Reserved  */
    0,                      /* IRQ = 11 : Reserved  */
    0,                      /* IRQ = 12 : Reserved  */
    0,                      /* IRQ = 13 : Reserved  */
    ADC0Seq0_IRQHandler,    /* IRQ = 14 : ADC 0 Seq 0 */
    ADC0Seq1_IRQHandler,    /* IRQ = 15 : ADC 0 Seq 1 */
    ADC0Seq2_IRQHandler,    /* IRQ = 16 : ADC 0 Seq 2 */
    ADC0Seq3_IRQHandler,    /* IRQ = 17 : ADC 0 Seq 3 */
    WDT_IRQHandler,         /* IRQ = 18 : Watchdog */
    TIMER16_0A_IRQHandler,  /* IRQ = 19 : Timer 0 A (16/32 bits) */
    TIMER16_0B_IRQHandler,  /* IRQ = 20 : Timer 0 B (16/32 bits) */
    TIMER16_1A_IRQHandler,  /* IRQ = 21 : Timer 1 A (16/32 bits) */
    TIMER16_1B_IRQHandler,  /* IRQ = 22 : Timer 1 B (16/32 bits) */
    TIMER16_2A_IRQHandler,  /* IRQ = 23 : Timer 2 A (16/32 bits) */
    TIMER16_2B_IRQHandler,  /* IRQ = 24 : Timer 2 A (16/32 bits) */
    ACOMP0_IRQHandler,      /* IRQ = 25 : Analog Comparator 0 */
    ACOMP1_IRQHandler,      /* IRQ = 26 : Analog Comparator 1 */
    0,                      /* IRQ = 27 : Reserved */
    SysCtl_IRQHandler,      /* IRQ = 28 : System Control */
    Flash_IRQHandler,       /* IRQ = 29 : Flash/EEPROM */
    GPIOF_IRQHandler,       /* IRQ = 30 : GPIO F */
    0,                      /* IRQ = 31 : Reserved */
    0,                      /* IRQ = 32 : Reserved */
    UART2_IRQHandler,       /* IRQ = 33 : UART 2 */
    SSI1_IRQHandler,        /* IRQ = 34 : SSI 1 */
    TIMER16_3A_IRQHandler,  /* IRQ = 35 : Timer 3 A (16/32 bits) */
    TIMER16_3B_IRQHandler,  /* IRQ = 36 : Timer 3 A (16/32 bits) */
    I2C1_IRQHandler,        /* IRQ = 37 :  */
    0,                      /* IRQ = 38 : Reserved */
    CAN0_IRQHandler,        /* IRQ = 39 : CAN 0 */
    0,                      /* IRQ = 40 : Reserved */
    0,                      /* IRQ = 41 : Reserved */
    0,                      /* IRQ = 42 : Reserved */
    Hibernation_IRQHandler, /* IRQ = 43 : Hibernation */
    USB_IRQHandler,         /* IRQ = 44 : USB */
    0,                      /* IRQ = 45 : Reserved */
    UDMASoft_IRQHandler,    /* IRQ = 46 : uDMA Soft */
    UDMAError_IRQHandler,   /* IRQ = 47 : uDMA Error */
    ADC1Seq0_IRQHandler,    /* IRQ = 48 : ADC 1 Seq 0 */
    ADC1Seq1_IRQHandler,    /* IRQ = 49 : ADC 1 Seq 1 */
    ADC1Seq2_IRQHandler,    /* IRQ = 50 : ADC 1 Seq 2 */
    ADC1Seq3_IRQHandler,    /* IRQ = 51 : ADC 1 Seq 3 */
    0,                      /* IRQ = 52 :  */
    0,                      /* IRQ = 53 :  */
    0,                      /* IRQ = 54 :  */
    0,                      /* IRQ = 55 :  */
    0,                      /* IRQ = 56 :  */
    0,                      /* IRQ = 57 :  */
    0,                      /* IRQ = 58 :  */
    UART3_IRQHandler,       /* IRQ = 59 : UART 3      */
    UART4_IRQHandler,       /* IRQ = 60 : UART 4      */
    UART5_IRQHandler,       /* IRQ = 61 : UART 5      */
    UART6_IRQHandler,       /* IRQ = 62 : UART 6      */
    UART7_IRQHandler,       /* IRQ = 63 : UART 7      */
    0,                      /* IRQ = 64 : Reserved */
    0,                      /* IRQ = 65 : Reserved */
    0,                      /* IRQ = 66 : Reserved */
    0,                      /* IRQ = 67 : Reserved */
    I2C1_IRQHandler,        /* IRQ = 68 : I2C 1       */
    I2C2_IRQHandler,        /* IRQ = 69 : I2C 2       */
    TIMER16_4A_IRQHandler,  /* IRQ = 70 : Timer 4 A (16/32 bits) */
    TIMER16_4B_IRQHandler,  /* IRQ = 71 : Timer 4 B (16/32 bits) */
    0,                      /* IRQ = 72 : Reserved */
    0,                      /* IRQ = 73 : Reserved */
    0,                      /* IRQ = 74 : Reserved */
    0,                      /* IRQ = 75 : Reserved */
    0,                      /* IRQ = 76 : Reserved */
    0,                      /* IRQ = 77 : Reserved */
    0,                      /* IRQ = 78 : Reserved */
    0,                      /* IRQ = 79 : Reserved */
    0,                      /* IRQ = 80 : Reserved */
    0,                      /* IRQ = 81 : Reserved */
    0,                      /* IRQ = 82 : Reserved */
    0,                      /* IRQ = 83 : Reserved */
    0,                      /* IRQ = 84 : Reserved */
    0,                      /* IRQ = 85 : Reserved */
    0,                      /* IRQ = 86 : Reserved */
    0,                      /* IRQ = 87 : Reserved */
    0,                      /* IRQ = 88 : Reserved */
    0,                      /* IRQ = 89 : Reserved */
    0,                      /* IRQ = 90 : Reserved */
    0,                      /* IRQ = 91 : Reserved */
    TIMER16_5A_IRQHandler,  /* IRQ = 23 : Timer 5 A (16/32 bits) */
    TIMER16_5B_IRQHandler,  /* IRQ = 24 : Timer 5 A (16/32 bits) */
    TIMER64_0A_IRQHandler,  /* IRQ = 94 : Timer 0 A (32/64 bits) */
    TIMER64_0B_IRQHandler,  /* IRQ = 95 : Timer 0 B (32/64 bits) */
    TIMER64_1A_IRQHandler,  /* IRQ = 96 : Timer 1 A (32/64 bits) */
    TIMER64_1B_IRQHandler,  /* IRQ = 97 : Timer 1 B (32/64 bits) */
    TIMER64_2A_IRQHandler,  /* IRQ = 98 : Timer 2 A (32/64 bits) */
    TIMER64_2B_IRQHandler,  /* IRQ = 99 : Timer 2 A (32/64 bits) */
    TIMER64_3A_IRQHandler,  /* IRQ = 100 : Timer 3 A (32/64 bits) */
    TIMER64_3B_IRQHandler,  /* IRQ = 101 : Timer 3 B (32/64 bits) */
    TIMER64_4A_IRQHandler,  /* IRQ = 102 : Timer 4 A (32/64 bits) */
    TIMER64_4B_IRQHandler,  /* IRQ = 103 : Timer 4 B (32/64 bits) */
    TIMER64_5A_IRQHandler,  /* IRQ = 104 : Timer 5 A (32/64 bits) */
    TIMER64_5B_IRQHandler,  /* IRQ = 105 : Timer 5 A (32/64 bits) */
    System_IRQHandler,      /* IRQ = 106 : System Exception */
    0,                      /* IRQ = 107 : Reserved */
    0,                      /* IRQ = 108 : Reserved */
    0,                      /* IRQ = 109 : Reserved */
    0,                      /* IRQ = 110 : Reserved */
    0,                      /* IRQ = 111 : Reserved */
    0,                      /* IRQ = 112 : Reserved */
    0,                      /* IRQ = 113 : Reserved */
    0,                      /* IRQ = 114 : Reserved */
    0,                      /* IRQ = 115 : Reserved */
    0,                      /* IRQ = 116 : Reserved */
    0,                      /* IRQ = 117 : Reserved */
    0,                      /* IRQ = 118 : Reserved */
    0,                      /* IRQ = 119 : Reserved */
    0,                      /* IRQ = 120 : Reserved */
    0,                      /* IRQ = 121 : Reserved */
    0,                      /* IRQ = 122 : Reserved */
    0,                      /* IRQ = 123 : Reserved */
    0,                      /* IRQ = 124 : Reserved */
    0,                      /* IRQ = 125 : Reserved */
    0,                      /* IRQ = 126 : Reserved */
    0,                      /* IRQ = 127 : Reserved */
    0,                      /* IRQ = 128 : Reserved */
    0,                      /* IRQ = 129 : Reserved */
    0,                      /* IRQ = 130 : Reserved */
    0,                      /* IRQ = 131 : Reserved */
    0,                      /* IRQ = 132 : Reserved */
    0,                      /* IRQ = 133 : Reserved */
    0,                      /* IRQ = 134 : Reserved */
    0,                      /* IRQ = 135 : Reserved */
    0,                      /* IRQ = 136 : Reserved */
    0,                      /* IRQ = 137 : Reserved */
    0                       /* IRQ = 138 : Reserved */

};

/*
 * Default Interrupt Handler (Halts)
 */

static uint32_t intmask;
void Default_Handler(void) {

    intmask = *((uint32_t *) (SCB_BASE+0xD04));
    while(1) {} /* Loop */
    /* NEVER */
}

/*
 * default SystemInit caso nao seja definido um
 */

void SystemInit(void) {

}

/*
 * default _main caso nao seja definido uma
 */

void _main(void) {

}

/*
 * rotina para parar tudo
 */

void _stop(void) {

    while(1) {}

}

/*
 * Ponto de entrada principal
 */
void __attribute__((weak,naked)) Reset_Handler(void) {
unsigned long *pSource;
unsigned long *pDest;

    /* Passo 1 : Copiar dados inicializados da flash para RAM (section DATA) */
    pSource = &_text_end;
    pDest   = &_data_start;
    while( pDest < &_data_end ) {
        *pDest++ = *pSource++;
    }

    /* Passo 2 : Zerar variaveis nao inicializadas (section BSS) */
    pDest = &_bss_start;
    while( pDest < &_bss_end ) {
        *pDest++ = 0;
    }

#ifdef ENABLE_FPU
    //
    // Enable the floating-point unit.  This must be done here to handle the
    // case where main() uses floating-point and the function prologue saves
    // floating-point registers (which will fault if floating-point is not
    // enabled).

    //
    uint32_t t = SCB->CPACR;
    t &= ~(SCB_CPACR_CP11_M|SCB_CPACR_CP10_M);
    t |= SCB_CPACR_CP11_FULL_V|SCB_CPACR_CP10_FULL_V ;
    SCB->CPACR = t;
#endif
    /* Passo 3 : Chamar SystemInit conforme CMSIS */
    SystemInit();

    /* Passo 4 : Chamar _main para inicializar biblioteca */
    _main();

    /* Passo 5 : Chamar main */
    main();

    _stop();
}
