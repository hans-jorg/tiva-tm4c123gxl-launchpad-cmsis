/**
 * @file     main.c
 * @brief    Blink LEDs using interrupts
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     All processing during interrupts (not a good idea)
 * @note     A list is used to control task run during interrupts
 * @note     CMSIS library used
 *
 **/

#include "TM4C123GH6PM.h"

/**
 * BIT Macro
 *
 * To avoid magic numbers
 */
#define BIT(N) (1U<<(N))

/**
 * LEDs are in PortF(3:1)
 */
//@{
#define LED_RED BIT(1)
#define LED_BLUE BIT(2)
#define LED_GREEN BIT(3)
#define LED_ALL (LED_RED|LED_BLUE|LED_GREEN)
#define LED_NONE 0
//@}

/**
 * Switches are in PortF(4) and PortF(0)
 */
//@{
#define SW1     BIT(4)
#define SW2     BIT(0)
#define SW_ALL  (SW1|SW2)
//@}

/**
 * @brief Bus to be used to access GPIO
 *
 * @note AHB: Uses base address 0x4005D000
 *            1 clock access
 *            Corresponding bit in HBCTL must be set
 * @note APB: Uses base address 0x40025000
 *            Slow (2 clock access)
 *            Legacy
 *
 * @note Comment out the following to use APB
 */
#define USE_AHB


/***************************************************************************
 *                                                                         *
 *              SysTick Routines                                           *
 *                                                                         *
 ***************************************************************************/
/**
 * @brief Data for SysTick routines
 *
 * @note  It is only possible to add functions
 */

//@{
#define NTASKS 10
typedef struct {
    void        (*func)(void);
    uint32_t    period;
    uint32_t    timecounter;
} Task_t;
static Task_t TaskTable[NTASKS];

static int TaskCount = 0;
//@}

/**
 * @brief Initializes data structure for SysTick
 *
 * @note  It is only possible to add functions
 */
void TaskInit(void) {
int i;

    TaskCount = 0;
    for(i=0;i<NTASKS;i++) {
        TaskTable[i].func        = 0;
        TaskTable[i].period      = 0;
        TaskTable[i].timecounter = 0;
    }
}


/**
 * @brief TaskAdd
 *
 * @note  Insert a function in the list of functions to be processed
 *        during interrupt
 *
 */
void TaskAdd(int period, void (*func)(void) ) {

    if( TaskCount >= NTASKS )
        return;
    TaskTable[TaskCount].period      = period;
    TaskTable[TaskCount].timecounter = period;
    TaskTable[TaskCount].func        = func;
    TaskCount++;
}

//@}

/**
 * @brief SysTick Handler
 *
 * @note  Run scheduled tasks
 * @note  It is not a good idea, to run task in interrupt mode
 */

//@{
volatile uint32_t tick = 0;

void SysTick_Handler(void) {
int i;

    tick++;

    for(i=0;i<TaskCount;i++) {
        if ( TaskTable[i].func && (TaskTable[i].timecounter==0)  ) {
             TaskTable[i].func();
             TaskTable[i].timecounter = TaskTable[i].period;
        }
        TaskTable[i].timecounter--;
    }
}
//@}

/**
 * @brief Delay
 *
 * @note  It gives a delay milliseconds delay
 *
 */
void Delay(uint32_t delay) {
uint32_t tbegin = tick;

    while( (tick-tbegin)<delay ) {}
}

/**
 * @brief xdelay
 *
 * @note  It gives a very small delay
 * @note  Units are given by the period of SysTick clock
 */
void xdelay(uint32_t delay) {
uint32_t tbegin = SysTick->VAL;

    while( ((SysTick->VAL-tbegin)&0xFFFFFF)<delay ) {}

}

/***************************************************************************
 *                                                                         *
 *              GPIO control routines                                      *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief Initialization of GPIO Port F
 *
 * @note LED pins set to output
 */
void
GPIO_Init(uint32_t outputs, uint32_t inputs) {
GPIOA_Type *gpio;

#ifdef USE_AHB
    gpio = GPIOF_AHB;
    /* Enable AHB access for Port F  */
    SYSCTL->GPIOHBCTL |= BIT(5);
#else
    gpio = GPIOF;
#endif

    /* Enable clock for Port F         */
    SYSCTL->RCGCGPIO  |= BIT(5);

    /* To modify PF0, it is necessary to unlock it. See 10.1 of Datasheet */
    if( (inputs|outputs)&1 ) {
        gpio->LOCK = 0x4C4F434B;                // unlock to set PF0
        *(uint32_t * )(&(gpio->CR))   = 0x0;    // CR is marked read-only
    }

    /* Pins for led are digital output */
    gpio->DIR    = outputs;         /* Only specified bits are outputs */
    gpio->DEN    = inputs|outputs;  /* All pins are digital I/O        */

    xdelay(10);
    if( (inputs|outputs)&1 ) {
        gpio->LOCK = 0x0;                       // lock again
        *(uint32_t * )(&(gpio->CR))   = 0x0;    // CR is marked read-only
    }

}


/**
 * @brief Write to GPIO Port F
 *
 * @note First, pins specified by zero are zeroed. Then the pins
 *       specified by ones, area set
 *
 * @note Use bit-banding
 *
 * @note GPIOA_Type defined as:
 * typedef struct {
 *   __I  uint32_t  RESERVED0[255];  // Deveria ser BITBAND[255]
 *   __IO uint32_t  DATA;
 *   __IO uint32_t  DIR;
 *    ....
 *    } GPIOA_Type;
 *
 */
void
GPIO_WritePins(uint32_t zeroes, uint32_t ones) {
uint32_t *base;

#ifdef USE_AHB
    base = (uint32_t *) GPIOF_AHB_BASE;
#else
    base = (uint32_t *) GPIOF_BASE;
#endif

    *(base+(zeroes|ones)) = ones;
}


/**
 * @brief Reads pins in GPIO Port F
 */
uint32_t
GPIO_ReadPins(void) {
GPIOA_Type *gpio;

#ifdef USE_AHB
    gpio = GPIOF_AHB;
#else
    gpio = GPIOF;
#endif
    return gpio->DATA;
}

/**
 * @brief Enable interrupts from specified pins
 */
static void (*gpiocallback)(uint32_t) = 0;
static uint32_t intpins = 0;

uint32_t
GPIO_EnableInterrupt(uint32_t pins, void (*callback)(uint32_t) ) {
GPIOA_Type *gpio;

#ifdef USE_AHB
    gpio = GPIOF_AHB;
#else
    gpio = GPIOF;
#endif
    gpio->IM = pins;
    intpins  = pins;
    gpiocallback = callback;
}

/**
 * @brief Interrupt Routine for GPIO Port F
 */
//@{
uint32_t ints = 0;

void GPIOF_IRQHandler(void)  {
GPIOA_Type *gpio;
uint32_t m;

#ifdef USE_AHB
    gpio = GPIOF_AHB;
#else
    gpio = GPIOF;
#endif

    m = gpio->RIS;
    gpio->ICR = intpins;

    if( gpiocallback ) gpiocallback(m);

}
//@}

/**
 * @brief Task to blink leds
 *
 * @note  Called every 1000 ms
 */

//@{
int sentido = 0;
void ledblinker(void) {
static int state = 0;

    switch(state) {
    case 0:
        GPIO_WritePins(LED_ALL, LED_RED);
        if( sentido )
            state = 1;
        else
            state = 2;
        break;
    case 1:
        GPIO_WritePins(LED_ALL, LED_GREEN);
        if( sentido )
            state = 2;
        else
            state = 0;
        break;
    case 2:
        GPIO_WritePins(LED_ALL, LED_BLUE);
        if( sentido )
            state = 0;
        else
            state = 1;
        break;
    }
}
//@}

/**
 * @brief Task to monitor switches
 *
 * @note  Called every 1 ms
 *
 * @note  No debounce yet
 */

void switchmonitor(void) {
uint32_t bits = GPIO_ReadPins();

    if( (bits&SW1) != 0 ) { // No debounce yet
        sentido = 1;
    } else {
        sentido = 0;
    }
}

/***************************************************************************
 *                                                                         *
 *              Main procedure                                             *
 *                                                                         *
 ***************************************************************************/

/**
 * @brief Main routine
 *
 * Initializes GPIO
 * Blink LEDs
 */

int main(void) {
int state;
uint32_t bits;
int sentido = 0;

    TaskInit();
    TaskAdd(1,switchmonitor);
    TaskAdd(1000,ledblinker);

    SysTick_Config(SystemCoreClock/1000);

    GPIO_Init(LED_ALL,SW_ALL);

    while(1) {}

    return 0;
}
