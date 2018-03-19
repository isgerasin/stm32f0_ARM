#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"

#define MSECINDAY 86400000
#define CLKINTERV 500
#define BOUNCE 50
#define MINUTE 1000 * 60
#define TENMINUTE MINUTE * 10
#define HOUR TENMINUTE * 6
#define TENHOUR HOUR * 10

enum ButtonState
{
    none,
    oneClick,
    doubleClick,
    both
};

uint64_t timeInMsec = 0;
uint64_t timeInSec = 0;
uint64_t alarmInMsec = 0;
uint8_t isAlarm = 0;
uint32_t digits[10] = {0x3F, 0x6, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x7, 0x7F, 0x6F};
uint64_t lastCallLine[2] = {};
enum ButtonState buttons[2] = {};

void SystemClock_Config(void);
void UserButton_Init(void);
void UserIndicator_Init(void);
void CatchEvent(uint64_t curMsec, uint64_t curSec);
void SetTime(void);
void GetTime(void);
void GetAlarm(void);
void SetAlarm(void);


uint32_t GetNumber(uint64_t sec, uint32_t pos);
uint32_t GetPos(uint64_t msec);
uint32_t GetDot(uint8_t isDot, uint32_t pos);


main(void) {

        SystemClock_Config();
        UserButton_Init();
        UserIndicator_Init();

        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
        LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);

        GetTime();
}

void SetAlarm(void)
{
    uint8_t pos = 0;
    while(1)
    {
        uint64_t curMsec = timeInMsec;
        uint64_t curSec = timeInSec;
        uint32_t posMask = GetPos(pos);
        CatchEvent(curMsec, curSec);

        if (buttons[0] == doubleClick)
        {
            buttons[0] = none;
            GetAlarm();
            return;
        }
        else if (buttons[0] == oneClick)
        {
            buttons[0] = none;
            pos++;
            pos = pos % 4;
        }

        if (buttons[1] == oneClick)
        {
            buttons[1] = none;
            if (pos == 0)
                alarmInMsec += MINUTE;
            else if (pos == 1)
                alarmInMsec += TENMINUTE;
            else if (pos == 2)
                alarmInMsec += HOUR;
            else if (pos == 3)
                alarmInMsec += TENHOUR;
        }
        else if (buttons[1] == doubleClick)
        {
            buttons[1] = none;
            if (pos == 0)
                alarmInMsec -= MINUTE - MSECINDAY;
            else if (pos == 1)
                alarmInMsec -= TENMINUTE- MSECINDAY;
            else if (pos == 2)
                alarmInMsec -= HOUR- MSECINDAY;
            else if (pos == 3)
                alarmInMsec -= TENHOUR- MSECINDAY;
        }
        alarmInMsec %=  MSECINDAY;
        LL_GPIO_WriteOutputPort(GPIOA, (GetNumber(alarmInMsec / 1000, curMsec%4) | ( curSec%2 ? GetPos(curMsec) : GetPos(curMsec) | posMask) | GetDot(1, curMsec%4) ) << 1);
    }
}

void GetAlarm(void)
{
    while(1)
    {
        uint64_t curMsec = timeInMsec;
        uint64_t curSec = timeInSec;
        CatchEvent(curMsec, curSec);

        if (buttons[0] == doubleClick)
        {
            buttons[0] = none;
            return;
        }
        LL_GPIO_WriteOutputPort(GPIOA, (GetNumber(alarmInMsec / 1000, curMsec%4) | GetPos(curMsec) | GetDot(1, curMsec%4) ) << 1 );
    }
}



void SetTime(void)
{
    uint8_t pos = 0;
    while(1)
    {
        uint64_t curMsec = timeInMsec;
        uint64_t curSec = timeInSec;
        uint32_t posMask = GetPos(pos);
        CatchEvent(curMsec, curSec);

        if (buttons[0] == doubleClick)
        {
            buttons[0] = none;
            SetAlarm();
            return;
        }
        else if (buttons[0] == oneClick)
        {
            buttons[0] = none;
            pos++;
            pos = pos % 4;
        }

        if (buttons[1] == oneClick)
        {
            buttons[1] = none;
            if (pos == 0)
                timeInMsec += MINUTE;
            else if (pos == 1)
                timeInMsec += TENMINUTE;
            else if (pos == 2)
                timeInMsec += HOUR;
            else if (pos == 3)
                timeInMsec += TENHOUR;
        }
        else if (buttons[1] == doubleClick)
        {
            buttons[1] = none;
            if (pos == 0)
                timeInMsec -= MINUTE - MSECINDAY;
            else if (pos == 1)
                timeInMsec -= TENMINUTE - MSECINDAY;
            else if (pos == 2)
                timeInMsec -= HOUR - MSECINDAY;
            else if (pos == 3)
                timeInMsec -= TENHOUR - MSECINDAY;
        }
        LL_GPIO_WriteOutputPort(GPIOA, (GetNumber(curSec, curMsec%4) | (curSec%2 ? GetPos(curMsec) :GetPos(curMsec) | posMask) | GetDot(curSec % 2, curMsec%4) ) << 1);
    }

}

void GetTime()
{
    while(1)
    {
        uint64_t curMsec = timeInMsec;
        uint64_t curSec = timeInSec;
        CatchEvent(curMsec, curSec);

        if (buttons[0] == doubleClick)
        {
            buttons[0] = none;
            SetTime();
        }
        if (isAlarm)
        {
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_13);
            LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);
            if (curMsec - alarmInMsec >= MINUTE || buttons[1] == oneClick)
            {
                LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
                LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_13);
                isAlarm = 0;
            }
        }
        LL_GPIO_WriteOutputPort(GPIOA, ( GetNumber(curSec, curMsec%4) | GetPos(curMsec) | GetDot(curSec % 2, curMsec%4) ) << 1);
    }
}

void CatchEvent(uint64_t curMsec, uint64_t curSec)
{
    if (curMsec - lastCallLine[0] > CLKINTERV && lastCallLine[0])
    {
        buttons[0] = oneClick;
        lastCallLine[0] = 0;
    }
    if (curMsec - lastCallLine[1] > CLKINTERV && lastCallLine[1])
    {
        buttons[1] = oneClick;
        lastCallLine[1] = 0;
    }
}


inline uint32_t GetPos(uint64_t curMsec)
{
    return 0xF00 ^ ( 1 << (11 - curMsec%4));
}

inline uint32_t GetDot(uint8_t isDot, uint32_t pos)
{
    return pos == 2 ? isDot << 7 : 0;
}

uint32_t GetNumber(uint64_t sec, uint32_t pos)
{
    uint64_t min = sec / 60;
    switch(pos)
    {
        case 0:
            return digits[min % 10];
        case 1:
            return digits[min % 60 / 10 % 10];
        case 2:
            return digits[min / 60 % 10];
        case 3:
            return digits[min / 600 % 10];
        default:
            return digits[8];
    }
}
/**
  * System Clock Configuration
  * The system Clock is configured as follow :
  *    System ClockGG source            = PLL (HSI/2)
  *    SYSCLK(Hz)                     = 48000000
  *    HCLK(Hz)                       = 48000000
  *    AHB Prescaler                  = 1
  *    APB1 Prescaler                 = 1
  *    HSI Frequency(Hz)              = 8000000
  *    PLLMUL                         = 12
  *    Flash Latency(WS)              = 1
  */

void
SystemClock_Config() {
        /* Set FLASH latency */
        LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
        /* Enable HSI and wait for activation*/
        LL_RCC_HSI_Enable();
        while (LL_RCC_HSI_IsReady() != 1);
        /* Main PLL configuration and activation */
        LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                    LL_RCC_PLL_MUL_12);
        LL_RCC_PLL_Enable();
        while (LL_RCC_PLL_IsReady() != 1);
        /* Sysclk activation on the main PLL */
        LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
        LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
        while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
        /* Set APB1 prescaler */
        LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
        /* Set systick to 1ms */
        SysTick_Config(48000000/1000);
        /* Update CMSIS variable (which can be updated also
         * through SystemCoreClockUpdate function) */
        SystemCoreClock = 48000000;
}

void
NMI_Handler(void) {
}

void
HardFault_Handler(void) {

        while (1);
}

void
SVC_Handler(void) {
}

void
PendSV_Handler(void) {
}

void UserIndicator_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
}

void UserButton_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE1);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_1); 
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);
    NVIC_SetPriority(EXTI0_1_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

#define HANDLERLINE(num) \
        do { \
        if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_##num)) \
        { \
            if (BOUNCE <= timeInMsec - lastCallLine[num] && timeInMsec - lastCallLine[num] <= CLKINTERV) \
            { \
                buttons[num] = doubleClick; \
                lastCallLine[num] = 0; \
            } \
            else \
            { \
                lastCallLine[num] = timeInMsec; \
            } \
        }  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_##num); }while(0)

void EXTI0_1_IRQHandler(void)
{ 
    HANDLERLINE(0);
    HANDLERLINE(1);
}
#undef HANDLERLINE

void EXTI2_3_IRQHandler(void)
{
    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

void
SysTick_Handler(void) {
    timeInMsec++;
    timeInSec = timeInMsec / 1000;

    if (timeInMsec == alarmInMsec)
        isAlarm = 1;


    if (timeInMsec >= MSECINDAY)
        timeInMsec %= MSECINDAY;
}
