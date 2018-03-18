#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"

#define MSECINDAY 86400000
#define CLKINTERV 500

uint64_t timeInMsec = 0;
uint64_t timeInSec = 0;
uint32_t digits[10] = {0x3F, 0x6, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x7, 0x7F, 0x6F};
uint8_t oneClickA = 0;
uint8_t doubleClickA = 0;
uint8_t oneClickC = 0;
uint8_t doubleClickC = 0;
uint8_t buttonA = 0;
uint8_t buttonC = 0;
int64_t lastCallMsecA = 0;
int64_t lastCallMsecC = 0;


void SystemClock_Config(void);
void UserButton_Init(void);
void UserIndicator_Init(void);

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

        while(1)
        {
            uint64_t curMsec = timeInMsec;
            uint64_t curSec = timeInSec;
            if (timeInMsec - lastCallMsecA > CLKINTERV && lastCallMsecA)
            {
                oneClickA = 1;
                lastCallMsecA = 0;
            }
            if (timeInMsec - lastCallMsecC > CLKINTERV && lastCallMsecC)
            {
                oneClickC = 1;
                lastCallMsecC = 0;
            }

            if (oneClickA)
            {
                //LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
                if (buttonA)
                oneClickA = 0;
            }
            else if (doubleClickA)
            {
                // LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);
                doubleClickA = 0;
            }

           // if ()LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_13);


            LL_GPIO_WriteOutputPort(GPIOA, ( GetNumber(curSec, curMsec%4) | GetPos(curMsec) | GetDot(curSec % 2, curMsec%4) ) << 1);
  // LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8 );

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
    //Rising
    // LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_1);

    NVIC_SetPriority(EXTI0_1_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}


void EXTI0_1_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0))
    {
        if ( timeInMsec -lastCallMsecA <= CLKINTERV)
        {
            doubleClickA = 1;
            lastCallMsecA = 0;
        }
        else
        {
            lastCallMsecA = timeInMsec;
        }
        buttonA = 1;
    }
    else if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1))
    {
        if ( timeInMsec -lastCallMsecC <= CLKINTERV)
        {
            doubleClickC = 1;
            lastCallMsecC = 0;
        }
        else
        {
            lastCallMsecC = timeInMsec;
        }
        buttonC = 1;
    }



    // LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);

}


void EXTI2_3_IRQHandler(void)
{
    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

void
SysTick_Handler(void) {
    timeInMsec++;
    timeInSec = timeInMsec / 1000;
    // LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);

    if (timeInMsec == MSECINDAY)
        timeInMsec = 0;
}
