// STM32F103 SysTick_Config(72000) + IWDG

#include "stm32f1xx.h"
#include <delay.h>

volatile uint32_t sysTick = 0;
volatile uint32_t msTicks = 0;
void delayMs(uint32_t ms)
{
    sysTick = ms;
    while (sysTick);
}

void SysTick_Init(int cpuFrequency) {
    SysTick->LOAD &= ~SysTick_LOAD_RELOAD_Msk;
    SysTick->LOAD = cpuFrequency/1000-1;
    SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk;
    SysTick->CTRL = (SysTick_CTRL_TICKINT_Msk   |  /* Enable SysTick exception */
            SysTick_CTRL_ENABLE_Msk) |    /* Enable SysTick system timer */
                    SysTick_CTRL_CLKSOURCE_Msk;   /* Use processor clock source */

}
