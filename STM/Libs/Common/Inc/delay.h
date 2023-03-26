// STM32F103 SysTick_Config(72000) + IWDG
#ifndef DELAY_H
#define DELAY_H

extern volatile uint32_t sysTick;
void delayMs(uint32_t ms);
void SysTick_Init(int cpuFrequency);

#endif
