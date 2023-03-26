#include "main.h"
#include "stm32f1xx.h"
#include <display7segmentmax7219.h>
#include <delay.h>

int clockInit(void)

{
    // RCC - reset and clock control, CR - Clock control register
    SET_BIT(RCC->CR, RCC_CR_HSEON); //Enable HSE clock

    while(READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET); //wait until HSE ready

    //Configuring of PLL (HSE crystal frequency is 8MHz)
    // SYSCLK = 72 МГц, USB = 48 МГц, PCLK1 = 36 МГц, PCLK2 = 72 МГц,  ADC = 12 МГц
    RCC->CFGR |= RCC_CFGR_PLLMULL9 //Bits 21:18, 0111: PLL input clock x 9
            | RCC_CFGR_PLLSRC; //Enable PLL as a source of HSE

    RCC->CR |= RCC_CR_PLLON; //Run PLL

    while(!(RCC->CR & RCC_CR_PLLRDY));
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    // 2 cycles of Flash wait because 48 MHz < SYSCLK ≤ 72 MHz
    FLASH->ACR &= ~FLASH_ACR_LATENCY_2;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1 //APB2/1
            | RCC_CFGR_PPRE1_DIV2 //APB1/2
            | RCC_CFGR_HPRE_DIV1; //AHB/1

    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_1);

    RCC->CR &= ~RCC_CR_HSION;

    return 0;
}

// Disable Jtag
void initSwdOnlyDebugging()
{
    // disable them from AF remap and debug I/O configuration register
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // JTAG is disabled, SWD is enabled
}

#ifdef __cplusplus
extern "C"
#endif
void SysTick_Handler(void)
{
    //++msTicks;
    if (sysTick > 0)
    {
        --sysTick;
    }
#ifdef __cplusplus
}
#endif

void animate7SegmentDisplay( Display7segmentMax7219<Controller::f103>& display)
{
    for(int intensity = 1;intensity <= 15;++intensity)
    {
        display.setIntensity(intensity);
        delayMs(50);
    }
    for(int intensity = 15;intensity > 0;--intensity)
    {
        display.setIntensity(intensity);
        delayMs(50);
    }

    for(int intensity = 1;intensity <= 15;++intensity)
    {
        display.setIntensity(intensity);
        delayMs(50);
    }
    for(int intensity = 15;intensity > 0;--intensity)
    {
        display.setIntensity(intensity);
        delayMs(50);
    }
}

void testSpi1()
{
    SpiF103 spi(SpiF103::Spi1, SpiF103::SpiFrameSize::Bit8, true, true);
    Display7segmentMax7219<Controller::f103> display(&spi);

    display.clean();
    display.init(1, 8);
    display.print(88888888);
    delayMs(1000);
    animate7SegmentDisplay(display);
    display.init(15, 8);
    for(int i = 1;i <= 100;++i)
    {
        display.clean();
        display.print(i);
        delayMs(50);
    }
}

void testSpi2()
{
    SpiF103 spi(SpiF103::Spi2, SpiF103::SpiFrameSize::Bit8, true, true);
    Display7segmentMax7219<Controller::f103> display(&spi);

    display.clean();
    display.init(1, 8);
    display.print(88888888);
    delayMs(1000);
    animate7SegmentDisplay(display);
    display.init(15, 8);
    for(int i = 1;i <= 100;++i)
    {
        display.clean();
        display.print(i);
        delayMs(50);
    }
}

int main(void)
{
    clockInit();
    SysTick_Init(72000000);
    initSwdOnlyDebugging();
    testSpi1();
    testSpi2();

    while (1)
    {
    }
}

