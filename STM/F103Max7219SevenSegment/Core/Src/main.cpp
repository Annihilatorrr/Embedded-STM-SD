#include "main.h"
#include "stm32f1xx.h"
#include "display7segmentmax7219.h"
#include "delay.h"
#include <spiStm32f1.h>

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

    // disable HSI for power-saving purposes
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



void test(const Spi& spi)
{
    Display7segmentMax7219 display(spi);

    display.clean();
    display.init(15, 8);
    display.print(88888888);
    delayMs(1000);
    //    display.printDigit(4, Display7Seg::Decoded::LETTER_H, false);
    //    display.printDigit(3, Display7Seg::Decoded::LETTER_E, false);
    //    display.printDigit(2, Display7Seg::Decoded::LETTER_L, false);
    //    display.printDigit(1, Display7Seg::Decoded::LETTER_L, false);
    //    display.printDigit(0, Display7Seg::Decoded::NUM_0, true);
    //    display.animate(&delayMs, 50);
    //    display.setIntensity(15);
    //    delayMs(500);
    //    display.print(1.2222222);
    //    delayMs(500);
    //    display.print(22.333333);
    //    delayMs(500);
    //    display.print(333.44444);
    //    delayMs(500);
    //    display.print(4444.5555);
    //    delayMs(500);
    //    display.print(55555.666);
    //    delayMs(500);
    //    display.print(666666.77);
    //    delayMs(500);
    //    display.print(7777777.8);
    //    delayMs(500);
    //    display.print(88888888.0);
    //    delayMs(500);
    //
    //    display.resetDecodeMode();
    //    display.clean();
    //    display.printChar(7, Display7Seg::NonDecoded::CHAR_g, false);
    //    display.printChar(6, Display7Seg::NonDecoded::CHAR_o, false);
    //    display.printChar(5, Display7Seg::NonDecoded::CHAR_o, false);
    //    display.printChar(4, Display7Seg::NonDecoded::CHAR_d, false);
    //    display.printChar(3, Display7Seg::NonDecoded::CHAR_b, false);
    //    display.printChar(2, Display7Seg::NonDecoded::CHAR_Y, false);
    //    display.printChar(1, Display7Seg::NonDecoded::CHAR_e, false);
}

int main(void)
{
    int pauseMs = 100;
    clockInit();
    SysTick_Init(72000000);
    initSwdOnlyDebugging();
    Spi spi(Spi::SpiFrameSize::Bit16, true, true);
    spi.init(Spi::Spi1);

    Display7segmentMax7219 display(spi);
    display.init(10, 8);

    int ar[]{1, 12, 123, 1234, 12345, 123456, 1234567, 12345678};
    for(auto item:ar)
    {
        display.print(item);
        delayMs(pauseMs);
    }
    display.clean();
    delayMs(pauseMs);
    for(int i = 0; i < 7; ++i)
    {
        display.print(-ar[i]);
        delayMs(pauseMs);
    }
    display.clean();
    delayMs(pauseMs);
    for(int i = 1; i <= 7; ++i)
    {
        display.print(1.0, i);
        delayMs(pauseMs);
    }
    display.clean();
    delayMs(pauseMs);

    display.printDigit(4, Display7segmentMax7219::Decoded::LETTER_H, false);
    display.printDigit(3, Display7segmentMax7219::Decoded::LETTER_E, false);
    display.printDigit(2, Display7segmentMax7219::Decoded::LETTER_L, false);
    display.printDigit(1, Display7segmentMax7219::Decoded::LETTER_L, false);
    display.printDigit(0, Display7segmentMax7219::Decoded::NUM_0, true);
    display.animate([](uint32_t s){delayMs(s);}, 50);
    //spi.enableDmaAndSend16(2, 0x03); // 8
    Spi spi2 = spi;
    //spi2.changeCs(PortPinPair{GPIOA, 2});
    //test(spi2);

    while (1)
    {
    }

    return 0;
}

