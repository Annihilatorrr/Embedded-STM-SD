#include "main.h"
#include "stm32f1xx.h"
#include "7segmentMax7219/Inc/display7segmentmax7219.h"
#include "Common/Inc/delay.h"

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

void test(const SpiF103& spi)
{
    Display7segmentMax7219<Controller::f103> display(spi);

    display.clean();
    display.init(15, 8);
    display.print(88888888);
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

__IO uint32_t tmpreg;
uint16_t src_dma_buf[1024] = {0};
uint16_t dst_dma_buf[1024] = {0};
uint8_t fl=0;

class Dma
{
public:
    void init()
    {
        //DMA controller clock enable
        RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;

        DMA1_Channel1->CCR |= DMA_CCR_MEM2MEM; //Включаем режим MEM2MEM
        // DMA1_Channel1->CCR |= DMA_CCR_DIR; // mem->per
        DMA1_Channel1->CCR &= ~DMA_CCR_DIR; //per->mem
        DMA1_Channel1->CCR |= DMA_CCR_PINC; //Peripheral increment mode after each transaction
        DMA1_Channel1->CCR |= DMA_CCR_MINC; //Memory increment mode after each transaction
        DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0; //Peripheral size: 16bit
        DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0; //Memory size: 16bit
        DMA1_Channel1->CCR |= DMA_CCR_PL; //Channel priority level: very high
        DMA1_Channel1->CCR &= ~DMA_CCR_CIRC; //Circular mode disabled

        //Enable Transfer complete interrupt
        DMA1_Channel1->CCR |= DMA_CCR_TCIE;
        //Enable Transfer error interrupt
        DMA1_Channel1->CCR |= DMA_CCR_TEIE;

        NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    }
};

extern "C" void DMA1_Channel1_IRQHandler(void)
{
    if(READ_BIT(DMA1->ISR, DMA_ISR_TCIF1) == (DMA_ISR_TCIF1))
    {
        DMA1->IFCR |= DMA_IFCR_CGIF1; /* Clear all interrupt flags */
    }
    else if(DMA1->ISR & DMA_ISR_TEIF1)
    {
        __NOP();
    }
}

int main(void)
{
    clockInit();
    SysTick_Init(72000000);
    initSwdOnlyDebugging();
//    Dma dma;
//    dma.init();
//    for(int i=0; i < 1024; i++)
//    {
//        src_dma_buf[i] = 111;
//    }
//
//    for(int i=0; i < 1024; i++)
//    {
//        dst_dma_buf[i] = 222;
//    }
//
//    //Clear Channel 1  transfer complete flag
//    DMA1->IFCR |= DMA_IFCR_CGIF1;
//    //Clear Channel 1 transfer error flag
//    DMA1->IFCR |= DMA_IFCR_CTEIF1;
//    //Set Number of data to transfer
//    DMA1_Channel1->CNDTR = 1024;
//    //Configure the Source and Destination addresses
//    DMA1_Channel1->CPAR =(uint32_t)&src_dma_buf;
//    DMA1_Channel1->CMAR = (uint32_t)&dst_dma_buf;
//
//    //Enable DMA channel
//    DMA1_Channel1->CCR |= DMA_CCR_EN;

    SpiF103 spi(SpiF103::SpiFrameSize::Bit16, true, true);
    spi.init<SpiF103::Spi1>();

    Display7segmentMax7219<Controller::f103> display(spi);

    display.clean();
    display.init(10, 8);
    display.print(9);
    spi.enableDmaAndSend16(2, 0x03); // 8
    //SpiF103 spi2 = spi;
    //spi2.changeCs(PortPinPair{GPIOA, 2});
    //test(spi2);

    while (1)
    {
    }
}

