#include "main.h"
#include "stm32f1xx.h"

#include <spi.h>
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


bool checkKeys = false;
SpiF103 spi2(SpiF103::SpiFrameSize::Bit8, false, false);
#ifdef __cplusplus
extern "C"
#endif

void SysTick_Handler(void)
{
	++msTicks;
	if (sysTick > 0)
	{
		--sysTick;
	}
	if (checkKeys && msTicks % 250)
	{
		uint8_t keys = 0, key_mas[4];
		spi2.readData(0x42, key_mas, 4);
		keys = (key_mas[3]&0x11) << 3 | (key_mas[2]&0x11) << 2 | (key_mas[1]&0x11) << 1 | (key_mas[0]&0x11);
		for(int start = 0;start < 8; start ++)
		{
			spi2.sendData(0xC1+start*2, keys&0x01);
			keys >>= 1;
		}
	}
#ifdef __cplusplus
}
#endif


uint8_t a = 0, dat = 0, mas[8], ind_mas[10];

uint8_t sym_mas[10] = { // ������ ���� ��� ����������
		0x3F, // 0
		0x06, // 1
		0x5B, // 2
		0x4F, // 3
		0x66, // 4
		0x6D, // 5
		0x7D, // 6
		0x07, // 7
		0x7F, // 8
		0x6F  // 9
};


int main(void)
{
	clockInit();
	SysTick_Init(72000000);

	spi2.init(SpiF103::Spi2);
	// init
	uint8_t brightness = 7;
	spi2.sendByte(0x88|(brightness & 0x07));

	spi2.sendByte(0x40); // ������������� ������
	uint8_t zeros[17]{0xC0};
	spi2.sendData(zeros, 17); // ��������� �����

	spi2.sendByte(0x44); // ������ �� ��������������� ������
	for(int start = 0;start < 8; start ++)
	{
		spi2.sendData(0xC1+start*2, 1&0x01);
		delayMs(100);
	}
	delayMs(100);
	for(int start = 0;start < 8; start ++)
	{
		spi2.sendData(0xC1+start*2, 0&0x01);
		delayMs(100);
	}
	delayMs(100);
	for(int start = 0;start < 8; start ++)
	{
		spi2.sendData(0xC0+start*2, sym_mas[start]);
		delayMs(100);
	}
	delayMs(100);
	int testCounter = 1;

	checkKeys = true;

	//    TM1638_Init(7);
	//
	//    	ALL_Clear();
	//
	//    	setupDisplay(true, 7);

	while (1)
	{
	}
}

