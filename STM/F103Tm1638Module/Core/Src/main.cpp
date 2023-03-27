#include "main.h"
#include "stm32f1xx.h"

#include <spi.h>
#include <delay.h>
#include <tm1638module.h>

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
Tm1638Module<Controller::f103>* tm1638ModulePtr = nullptr;
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
}


#ifdef __cplusplus
extern "C"
#endif
void TIM2_IRQHandler (void)
{
	if (TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;
		GPIOC->ODR^=GPIO_ODR_ODR13;
		if (checkKeys)
		{
			auto keys = tm1638ModulePtr->readKeys();
			if (keys)
			{
				TIM2->ARR = 36000-1;
			}
			else
			{
				TIM2->ARR = 4500-1;
			}
			for(int start = 0;start < 8; start ++)
			{
				if (keys&0x01)
				{
					tm1638ModulePtr->setLed(start);
					tm1638ModulePtr->incerementDigit(start);
				}
				else
				{
					tm1638ModulePtr->resetLed(start);
				}
				keys >>= 1;
			}
		}
	}

#ifdef __cplusplus
}
#endif

void timer2Init(void)
{
	// (PSC+1)(ARR+1) = TIMER_CLK/EVENT_CLK
	// EVENT_CLK = 72000000/(4500*1000)

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 1000-1;

	// ARR - auto-reload value
	TIM2->ARR = 4500-1;

	// Re-initializes the timer counter and generates an update of the registers
	TIM2->EGR |= TIM_EGR_UG;


	NVIC_EnableIRQ (TIM2_IRQn);
}

void timer2Start (void)
{
	//Clear TIM2_IRQn update interrupt
	TIM2->SR &= ~TIM_SR_UIF;

	// Enable interrupt on update
	TIM2->DIER |= TIM_DIER_UIE;

	// Enable timer
	TIM2->CR1 |= TIM_CR1_CEN;
	while (!(TIM2->SR & (1<<0)));
	NVIC_EnableIRQ(TIM2_IRQn);
}

int main(void)
{
	clockInit();
	timer2Init();
	timer2Start();
	SysTick_Init(72000000);
	__enable_irq();

	SpiF103 spi2(SpiF103::SpiFrameSize::Bit8, false, false);
	spi2.init(SpiF103::Spi2);
	//IoDevices::tm1638ModuleInstance(spi2);
	// init
	uint8_t brightness = 7;
	Tm1638Module<Controller::f103> tm1638Module(brightness, spi2);
	tm1638ModulePtr = &tm1638Module;
	tm1638Module.clear();

	for(int index = 0; index < 8; index++)
	{
		tm1638Module.setLed(index);
		delayMs(50);
	}
	delayMs(50);
	for(int index = 0; index < 8; index++)
	{
		tm1638Module.resetLed(index);
		delayMs(50);
	}
	delayMs(50);

	for(int start = 0;start < 8; start ++)
	{
		tm1638Module.setDigit(0, start);
		delayMs(50);
	}
	delayMs(100);

	checkKeys = true;

	while (1)
	{
	}
}

