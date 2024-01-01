#include "spiStm32f1.h"

Spi::Spi(SpiFrameSize frameSize, bool msbFirst, bool fullDuplex):
m_frameSize(frameSize)
, m_msbFirst(msbFirst)
, m_fullDuplex(fullDuplex)
{

}


void Spi::init(Spi::SpiNumber spiNumber)
{
	switch(spiNumber)
	{
	case Spi1:
		initSpi1();
		break;
	case Spi2:
		initSpi2();
		break;
	}
}

void Spi::initPortAClock()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
}

void Spi::initPortBClock()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
}

void Spi::initAltFunctionsClock()
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
}

void Spi::initSpi1()
{
	initAltFunctionsClock();
	initPortAClock();
	initSpiX(1, PortPinPair{GPIOA, 4}, PortPinPair{GPIOA, 5}, PortPinPair{GPIOA,6}, PortPinPair{GPIOA,7});
}

void Spi::initSpi2()
{
	initAltFunctionsClock();
	initPortBClock();
	initSpiX(2, PortPinPair{GPIOB, 12}, PortPinPair{GPIOB,13}, PortPinPair{GPIOB,14}, PortPinPair{GPIOB,15});
}

void Spi::initSpiX(uint8_t spiIndex, PortPinPair cs, PortPinPair clock, PortPinPair miso, PortPinPair mosi)
{
	m_cs = cs;
	m_miso = miso;
	m_mosi = mosi;
	m_clock = clock;

	m_spi = spiIndex == 1 ? SPI1:SPI2;

	// Selecting configuration port
	// For pins 0..7 Port configuration register low (CRL) is used, for pins 8..15 Port configuration register high (CRH) is used

	// To reset CNF of PinX, use GPIO_CRL_CNFX_Pos which is (x%8)*4 + 2, for example for pin3 the expression is ~(11 << (3%8)*4+2)
	// To reset MODE of PinX, use GPIO_CRL_MODEX_Pos which is (x%8)*4, for example for pin12 the expression is ~(11 << (12%8)*4)

	__IO uint32_t& csPortConfigRegister = cs.pin > 7 ? cs.port->CRH : cs.port->CRL;
	csPortConfigRegister = csPortConfigRegister &
			~((0b11 << (cs.pin % 8 * 4 + 2)) |   // reset CS CNF pin
					(0b11 << cs.pin % 8 * 4)); // reset CS MODE pin

	__IO uint32_t& clockPortConfigRegister = clock.pin > 7 ? clock.port->CRH : clock.port->CRL;
	clockPortConfigRegister = clockPortConfigRegister &
			~((0b11 << (clock.pin % 8 * 4 + 2)) | // reset Clock CNF pin
					(0b11 << clock.pin % 8 * 4));         // reset Clock MODE pin

	__IO uint32_t& misoPortConfigRegister = miso.pin > 7 ? miso.port->CRH : miso.port->CRL;
	misoPortConfigRegister = misoPortConfigRegister &
			~((0b11 << (miso.pin % 8 * 4 + 2)) | // reset MISO CNF pin
					(0b11 << miso.pin % 8 * 4));         // reset MISO MODE pin

	__IO uint32_t& mosiPortConfigRegister = mosi.pin > 7 ? mosi.port->CRH : mosi.port->CRL;
	mosiPortConfigRegister = mosiPortConfigRegister &
			~((0b11 << (mosi.pin % 8 * 4 + 2)) | // reset MOSI CNF pin
					(0b11 << mosi.pin % 8 * 4));         // reset MOSI MODE pin

	mosiPortConfigRegister |=  (0b11 << mosi.pin%8*4);  	// output, 50 MHz (11)
	mosiPortConfigRegister &= ~(0b11 << (mosi.pin%8*4+2)); // Push-Pull (00)
	mosiPortConfigRegister |=  (0b10 << (mosi.pin%8*4+2)); // alternative function push-pull (10)

	misoPortConfigRegister &= ~(0b11 << miso.pin%8*4);  // Input (00)
	misoPortConfigRegister |=  (0b10 << (miso.pin%8*4+2)); // with pull-up / pull-down
	miso.port->BSRR = (1 << miso.pin);   // Set bit 14 High

	clockPortConfigRegister   |=  (0b11 << clock.pin%8*4);  // output 50 MHz
	clockPortConfigRegister   |=  (0b10 << (clock.pin%8*4+2)); // alternative function push-pull

	csPortConfigRegister   |=  (0b11 << cs.pin%8*4);  // output 50 MHz
	csPortConfigRegister   &= ~(0b11 << (cs.pin%8*4+2));	  // Push-Pull General Purpose
	cs.port->BSRR  =   (1 << cs.pin);   // Set bit High

	m_spi->CR1 = 0x0000; // reset SPI configuration registers
	m_spi->CR2 = 0x0000; // reset SPI configuration registers

	if (spiIndex == 2)
	{
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // enable spi clock
	}
	else if (spiIndex == 1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // enable spi clock
	}

	m_spi->CR1   &= ~SPI_CR1_SPE; // disable SPI before configuring
	m_spi->CR1 = (m_frameSize == SpiFrameSize::Bit8 ? (0 << SPI_CR1_DFF_Pos):SPI_CR1_DFF)    // 8 bit Data frame format
			| (m_msbFirst ? 0 << SPI_CR1_LSBFIRST_Pos:SPI_CR1_LSBFIRST)//  MSB transferred first
			| (m_fullDuplex ? (0 << SPI_CR1_BIDIMODE_Pos):(1 << SPI_CR1_BIDIMODE_Pos))
			| SPI_CR1_SSM               //Software SS
			| SPI_CR1_SSI               // NSS (CS) pin is high
			| SPI_CR1_BR  //Baud
			| SPI_CR1_MSTR // Master mode
			| 0 << SPI_CR1_CPOL_Pos // Clock polarity
			| 0 << SPI_CR1_CPHA_Pos;  // Clock phase

	m_spi->CR1 |= SPI_CR1_SPE; // Enable SPI
}

void Spi::startTransfer() const
{
	m_cs.port->BSRR = 1 << m_cs.pin << 16U;  // CS RESET
}

void Spi::endTransfer() const
{
	m_cs.port->BSRR = 1 << m_cs.pin; // CS SET
}

void Spi::sendData(uint8_t address, uint8_t data) const
{
	startTransfer();  // CS RESET
	if (!m_fullDuplex)
	{
		m_spi->CR1 |= SPI_CR1_BIDIOE;
	}
	while(!(m_spi->SR & SPI_SR_TXE));
	if (m_frameSize == Spi::SpiFrameSize::Bit8)
	{
		m_spi->DR = address;
		static_cast<void>(m_spi->DR);
		while(!(m_spi->SR & SPI_SR_TXE));
		m_spi->DR = data;
	}
	else
	{
		m_spi->DR = static_cast<uint16_t>(address) << 8 | data;
	}

	while(!(m_spi->SR & SPI_SR_TXE));
	static_cast<void>(m_spi->DR);
	while(m_spi->SR & SPI_SR_BSY) {}
	endTransfer(); // CS SET
}

void Spi::readData(uint8_t send, uint8_t *data_mas, uint8_t count) const
{
	startTransfer();  // CS RESET
	uint8_t i = 1;

	if (!m_fullDuplex)
	{
		m_spi->CR1 |= SPI_CR1_BIDIOE;
	}
	m_spi->DR = send;

	while(!(m_spi->SR & SPI_SR_TXE));
	while ((m_spi->SR & SPI_SR_BSY));

	m_spi->CR1 &= ~SPI_CR1_BIDIOE;

	while (!(m_spi->SR & SPI_SR_RXNE));
	while ((m_spi->SR & SPI_SR_BSY));

	data_mas[0] = m_spi->DR;

	while(count-- > 1)
	{

		m_spi->DR = 0xFF;

		while (!(m_spi->SR & SPI_SR_RXNE));
		while ((m_spi->SR & SPI_SR_BSY));

		data_mas[i++] = m_spi->DR;
	}
	if (!m_fullDuplex)
	{
		m_spi->CR1 |= SPI_CR1_BIDIOE;
	}
	endTransfer(); // CS SET
}

void Spi::sendData(uint8_t* data, int dataLength) const
{
	startTransfer();  // CS RESET
	if (!m_fullDuplex)
	{
		m_spi->CR1 |= SPI_CR1_BIDIOE;
	}

	for(int i = 0; i< dataLength; ++i)
	{
		while(!(m_spi->SR & SPI_SR_TXE));
		m_spi->DR = data[i];
	}
	while(m_spi->SR&SPI_SR_BSY) {}

	endTransfer(); // CS SET
}

void Spi::sendData(uint8_t address, uint8_t* data, int dataLength) const
{
	startTransfer();  // CS RESET
	if (!m_fullDuplex)
	{
		m_spi->CR1 |= SPI_CR1_BIDIOE;
	}
	while(!(m_spi->SR & SPI_SR_TXE));
	if (m_frameSize == Spi::SpiFrameSize::Bit8)
	{
		for(int i = 0; i< dataLength; ++i)
		{
			m_spi->DR = address;
			while(!(m_spi->SR & SPI_SR_TXE));
			m_spi->DR = data[i];
			while(!(m_spi->SR & SPI_SR_TXE));
			while(m_spi->SR&SPI_SR_BSY) {}
		}
	}
	else
	{
		for(int i = 0; i< dataLength; ++i)
		{
			m_spi->DR = static_cast<uint16_t>(address) << 8 | data[i];
			while(!(m_spi->SR & SPI_SR_TXE));
			static_cast<void>(m_spi->DR);
			while(!(m_spi->SR & SPI_SR_TXE));
			while(m_spi->SR&SPI_SR_BSY) {}
		}
	}

	while(!(m_spi->SR & SPI_SR_TXE));
	static_cast<void>(m_spi->DR);
	while(m_spi->SR & SPI_SR_BSY) {}
	endTransfer(); // CS SET
}

void Spi::sendByte(uint8_t data) const
{
	startTransfer();  // CS RESET
	if (!m_fullDuplex)
	{
		m_spi->CR1 |= SPI_CR1_BIDIOE;
	}
	while(!(m_spi->SR & SPI_SR_TXE));
	if (m_frameSize == Spi::SpiFrameSize::Bit8)
	{
		while(!(m_spi->SR & SPI_SR_TXE));
		m_spi->DR = data;
	}
	else
	{
		m_spi->DR = static_cast<uint16_t>(data) << 8;
	}

	while(!(m_spi->SR & SPI_SR_TXE));
	static_cast<void>(m_spi->DR);
	while(m_spi->SR & SPI_SR_BSY) {}
	endTransfer(); // CS SET
}
