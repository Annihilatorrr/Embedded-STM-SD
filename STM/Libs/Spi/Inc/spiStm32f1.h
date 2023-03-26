/*
 * spiStm32f1.h
 *
 *  Created on: 2 mar. 2023
 *      Author: VertexNi
 */

#ifndef SPISTM32F1_H_
#define SPISTM32F1_H_

#include "controllerdef.h"
#include "stm32f1xx.h"

template <Controller> class Spi;

template <> class Spi<Controller::f103> {

public:
    enum SpiNumber{Spi1, Spi2};
    enum SpiFrameSize{Bit8, Bit16};

    struct PortPinPair
    {
        GPIO_TypeDef* port;
        uint8_t pin;
    };

private:
    PortPinPair m_cs;
    PortPinPair m_clock;
    PortPinPair m_miso;
    PortPinPair m_mosi;
    SPI_TypeDef* m_spi;
    Spi<Controller::f103>::SpiFrameSize m_frameSize;
    bool m_msbFirst;
    bool m_fullDuplex;

    void spiXInit(uint8_t spiIndex, PortPinPair cs, PortPinPair clock, PortPinPair miso, PortPinPair mosi)
    {
        m_cs = cs;
        m_miso = miso;
        m_mosi = mosi;
        m_clock = clock;

        m_spi = spiIndex == 1 ? SPI1:SPI2;

        __IO uint32_t& csPortConfigRegister = cs.pin > 7 ? cs.port->CRH : cs.port->CRL;
        __IO uint32_t& clockPortConfigRegister = clock.pin > 7 ? clock.port->CRH : clock.port->CRL;
        __IO uint32_t& misoPortConfigRegister = miso.pin > 7 ? miso.port->CRH : miso.port->CRL;
        __IO uint32_t& mosiPortConfigRegister = mosi.pin > 7 ? mosi.port->CRH : mosi.port->CRL;

        // ~(GPIO_CRL_CNFX | GPIO_CRL_MODEX | GPIO_CRL_CNFX_+_1 | GPIO_CRL_MODEX_+_1 | GPIO_CRL_CNFX_+_2 | GPIO_CRL_MODEX_+_2 | GPIO_CRL_CNFX_+_3 | GPIO_CRL_MODEX_+_3);
        uint32_t csPortConfigTemp = csPortConfigRegister &
                ~((0b11 << (cs.pin%8*4+2)) |
                        (0b11 << cs.pin%8*4) |
                        (0b11 << (clock.pin%8*4+2)) |
                        (0b11 << clock.pin%8*4) |
                        (0b11 << (miso.pin%8*4+2)) |
                        (0b11 << miso.pin%8*4) |
                        (0b11 << (mosi.pin%8*4+2)) |
                        (0b11 << mosi.pin%8*4));
        csPortConfigRegister = csPortConfigTemp;

        mosiPortConfigRegister   |=  (0b11 << mosi.pin%8*4);  	// output, 50 MHz (11)
        mosiPortConfigRegister   &= ~(0b11 << (mosi.pin%8*4+2)); // Push-Pull (00)
        mosiPortConfigRegister   |=  (0b10 << (mosi.pin%8*4+2)); // alternative function push-pull (10)

        misoPortConfigRegister   &= ~(0b11 << miso.pin%8*4);  // Input (00)
        misoPortConfigRegister   |=  (0b10 << (miso.pin%8*4+2)); // with pull-up / pull-down
        miso.port->BSRR   =  (1 << miso.pin);   // Set bit 14 High

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

        if (spiIndex == 1)
        {
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // enable spi clock
        }

        m_spi->CR1   &= ~SPI_CR1_SPE; // disable SPI before configuring
        m_spi->CR1 = (m_frameSize == SpiFrameSize::Bit8 ? (0 << SPI_CR1_DFF_Pos):SPI_CR1_DFF)    // 8 bit Data frame format
						                        | (m_msbFirst ? 0 << SPI_CR1_LSBFIRST_Pos:SPI_CR1_LSBFIRST)//  MSB transferred first
						                        | (m_fullDuplex ? (0 << SPI_CR1_BIDIMODE_Pos):(1 << SPI_CR1_BIDIMODE_Pos))
						                        | SPI_CR1_SSM               //Software SS
						                        | SPI_CR1_SSI               // NSS (CS) pin is high
						                        | SPI_CR1_BR_0 | SPI_CR1_BR_1  //Baud
						                        | SPI_CR1_MSTR // Master mode
						                        | 0 << SPI_CR1_CPOL_Pos // Clock polarity
						                        | 0 << SPI_CR1_CPHA_Pos;  // Clock phase

        m_spi->CR1 |= SPI_CR1_SPE; // Enable SPI
    }

    void initPortAClock()
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    }

    void initPortBClock()
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    }

    void initAltFunctionsClock()
    {
        RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    }

    void initSpi1()
    {
        initAltFunctionsClock();
        initPortAClock();
        spiXInit(1, PortPinPair{GPIOA, 4}, PortPinPair{GPIOA, 5}, PortPinPair{GPIOA,6}, PortPinPair{GPIOA,7});
    }

    void initSpi2()
    {
        initAltFunctionsClock();
        initPortBClock();
        spiXInit(2, PortPinPair{GPIOB, 12}, PortPinPair{GPIOB,13}, PortPinPair{GPIOB,14}, PortPinPair{GPIOB,15});
    }
public:

    Spi(Spi<Controller::f103>::SpiNumber spiNumber, Spi<Controller::f103>::SpiFrameSize frameSize, bool msbFirst, bool fullDuplex):
        m_frameSize(frameSize)
    , m_msbFirst(msbFirst)
    , m_fullDuplex(fullDuplex)
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


    void sendData(uint8_t address, uint8_t data)
    {
        m_cs.port->BSRR = 1 << m_cs.pin << 16U;  // CS RESET
        if (!m_fullDuplex)
        {
            m_spi->CR1 |= SPI_CR1_BIDIOE;
        }
        while(!(m_spi->SR & SPI_SR_TXE));
        if (m_frameSize == Spi<Controller::f103>::SpiFrameSize::Bit8)
        {
            m_spi->DR = address;
            (void) m_spi->DR;
            while(!(m_spi->SR & SPI_SR_TXE));
            m_spi->DR = data;
        }
        else
        {
            m_spi->DR = (uint16_t)address << 8 | data;
        }

        while(!(m_spi->SR & SPI_SR_TXE));
        (void) m_spi->DR;
        while(m_spi->SR & SPI_SR_BSY) {}
        m_cs.port->BSRR = 1 << m_cs.pin; // CS SET
    }

    void readData(uint8_t send, uint8_t *data_mas, uint8_t count){
        m_cs.port->BSRR = 1 << m_cs.pin << 16U;  // CS RESET
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
        m_cs.port->BSRR = 1 << m_cs.pin; // CS SET
    }

    void sendData(uint8_t* data, int dataLength)
    {
        m_cs.port->BSRR = 1 << m_cs.pin << 16U;  // CS RESET
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


        m_cs.port->BSRR = 1 << m_cs.pin; // CS SET
    }

    void sendData(uint8_t address, uint8_t* data, int dataLength)
    {
        m_cs.port->BSRR = 1 << m_cs.pin << 16U;  // CS RESET
        if (!m_fullDuplex)
        {
            m_spi->CR1 |= SPI_CR1_BIDIOE;
        }
        while(!(m_spi->SR & SPI_SR_TXE));
        if (m_frameSize == Spi<Controller::f103>::SpiFrameSize::Bit8)
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
                (void) m_spi->DR;
                while(!(m_spi->SR & SPI_SR_TXE));
                while(m_spi->SR&SPI_SR_BSY) {}
            }
        }

        while(!(m_spi->SR & SPI_SR_TXE));
        (void) m_spi->DR;
        while(m_spi->SR & SPI_SR_BSY) {}
        m_cs.port->BSRR = 1 << m_cs.pin; // CS SET
    }

    void sendByte(uint8_t data)
    {
        m_cs.port->BSRR = 1 << m_cs.pin << 16U;  // CS RESET
        if (!m_fullDuplex)
        {
            m_spi->CR1 |= SPI_CR1_BIDIOE;
        }
        while(!(m_spi->SR & SPI_SR_TXE));
        if (m_frameSize == Spi<Controller::f103>::SpiFrameSize::Bit8)
        {
            while(!(m_spi->SR & SPI_SR_TXE));
            m_spi->DR = data;
        }
        else
        {
            m_spi->DR = (uint16_t)data << 8;
        }

        while(!(m_spi->SR & SPI_SR_TXE));
        (void) m_spi->DR;
        while(m_spi->SR & SPI_SR_BSY) {}
        m_cs.port->BSRR = 1 << m_cs.pin; // CS SET
    }
};

using SpiF103 = Spi<Controller::f103>;
#endif /* SPISTM32F1_H_ */
