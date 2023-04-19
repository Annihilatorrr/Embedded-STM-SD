/*
 * spiStm32f1.h
 *
 *  Created on: 2 mar. 2023
 *      Author: VertexNi
 */

#ifndef SPISTM32F1_H_
#define SPISTM32F1_H_

#include "../../Common/Inc/controllerdef.h"
#include "../../Port/Inc/port.h"

#include "stm32f1xx.h"

template <Controller> class Spi;

template <> class Spi<Controller::f103> {

public:
    enum SpiNumber{Spi1, Spi2};
    enum SpiFrameSize{Bit8, Bit16};

private:

    PortPinPair m_cs;
    PortPinPair m_clock;
    PortPinPair m_miso;
    PortPinPair m_mosi;
    SPI_TypeDef* m_spi;
    Spi<Controller::f103>::SpiFrameSize m_frameSize;
    bool m_msbFirst;
    bool m_fullDuplex;

    void spiXInit(uint8_t spiIndex, const PortPinPair& cs, const PortPinPair& clock, const PortPinPair& miso, const PortPinPair& mosi)
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

        // 32bit for pins 0-7  - MODE0[1:0]CNF0[1:0]  .. MODE7[1:0]CNF7[1:0]
        // 32bit for pins 8-15 - MODE8[1:0]CNF8[1:0]  .. MODE15[1:0]CNF15[1:0]

        // CNFy[1:0]
        // in output mode (MODE[1:0] > 00)
        // 00: General purpose output push-pull
        // 01: General purpose output Open-drain
        // 10: Alternate function output Push-pull
        // 11: Alternate function output Open-drain

        // In input mode (MODE[1:0] = 00):
        // 00: Analog mode
        // 01: Floating input (reset state)
        // 10: Input with pull-up / pull-down
        // 11: Reserved

        // MODEy[1:0]
        // 00: Input mode (reset state)
        // 01: Output mode, max speed 10 MHz.
        // 10: Output mode, max speed 2 MHz.
        // 11: Output mode, max speed 50 MHz.

        // to calculate MODEy value: pin%8 * 4
        // to calculate CNFy value: pin%8 * 4 + 2

        // resetting CNFy and MODEy registers:
        // ~(GPIO_CRL_CNFX | GPIO_CRL_MODEX | GPIO_CRL_CNFX_+_1 | GPIO_CRL_MODEX_+_1 | GPIO_CRL_CNFX_+_2 | GPIO_CRL_MODEX_+_2 | GPIO_CRL_CNFX_+_3 | GPIO_CRL_MODEX_+_3);

        uint32_t csPortConfigReset = ~(
                (0b11 << (m_cs.pin%8 * 4 + 2)) |
                (0b11 << m_cs.pin%8 * 4)
        );

        uint32_t mosiPortConfigReset = ~(
                (0b11 << (m_mosi.pin%8 * 4 + 2)) |
                (0b11 << m_mosi.pin%8 * 4)
        );

        uint32_t misoPortConfigReset = ~(
                (0b11 << (m_miso.pin%8 * 4 + 2)) |
                (0b11 << m_miso.pin%8 * 4)
        );

        uint32_t clockPortConfigReset = ~(
                (0b11 << (m_clock.pin%8 * 4 + 2)) |
                (0b11 << m_clock.pin%8 * 4)
        );

        csPortConfigRegister &= csPortConfigReset;
        mosiPortConfigRegister &= mosiPortConfigReset;
        misoPortConfigRegister &= misoPortConfigReset;
        clockPortConfigRegister &= clockPortConfigReset;


        mosiPortConfigRegister   |=  (0b11 << m_mosi.pin%8*4);  	  // output, 50 MHz (11)
        mosiPortConfigRegister   |=  (0b10 << (m_mosi.pin%8*4+2)); // alternative function push-pull (10)

        misoPortConfigRegister   &= ~(0b11 << m_miso.pin%8 * 4);       // input (00)
        misoPortConfigRegister   |=  (0b10 << (m_miso.pin%8 * 4 + 2)); // Input with pull-up / pull-down
        m_miso.port->BSRR   =  (1 << m_miso.pin);                        // Set bit 14 High

        clockPortConfigRegister   |=  (0b11 << m_clock.pin%8*4);     // output, 50 MHz (11)
        clockPortConfigRegister   |=  (0b10 << (m_clock.pin%8*4+2)); // Alternate function output Push-pull (10)

        csPortConfigRegister   |=  (0b11 << m_cs.pin%8*4);     // output 50 MHz
        csPortConfigRegister   &= ~(0b11 << (m_cs.pin%8*4+2));	// Push-Pull General Purpose (11)

        m_cs.port->BSRR  =   (1 << cs.pin);   // Set bit High

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
        m_spi->CR1 = (m_frameSize == SpiFrameSize::Bit8 ? (0 << SPI_CR1_DFF_Pos):SPI_CR1_DFF) // 8/16 bit Data frame format
						                                                        | (m_msbFirst ? 0 << SPI_CR1_LSBFIRST_Pos:SPI_CR1_LSBFIRST)     // LSB/MSB transferred first
						                                                        | (m_fullDuplex ? (0 << SPI_CR1_BIDIMODE_Pos):(1 << SPI_CR1_BIDIMODE_Pos)) // half/full duplex
						                                                        | SPI_CR1_SSM  // Software SS
						                                                        | SPI_CR1_SSI  // NSS (CS) pin is high
						                                                        | SPI_CR1_BR   //Baud rate
						                                                        | SPI_CR1_MSTR // Master mode
						                                                        | 0 << SPI_CR1_CPOL_Pos  // Clock polarity
						                                                        | 0 << SPI_CR1_CPHA_Pos; // Clock phase

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

    Spi(Spi<Controller::f103>::SpiFrameSize frameSize, bool msbFirst, bool fullDuplex):
        m_frameSize(frameSize)
    , m_msbFirst(msbFirst)
    , m_fullDuplex(fullDuplex)
    {

    }
    void changeCs(PortPinPair cs)
    {
        m_cs = cs;
        __IO uint32_t& csPortConfigRegister = cs.pin > 7 ? cs.port->CRH : cs.port->CRL;
        uint32_t csPortConfigReset = ~(
                (0b11 << (cs.pin%8 * 4 + 2)) |
                (0b11 << cs.pin%8 * 4)
        );
        csPortConfigRegister &= csPortConfigReset;
        csPortConfigRegister |=  (0b11 << cs.pin%8 * 4);     // output 50 MHz
        csPortConfigRegister &= ~(0b11 << (cs.pin%8 * 4 + 2)); // Push-Pull General Purpose (11)

        cs.port->BSRR  =   (1 << cs.pin);   // Set bit High
    }

    void init(Spi<Controller::f103>::SpiNumber spiNumber)
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

    void sendData(uint8_t address, uint8_t data) const
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
            static_cast<void> (m_spi->DR);
            while(!(m_spi->SR & SPI_SR_TXE));
            m_spi->DR = data;
        }
        else
        {
            m_spi->DR = static_cast<uint16_t> (address) << 8 | data;
        }

        while(!(m_spi->SR & SPI_SR_TXE));
        (void) m_spi->DR;
        while(m_spi->SR & SPI_SR_BSY) {}
        m_cs.port->BSRR = 1 << m_cs.pin; // CS SET
    }

    void readData(uint8_t send, uint8_t *data_mas, uint8_t count) const
    {
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

    void sendData(uint8_t* data, int dataLength) const
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
        while(m_spi->SR & SPI_SR_BSY) {}

        m_cs.port->BSRR = 1 << m_cs.pin; // CS SET
    }

    void sendData(uint8_t address, uint8_t* data, int dataLength) const
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

    void sendByte(uint8_t data) const
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
