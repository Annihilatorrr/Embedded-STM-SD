/*
 * spiStm32f1.h
 *
 *  Created on: 2 mar. 2023
 *      Author: VertexNi
 */

#ifndef SPISTM32F1_H_
#define SPISTM32F1_H_

#include "stm32f1xx.h"
#include "port.h"
class Spi{

public:
    enum SpiNumber{Spi1, Spi2};
    enum SpiFrameSize{Bit8, Bit16};

private:
    PortPinPair m_cs;
    PortPinPair m_clock;
    PortPinPair m_miso;
    PortPinPair m_mosi;
    SPI_TypeDef* m_spi;
    SpiFrameSize m_frameSize;
    bool m_msbFirst;
    bool m_fullDuplex;

    void initSpi1();
    void initSpi2();
    void initSpiX(uint8_t spiIndex, PortPinPair cs, PortPinPair clock, PortPinPair miso, PortPinPair mosi);

    void initPortAClock();
    void initPortBClock();
    void initAltFunctionsClock();

public:

    Spi(SpiFrameSize frameSize, bool msbFirst, bool fullDuplex);
    void init(Spi::SpiNumber spiNumber);
    void sendData(uint8_t address, uint8_t data) const;

    void readData(uint8_t send, uint8_t *data_mas, uint8_t count) const;

    void sendData(uint8_t* data, int dataLength) const;
    void sendData(uint8_t address, uint8_t* data, int dataLength) const;
    void sendByte(uint8_t data) const;
};

#endif /* SPISTM32F1_H_ */
