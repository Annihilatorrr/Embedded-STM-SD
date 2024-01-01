#include "display7segmentmax7219.h"

void Display7segmentMax7219::sendData(uint8_t rg, uint8_t dt)
{
    m_spi.sendData(rg, dt);
}
