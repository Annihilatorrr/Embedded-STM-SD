#include "display7segmentmax7219.h"
#include <stdio.h>
#include <cstring>

void Display7segmentMax7219::sendData(uint8_t rg, uint8_t dt)
{
    m_spi.sendData(rg, dt);
}

uint8_t Display7segmentMax7219::getIntPartLengthInDigits(int value)
{
    uint8_t numberOfDigits{};
    if (value == 0)
    {
        numberOfDigits = 1;
    }
    while (value != 0)
    {
        value /= 10;
        ++numberOfDigits;
    }
    return numberOfDigits;
}

void Display7segmentMax7219::setZeros(uint8_t digits)
{
    uint8_t clear = m_decodeMode == DecodeMode::On ? static_cast<uint8_t>(Decoded::NUM_0) : static_cast<uint8_t>(NonDecoded::NUM_0);

    for (int i = 1; i <= digits;++i)
    {
        sendData(i, clear);
    }
}

Display7segmentMax7219::Display7segmentMax7219(const Spi& spi):m_spi(spi)
{}

uint32_t Display7segmentMax7219::getPow10n(uint8_t n)
{
    uint32_t retval = 1u;

    while (n > 0u)
    {
        retval *= 10u;
        n--;
    }

    return retval;
}

Display7segmentMax7219::~Display7segmentMax7219()
{
    // TODO Auto-generated destructor stub
}

void Display7segmentMax7219::init(uint8_t intensity, uint8_t maxDigits){
    m_maxDigits = maxDigits;
    sendData(static_cast<uint8_t>(Registers::REG_DISPLAY_TEST), 0);
    setDecodeMode();
    sendData(static_cast<uint8_t>(Registers::REG_SCAN_LIMIT), m_maxDigits - 1);
    setIntensity(intensity);
    turnOn();
    clean();
}

void Display7segmentMax7219::setIntensity(uint8_t intensity)
{
    m_intensity = intensity;
    if (intensity > MaxIntensity)
    {
        return;
    }

    sendData(static_cast<uint8_t>(Registers::REG_INTENSITY), m_intensity);
}

void Display7segmentMax7219::clearDigit(uint8_t digit)
{
    uint8_t clear = m_decodeMode == DecodeMode::On ? static_cast<uint8_t>(Decoded::BLANK) : static_cast<uint8_t>(DecodeMode::Off);
    sendData(digit + 1, clear);
}

void Display7segmentMax7219::clean(void)
{
    uint8_t clear = m_decodeMode == DecodeMode::On ? static_cast<uint8_t>(Decoded::BLANK) : static_cast<uint8_t>(DecodeMode::Off);

    for (int i = 0; i < m_maxDigits; ++i)
    {
        sendData(i + 1, clear);
    }
}

void Display7segmentMax7219::animate(DelayFn f, uint32_t speed)
{
    for(int intensity = 1;intensity <= MaxIntensity;++intensity)
    {
        setIntensity(intensity);
        f(speed);
    }
    for(int intensity = MaxIntensity;intensity > 0;--intensity)
    {
        setIntensity(intensity);
        f(speed);
    }

    for(int intensity = 1;intensity <= MaxIntensity;++intensity)
    {
        setIntensity(intensity);
        f(speed);
    }
    for(int intensity = MaxIntensity;intensity > 0;--intensity)
    {
        setIntensity(intensity);
        f(speed);
    }
}

void Display7segmentMax7219::printDigit(int position, Decoded numeric, bool point)
{
    if(position + 1 > m_maxDigits)
    {
        return;
    }

    if(point)
    {
        sendData (position + 1, static_cast<int> (numeric) | (1 << 7));
    }
    else
    {
        sendData (position + 1, static_cast<int> (numeric) & (~(1 << 7)));
    }
}

void Display7segmentMax7219::printChar(int position, NonDecoded ch, bool point)
{
    if(position + 1 > m_maxDigits)
    {
        return;
    }

    if(point)
    {
        if(m_decodeMode == DecodeMode::Off)
        {
            sendData (position + 1, static_cast<uint8_t>(ch) | (1 << 7));
        }
    }
    else
    {
        if(m_decodeMode == DecodeMode::Off)
        {
            sendData (position + 1, static_cast<uint8_t>(ch) & (~(1 << 7)));
        }
    }
}

//int Display7segmentMax7219::print(double value)
//{
//    char buffer[8]{};
//    sprintf(buffer, "%.2lf", value);
//    auto l{strlen(buffer)};
//    char* ptrToPoint{strchr(buffer, '.')};
//    auto digitsAfterPoint{ptrToPoint != nullptr?l-(ptrToPoint-buffer)-1:0};
//    //setZeros(m_maxDigits);
//    return print(value, digitsAfterPoint, l);
//}

int Display7segmentMax7219::print(double value, uint8_t digitsAfterPoint)
{
    int32_t numberOfDigits{getIntPartLengthInDigits(value) + digitsAfterPoint + int(value < 0)};
    //setZeros(numberOfDigits);
    return print(value, digitsAfterPoint, numberOfDigits);
}

int Display7segmentMax7219::print(double value, uint8_t digitsAfterPoint, int atPosition)
{
    if(digitsAfterPoint > 7)
    {
        digitsAfterPoint = 7;
    }

    sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), 0xFF);

    if (value < 0.0)
    {
        if(atPosition > 0)
        {
            sendData(atPosition, static_cast<uint8_t>(Decoded::MINUS));
            atPosition--;
        }

        value = -value;
    }

    int decimal = (value - static_cast<int>(value))*getPow10n(digitsAfterPoint);
    atPosition -= print(static_cast<int>(value), atPosition, false, decimal !=0 || digitsAfterPoint>0);


    auto firstDecimalPosition = atPosition + 1;
    while(atPosition)
    {
        auto v = int(value*getPow10n(firstDecimalPosition - atPosition))%10;
        sendData(atPosition, v);
        --atPosition;
    }

    sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), static_cast<uint8_t>(m_decodeMode));

    return atPosition;
}

int Display7segmentMax7219::print(int value)
{
    int32_t numberOfDigits{getIntPartLengthInDigits(value)+int(value < 0)};
    setZeros(numberOfDigits);
    return print(value, numberOfDigits, false, false);
}

int Display7segmentMax7219::print(int value, uint8_t position)
{
    return print(value, position, false, false);
}

int Display7segmentMax7219::print(int value, uint8_t position, bool asDecimalPart, bool withPoint)
{
    sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), 0xFF);

    int length{};
    if (value < 0)
    {
        if(position > 0)
        {
            sendData(position, static_cast<uint8_t>(Decoded::MINUS));
            position--;
        }
        value = -value;
        ++length;
    }

    int32_t numberOfDigits{getIntPartLengthInDigits(value)};

    int rightCursor = position - numberOfDigits + 1;

    int tempValue = value;
    int lastIntDigit{tempValue%10};
    if (tempValue == 0)
    {
        if (!asDecimalPart)
        {
            sendData(rightCursor, 0);
            ++length;
        }
        else
        {

        }
    }
    while (tempValue != 0)
    {
        int digit = tempValue % 10;
        tempValue /= 10;

        if (rightCursor >= 0)
        {
            sendData(rightCursor, digit);
            ++length;
        }
        ++rightCursor;
    }

    if(withPoint)
    {
        sendData(position - numberOfDigits + 1, lastIntDigit | (1 << 7));
    }
    // set back initial decode mode
    sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), static_cast<uint8_t>(m_decodeMode));

    return length;
}

void Display7segmentMax7219::turnOn(void)
{
    sendData(static_cast<uint8_t>(Registers::REG_SHUTDOWN), 0x01);
}

void Display7segmentMax7219::turnOff(void)
{
    sendData(static_cast<uint8_t>(Registers::REG_SHUTDOWN), 0x00);
}

void Display7segmentMax7219::setDecodeMode(void)
{
    m_decodeMode = DecodeMode::On;
    sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), static_cast<uint8_t>(m_decodeMode));
}

void Display7segmentMax7219::resetDecodeMode(void)
{
    m_decodeMode = DecodeMode::Off;
    sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), static_cast<uint8_t>(m_decodeMode));
}
