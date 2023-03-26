/*
 * display7segmentmax7219.h
 *
 *  Created on: Nov 29, 2022
 *      Author: VertexNi
 */

#ifndef DISPLAY7SEGMENTMAX7219_H
#define DISPLAY7SEGMENTMAX7219_H

#include "stm32f1xx.h"
#include "spi.h"

template <Controller controllerModel> class Display7segmentMax7219
{
    uint8_t m_decodeMode;
    uint8_t m_intensity = 1;
    const uint8_t MaxIntensity = 0x0F;

    Spi<controllerModel>* m_spi;
    uint16_t m_spiCsPin;
    uint8_t m_maxDigits;

    static constexpr uint8_t SYMBOLS[]
                                     {
                                             0x7E,	// numeric 0
                                             0x30,	// numeric 1
                                             0x6D,	// numeric 2
                                             0x79,	// numeric 3
                                             0x33,	// numeric 4
                                             0x5B,	// numeric 5
                                             0x5F,	// numeric 6
                                             0x70,	// numeric 7
                                             0x7F,	// numeric 8
                                             0x7B,	// numeric 9
                                             0x01,	// minus
                                             0x4F,	// letter E
                                             0x37,	// letter H
                                             0x0E,	// letter L
                                             0x67,	// letter P
                                             0x00	// blank
                                     };

    uint8_t getLengthInDigits(int value)
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

    void setZeros(uint8_t digits)
    {
        uint8_t clear = m_decodeMode == 0xFF ? static_cast<uint8_t>(Letters::NUM_0) : 0x00;

        for (int i = 0; i < digits;++i)
        {
            sendData(i, clear);
        }
    }

public:

    enum class Registers
    {
        REG_NO_OP 			= 0x00,
                REG_DIGIT_0 		= 0x01,
                REG_DIGIT_1 		= 0x02,
                REG_DIGIT_2 		= 0x03,
                REG_DIGIT_3 		= 0x04,
                REG_DIGIT_4 		= 0x05,
                REG_DIGIT_5 		= 0x06,
                REG_DIGIT_6 		= 0x07,
                REG_DIGIT_7 		= 0x08,
                REG_DECODE_MODE 	= 0x09,
                REG_INTENSITY 		= 0x0A,
                REG_SCAN_LIMIT 		= 0x0B,
                REG_SHUTDOWN 		= 0x0C,
                REG_DISPLAY_TEST 	= 0x0F
    };

    enum class Digits {
        DIGIT_1 = 1, DIGIT_2 = 2, DIGIT_3 = 3, DIGIT_4 = 4,
                DIGIT_5 = 5, DIGIT_6 = 6, DIGIT_7 = 7, DIGIT_8 = 8
    };

    enum class Letters {
        NUM_0		= 0x00,
                NUM_1		= 0x01,
                NUM_2		= 0x02,
                NUM_3		= 0x03,
                NUM_4		= 0x04,
                NUM_5		= 0x05,
                NUM_6		= 0x06,
                NUM_7		= 0x07,
                NUM_8		= 0x08,
                NUM_9		= 0x09,
                MINUS		= 0x0A,
                LETTER_E	= 0x0B,
                LETTER_H	= 0x0C,
                LETTER_L	= 0x0D,
                LETTER_P	= 0x0E,
                BLANK		= 0x0F
    };



    Display7segmentMax7219(Spi<controllerModel>* spi):m_spi(spi)
    {}

    static uint32_t getPow10n(uint8_t n)
    {
        uint32_t retval = 1u;

        while (n > 0u)
        {
            retval *= 10u;
            n--;
        }

        return retval;
    }

    ~Display7segmentMax7219() {
        // TODO Auto-generated destructor stub
    }

    void init(uint8_t intensity, uint8_t maxDigits){
        m_maxDigits = maxDigits;
        sendData(static_cast<uint8_t>(Registers::REG_DISPLAY_TEST), 0);
        setDecodeMode();
        sendData(static_cast<uint8_t>(Registers::REG_SCAN_LIMIT), m_maxDigits - 1);
        setIntensity(intensity);
        turnOn();
        clean();
    }

    void setIntensity(uint8_t intensity)
    {
        m_intensity = intensity;
        if (intensity > MaxIntensity)
        {
            return;
        }

        sendData(static_cast<uint8_t>(Registers::REG_INTENSITY), m_intensity);
    }

    void clearDigit(uint8_t digit)
    {
        uint8_t clear = m_decodeMode == 0xFF ? static_cast<uint8_t>(Letters::BLANK) : 0x00;
        sendData(digit, clear);
    }

    void clean(void)
    {
        uint8_t clear = m_decodeMode == 0xFF ? static_cast<uint8_t>(Letters::BLANK) : 0x00;

        for (int i = 0; i < 8; ++i)
        {
            sendData(i + 1, clear);
        }
    }

    void sendData(uint8_t rg, uint8_t dt)
    {
        m_spi->sendData(rg, dt);
    }

    void printDigit(int position, Letters numeric, bool point)
    {
        if(position > m_maxDigits)
        {
            return;
        }

        if(point)
        {
            if(m_decodeMode == 0x00)
            {
                sendData(position, SYMBOLS[(int)numeric] | (1 << 7));
            }
            else if(m_decodeMode == 0xFF)
            {
                sendData(position, (int)numeric | (1 << 7));
            }
        }
        else
        {
            if(m_decodeMode == 0x00)
            {
                sendData(position, SYMBOLS[(int)numeric]  & (~(1 << 7)));
            }
            else if(m_decodeMode == 0xFF)
            {
                sendData(position,(int)numeric & (~(1 << 7)));
            }
        }
    }

    int print(float value, uint8_t digitsAfterPoint)
    {
        int32_t numberOfDigits{getLengthInDigits(value)+digitsAfterPoint+int(value < 0)};
        setZeros(numberOfDigits);
        return print(value, digitsAfterPoint, numberOfDigits);
    }

    int print(float value, uint8_t digitsAfterPoint, int position)
    {
        if(digitsAfterPoint > 4)
        {
            digitsAfterPoint = 4;
        }

        sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), 0xFF);

        if (value < 0.0)
        {
            if(position > 0)
            {
                sendData(position, static_cast<uint8_t>(Letters::MINUS));
                position--;
            }

            value = -value;
        }

        int decimal = (value - static_cast<int>(value))*getPow10n(digitsAfterPoint);
        position -= print(static_cast<int>(value), position, false, decimal !=0 || digitsAfterPoint>0);


        auto firstDecimalPosition = position + 1;
        while(position)
        {
            auto v = int(value*getPow10n(firstDecimalPosition - position))%10;
            sendData(position, v);
            --position;
        }

        sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), m_decodeMode);

        return position;
    }

    int print(int value)
    {
        int32_t numberOfDigits{getLengthInDigits(value)+int(value < 0)};
        setZeros(numberOfDigits);
        return print(value, numberOfDigits, false, false);
    }

    int print(int value, uint8_t position)
    {
        return print(value, position, false, false);
    }
    int print(int value, uint8_t position, bool asDecimalPart, bool withPoint)
    {
        sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), 0xFF);

        int length{};
        if (value < 0)
        {
            if(position > 0)
            {
                sendData(position, static_cast<uint8_t>(Letters::MINUS));
                position--;
            }
            value = -value;
            ++length;
        }

        int32_t numberOfDigits{getLengthInDigits(value)};

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
            if(m_decodeMode == 0x00)
            {
                sendData(position - numberOfDigits + 1, SYMBOLS[lastIntDigit] | (1 << 7));
            }
            else if(m_decodeMode == 0xFF)
            {
                sendData(position - numberOfDigits + 1, lastIntDigit | (1 << 7));
            }
        }
        // set back initial decode mode
        sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), m_decodeMode);

        return length;
    }

    void turnOn(void)
    {
        sendData(static_cast<uint8_t>(Registers::REG_SHUTDOWN), 0x01);
    }

    void turnOff(void)
    {
        sendData(static_cast<uint8_t>(Registers::REG_SHUTDOWN), 0x00);
    }

    void setDecodeMode(void)
    {
        m_decodeMode = 0xFF;
        sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), m_decodeMode);
    }

    void resetDecodeMode(void)
    {
        m_decodeMode = 0x00;
        sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), m_decodeMode);
    }
};

#endif /* DISPLAY7SEGMENTMAX7219_H */
