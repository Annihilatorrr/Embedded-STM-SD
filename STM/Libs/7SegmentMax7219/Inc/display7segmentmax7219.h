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
    enum class DecodeMode: uint8_t
    {
        Off = 0x00,
                On = 0xFF
    };
    DecodeMode m_decodeMode;
    uint8_t m_intensity = 1;
    const uint8_t MaxIntensity = 0x0F;

    const Spi<controllerModel>& m_spi;
    uint16_t m_spiCsPin;
    uint8_t m_maxDigits;

    uint8_t getIntPartLengthInDigits(int value)
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
        uint8_t clear = m_decodeMode == DecodeMode::On ? static_cast<uint8_t>(Decoded::NUM_0) : static_cast<uint8_t>(NonDecoded::NUM_0);

        for (int i = 1; i <= digits;++i)
        {
            sendData(i, clear);
        }
    }

    using DelayFn = void(*)(uint32_t);
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

    enum class Decoded {
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
                BLANK		= 0x0F,
                HZ  = 0x10
    };

    enum class NonDecoded
    {
        NUM_0 = 0x7E, // numeric 0
                NUM_1 = 0x30, // numeric 1
                NUM_2 = 0x6D, // numeric 2
                NUM_3 = 0x79, // numeric 3
                NUM_4 = 0x33, // numeric 4
                NUM_5 = 0x5B, // numeric 5
                NUM_6 = 0x5F, // numeric 6
                NUM_7 = 0x70, // numeric 7
                NUM_8 = 0x7F, // numeric 8
                NUM_9 = 0x7B, // numeric 9
                CHAR_MINUS = 0x01, // minus
                CHAR_A = 0b01110111, // A
                CHAR_B =  0b01111111, // B
                CHAR_b =  0b00011111, // b
                CHAR_c = 0b00001101, // c
                CHAR_C = 0b01001110, // C
                CHAR_D = 0b01111110, // D
                CHAR_d = 0b00111101, // d
                CHAR_E = 0x4F, // letter E
                CHAR_e = 0b01101111, // e
                CHAR_F = 0b01000111,
                CHAR_g = 0b01111011,
                CHAR_h = 0b00010111,
                CHAR_H = 0b00110111,
                CHAR_i = 0b00010000,
                CHAR_J = 0b01111100,
                CHAR_j = 0b00011000,
                CHAR_L = 0b00001110,
                CHAR_O = 0b01111110,
                CHAR_o = 0b00011101,
                CHAR_P = 0b01100111,
                CHAR_q = 0b01110011,
                CHAR_r = 0b00000101,
                CHAR_S = 0b01011011,
                CHAR_t = 0b00001111,
                CHAR_U = 0b00111110,
                CHAR_u = 0b00011100,
                CHAR_Y = 0b00111011,
                CHAR_BLANK = 0b00000000,
                CHAR_I = 0b00110000
    };

    Display7segmentMax7219(const Spi<controllerModel>& spi):m_spi(spi)
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

    ~Display7segmentMax7219()
    {
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
        uint8_t clear = m_decodeMode == DecodeMode::On ? static_cast<uint8_t>(Decoded::BLANK) : DecodeMode::Off;
        sendData(digit + 1, clear);
    }

    void clean(void)
    {
        uint8_t clear = m_decodeMode == DecodeMode::On ? static_cast<uint8_t>(Decoded::BLANK) : static_cast<uint8_t>(DecodeMode::Off);

        for (int i = 0; i < m_maxDigits; ++i)
        {
            sendData(i + 1, clear);
        }
    }

    void animate(DelayFn f, uint32_t speed)
    {
        for(int intensity = 1;intensity <= 15;++intensity)
        {
            setIntensity(intensity);
            f(speed);
        }
        for(int intensity = 15;intensity > 0;--intensity)
        {
            setIntensity(intensity);
            f(speed);
        }

        for(int intensity = 1;intensity <= 15;++intensity)
        {
            setIntensity(intensity);
            f(speed);
        }
        for(int intensity = 15;intensity > 0;--intensity)
        {
            setIntensity(intensity);
            f(speed);
        }
    }
    void sendData(uint8_t rg, uint8_t dt)
    {
        m_spi.sendData(rg, dt);
    }

    void printDigit(int position, Decoded numeric, bool point)
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

    void printChar(int position, NonDecoded ch, bool point)
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

    int print(double value)
    {
        uint8_t digitsAfterPoint = m_maxDigits - getIntPartLengthInDigits(value) - int(value < 0);
        setZeros(m_maxDigits);
        return print(value, digitsAfterPoint, m_maxDigits);
    }

    int print(double value, uint8_t digitsAfterPoint)
    {
        int32_t numberOfDigits{getIntPartLengthInDigits(value) + digitsAfterPoint + int(value < 0)};
        setZeros(numberOfDigits);
        return print(value, digitsAfterPoint, numberOfDigits);
    }

    int print(double value, uint8_t digitsAfterPoint, int atPosition)
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

    int print(int value)
    {
        int32_t numberOfDigits{getIntPartLengthInDigits(value)+int(value < 0)};
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
        m_decodeMode = DecodeMode::On;
        sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), static_cast<uint8_t>(m_decodeMode));
    }

    void resetDecodeMode(void)
    {
        m_decodeMode = DecodeMode::Off;
        sendData(static_cast<uint8_t>(Registers::REG_DECODE_MODE), static_cast<uint8_t>(m_decodeMode));
    }
};

#endif /* DISPLAY7SEGMENTMAX7219_H */
