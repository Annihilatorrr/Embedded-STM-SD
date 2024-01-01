/*
 * display7segmentmax7219.h
 *
 *  Created on: Nov 29, 2022
 *      Author: VertexNi
 */

#ifndef DISPLAY7SEGMENTMAX7219_H
#define DISPLAY7SEGMENTMAX7219_H

#include "stm32f1xx.h"
#include <spistm32f1.h>

class Display7segmentMax7219
{
    enum class DecodeMode: uint8_t
    {
        Off = 0x00,
        On = 0xFF
    };

    DecodeMode m_decodeMode;
    uint8_t m_intensity = 1;
    const uint8_t MaxIntensity = 0x0F;

    const Spi& m_spi;
    uint16_t m_spiCsPin;
    uint8_t m_maxDigits;

    uint8_t getIntPartLengthInDigits(int value);
    void setZeros(uint8_t digits);

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

    enum class Decoded: uint8_t {
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

    enum class NonDecoded:uint8_t
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

    Display7segmentMax7219(const Spi& spi);

    static uint32_t getPow10n(uint8_t n);
    ~Display7segmentMax7219();
    void init(uint8_t intensity, uint8_t maxDigits);
    void setIntensity(uint8_t intensity);
    void clearDigit(uint8_t digit);
    void clean(void);
    void animate(DelayFn f, uint32_t speed);
    void sendData(uint8_t rg, uint8_t dt);
    void printDigit(int position, Decoded numeric, bool point);
    void printChar(int position, NonDecoded ch, bool point);
    //int print(double value);
    int print(double value, uint8_t digitsAfterPoint = 2);
    int print(double value, uint8_t digitsAfterPoint, int atPosition);
    int print(int value);
    int print(int value, uint8_t position);
    int print(int value, uint8_t position, bool asDecimalPart, bool withPoint);
    void turnOn(void);
    void turnOff(void);
    void setDecodeMode(void);
    void resetDecodeMode(void);
};

#endif /* DISPLAY7SEGMENTMAX7219_H */
