
#ifndef TM1638MODULE_H
#define TM1638MODULE_H

#include "stm32f1xx.h"
#include <spi.h>

template <Controller controllerModel> class Tm1638Module
{
	const Spi<controllerModel>& m_spi;

	static constexpr uint8_t Symbols[10] = { // ������ ���� ��� ����������
			0x3F, // 0
			0x06, // 1
			0x5B, // 2
			0x4F, // 3
			0x66, // 4
			0x6D, // 5
			0x7D, // 6
			0x07, // 7
			0x7F, // 8
			0x6F  // 9
	};

	char m_internalCharBuffer[8]{};
public:
	Tm1638Module(uint8_t brightness, const Spi<controllerModel>& spi):
		m_spi(spi)
	{
		m_spi.sendByte(0x88|(brightness & 0x07));
	}

	void clear()
	{
		m_spi.sendByte(0x40);
		uint8_t zeros[17]{0xC0};
		m_spi.sendData(zeros, 17);

	}

	uint8_t readKeys()
	{
		uint8_t keys = 0, key_mas[4];
		m_spi.readData(0x42, key_mas, 4);
		keys = (key_mas[3]&0x11) << 3 | (key_mas[2]&0x11) << 2 | (key_mas[1]&0x11) << 1 | (key_mas[0]&0x11);
		return keys;
	}
	void setLed(int index)
	{
		m_spi.sendByte(0x44);
		m_spi.sendData(0xC1 + index*2, 1 & 0x01);
	}

	void resetLed(int index)
	{
		m_spi.sendByte(0x44);
		m_spi.sendData(0xC1 + index*2, 0 & 0x01);
	}

	void setDigit(int value, int position)
	{
		m_spi.sendByte(0x44);
		m_spi.sendData(0xC0+(position)*2, Symbols[value]);
		m_internalCharBuffer[position] = value+0x30;
	}

	void incerementDigit(int position)
	{
		if (m_internalCharBuffer[position] == '9')
		{
			m_internalCharBuffer[position] = '0';
		}
		else
		{
			++m_internalCharBuffer[position];
		}
		setDigit(m_internalCharBuffer[position]-0x30, position);
	}
};

#endif
