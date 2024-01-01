#ifndef PORT_H
#define PORT_H

struct PortPinPair
{
    GPIO_TypeDef* port;
    uint8_t pin;
};

#endif
