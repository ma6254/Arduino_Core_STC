/*
 created by Deqing Sun for use with CH55xduino
 */

#define ARDUINO_MAIN
// #include "wiring_private.h"
#include <stdint.h>
#include <stdio.h>
#include "Arduino.h"

#include "pins_arduino_include.h"

#include "pins_arduino.h" //only include once in core

void pinMode(uint8_t pin, __xdata uint8_t mode)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    if (port == NOT_A_PORT)
        return;

    if (bit == NOT_A_PIN)
        return;

    switch (port)
    {
    case P0PORT:
        Set_GPIO_MODE_Digital(0, bit);
        switch (mode)
        {
        case INPUT:
            Clr_GPIO_Pullup(0, bit);
            Set_GPIO_In(0, bit);
            break;
        case OUTPUT:
            Clr_GPIO_Pullup(0, bit);
            Set_GPIO_Out_PP(0, bit);
            break;
        case INPUT_PULLUP:
            Set_GPIO_Pullup(0, bit);
            Set_GPIO_In(0, bit);
            break;
        case OUTPUT_OD:
            Clr_GPIO_Pullup(0, bit);
            Set_GPIO_Out_OD(0, bit);
            break;
        }
        break;
    case P1PORT:
        Set_GPIO_MODE_Digital(1, bit);
        switch (mode)
        {
        case INPUT:
            Clr_GPIO_Pullup(1, bit);
            Set_GPIO_In(1, bit);
            break;
        case OUTPUT:
            Clr_GPIO_Pullup(1, bit);
            Set_GPIO_Out_PP(1, bit);
            break;
        case INPUT_PULLUP:
            Set_GPIO_Pullup(1, bit);
            Set_GPIO_In(1, bit);
            break;
        case OUTPUT_OD:
            Clr_GPIO_Pullup(1, bit);
            Set_GPIO_Out_OD(1, bit);
            break;
        }
        break;
    case P2PORT:
        Set_GPIO_MODE_Digital(2, bit);
        switch (mode)
        {
        case INPUT:
            Clr_GPIO_Pullup(2, bit);
            Set_GPIO_In(2, bit);
            break;
        case OUTPUT:
            Clr_GPIO_Pullup(2, bit);
            Set_GPIO_Out_PP(2, bit);
            break;
        case INPUT_PULLUP:
            Set_GPIO_Pullup(2, bit);
            Set_GPIO_In(2, bit);
            break;
        case OUTPUT_OD:
            Clr_GPIO_Pullup(2, bit);
            Set_GPIO_Out_OD(2, bit);
            break;
        }
        break;
    case P3PORT:
        Set_GPIO_MODE_Digital(3, bit);
        switch (mode)
        {
        case INPUT:
            Clr_GPIO_Pullup(3, bit);
            Set_GPIO_In(3, bit);
            break;
        case OUTPUT:
            Clr_GPIO_Pullup(3, bit);
            Set_GPIO_Out_PP(3, bit);
            break;
        case INPUT_PULLUP:
            Set_GPIO_Pullup(3, bit);
            Set_GPIO_In(3, bit);
            break;
        case OUTPUT_OD:
            Clr_GPIO_Pullup(3, bit);
            Set_GPIO_Out_OD(3, bit);
            break;
        }
        break;
    case P4PORT:
        Set_GPIO_MODE_Digital(4, bit);
        switch (mode)
        {
        case INPUT:
            Clr_GPIO_Pullup(4, bit);
            Set_GPIO_In(4, bit);
            break;
        case OUTPUT:
            Clr_GPIO_Pullup(4, bit);
            Set_GPIO_Out_PP(4, bit);
            break;
        case INPUT_PULLUP:
            Set_GPIO_Pullup(4, bit);
            Set_GPIO_In(4, bit);
            break;
        case OUTPUT_OD:
            Clr_GPIO_Pullup(4, bit);
            Set_GPIO_Out_OD(4, bit);
            break;
        }
        break;
    case P5PORT:
        Set_GPIO_MODE_Digital(5, bit);
        switch (mode)
        {
        case INPUT:
            Clr_GPIO_Pullup(5, bit);
            Set_GPIO_In(5, bit);
            break;
        case OUTPUT:
            Clr_GPIO_Pullup(5, bit);
            Set_GPIO_Out_PP(5, bit);
            break;
        case INPUT_PULLUP:
            Set_GPIO_Pullup(5, bit);
            Set_GPIO_In(5, bit);
            break;
        case OUTPUT_OD:
            Clr_GPIO_Pullup(5, bit);
            Set_GPIO_Out_OD(5, bit);
            break;
        }
        break;
    case P6PORT:
        Set_GPIO_MODE_Digital(6, bit);
        switch (mode)
        {
        case INPUT:
            Clr_GPIO_Pullup(6, bit);
            Set_GPIO_In(6, bit);
            break;
        case OUTPUT:
            Clr_GPIO_Pullup(6, bit);
            Set_GPIO_Out_PP(6, bit);
            break;
        case INPUT_PULLUP:
            Set_GPIO_Pullup(6, bit);
            Set_GPIO_In(6, bit);
            break;
        case OUTPUT_OD:
            Clr_GPIO_Pullup(6, bit);
            Set_GPIO_Out_OD(6, bit);
            break;
        }
        break;
    case P7PORT:
        Set_GPIO_MODE_Digital(7, bit);
        switch (mode)
        {
        case INPUT:
            Clr_GPIO_Pullup(7, bit);
            Set_GPIO_In(7, bit);
            break;
        case OUTPUT:
            Clr_GPIO_Pullup(7, bit);
            Set_GPIO_Out_PP(7, bit);
            break;
        case INPUT_PULLUP:
            Set_GPIO_Pullup(7, bit);
            Set_GPIO_In(7, bit);
            break;
        case OUTPUT_OD:
            Clr_GPIO_Pullup(7, bit);
            Set_GPIO_Out_OD(7, bit);
            break;
        }
        break;
    }
}

void digitalWrite(uint8_t pin, __xdata uint8_t val)
{
    // uint8_t pwm = digitalPinToPWM(pin);
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    if (port == NOT_A_PORT)
        return;

    if (bit == NOT_A_PIN)
        return;

    switch (port)
    {
    case P0PORT:
        if (val == LOW)
            P0 &= ~bit;
        else
            P0 |= bit;
        break;
    case P1PORT:
        if (val == LOW)
            P1 &= ~bit;
        else
            P1 |= bit;
        break;
    case P2PORT:
        if (val == LOW)
            P2 &= ~bit;
        else
            P2 |= bit;
        break;
    case P3PORT:
        if (val == LOW)
            P3 &= ~bit;
        else
            P3 |= bit;
        break;
    case P4PORT:
        if (val == LOW)
            P4 &= ~bit;
        else
            P4 |= bit;
        break;
    case P5PORT:
        if (val == LOW)
            P5 &= ~bit;
        else
            P5 |= bit;
        break;
    case P6PORT:
        if (val == LOW)
            P6 &= ~bit;
        else
            P6 |= bit;
        break;
    case P7PORT:
        if (val == LOW)
            P7 &= ~bit;
        else
            P7 |= bit;
        break;
    }
}

uint8_t digitalRead(uint8_t pin)
{
    // uint8_t pwm = digitalPinToPWM(pin);
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    uint8_t portBuf = 0;

    if (port == NOT_A_PORT)
        return LOW;

    if (bit == NOT_A_PIN)
        return LOW;

    switch (port)
    {
    case P0PORT:
        portBuf = P0;
        break;
    case P1PORT:
        portBuf = P1;
        break;
    case P2PORT:
        portBuf = P2;
        break;
    case P3PORT:
        portBuf = P3;
        break;
    case P4PORT:
        portBuf = P4;
        break;
    case P5PORT:
        portBuf = P5;
        break;
    case P6PORT:
        portBuf = P6;
        break;
    case P7PORT:
        portBuf = P7;
        break;
    }

    if (portBuf & bit)
        return HIGH;
    return LOW;
}
