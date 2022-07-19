#include "wiring_private.h"
#include "HardwareSerial.h"

void init()
{
    EA = 0;

    Set_GPIO_MODE_Digital(0, 0xFF);
    Set_GPIO_MODE_Digital(1, 0xFF);
    Set_GPIO_MODE_Digital(2, 0xFF);
    Set_GPIO_MODE_Digital(3, 0xFF);
    Set_GPIO_MODE_Digital(4, 0xFF);
    Set_GPIO_MODE_Digital(5, 0xFF);
    Set_GPIO_MODE_Digital(6, 0xFF);
    Set_GPIO_MODE_Digital(7, 0xFF);

    Clr_GPIO_Pullup(0, 0xFF);
    Clr_GPIO_Pullup(1, 0xFF);
    Clr_GPIO_Pullup(2, 0xFF);
    Clr_GPIO_Pullup(3, 0xFF);
    Clr_GPIO_Pullup(4, 0xFF);
    Clr_GPIO_Pullup(5, 0xFF);
    Clr_GPIO_Pullup(6, 0xFF);
    Clr_GPIO_Pullup(7, 0xFF);

    Set_GPIO_In(0, 0xFF);
    Set_GPIO_In(1, 0xFF);
    Set_GPIO_In(2, 0xFF);
    Set_GPIO_In(3, 0xFF);
    Set_GPIO_In(4, 0xFF);
    Set_GPIO_In(5, 0xFF);
    Set_GPIO_In(6, 0xFF);
    Set_GPIO_In(7, 0xFF);

    ET0 = 1;
    Set_Timer0_1T();
#if F_CPU == 12000000
    Set_Timer0_T(12000);
#elif F_CPU == 24000000
    Set_Timer0_T(24000);
#elif F_CPU == 30000000
    Set_Timer0_T(30000);
#elif F_CPU == 35000000
    Set_Timer0_T(35000);
#endif
    Set_Timer0_Enable();

    _HwSerialInit();

    EA = 1;
}

uint32_t uptime_millis = 0;
uint32_t millis(void)
{
    uint32_t tmp = 0;

    ET0 = 0;
    tmp = uptime_millis;
    ET0 = 1;
    return tmp;
}

void delay(uint32_t ms)
{
    uint32_t start = millis();
    uint8_t i = 0;

    while (millis() < (start + ms))
    {
        for (i = 0; i < 255; i++)
        {
        }
    }
}
