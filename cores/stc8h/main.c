#include <Arduino.h>

// void Interrupt_INT0(void) __interrupt(ISR_NO_INT0);
void Interrupt_TIMER0(void) __interrupt(ISR_NO_TIMER0);
// void Interrupt_INT1(void) __interrupt(ISR_NO_INT1);
// void Interrupt_TIMER1(void) __interrupt(ISR_NO_TIMER1);
// void Interrupt_UART0(void) __interrupt(ISR_NO_UART0);
// void Interrupt_ADC(void) __interrupt(ISR_NO_ADC);
// void Interrupt_LVD(void) __interrupt(ISR_NO_LVD);
// void Interrupt_PCA(void) __interrupt(ISR_NO_PCA);
// void Interrupt_UART2(void) __interrupt(ISR_NO_UART2);
// void Interrupt_SPI(void) __interrupt(ISR_NO_SPI);
// void Interrupt_INT2(void) __interrupt(ISR_NO_INT2);
// void Interrupt_INT3(void) __interrupt(ISR_NO_INT3);
// void Interrupt_TIMER2(void) __interrupt(ISR_NO_TIMER2);
// void Interrupt_INT4(void) __interrupt(ISR_NO_INT4);
// void Interrupt_UART3(void) __interrupt(ISR_NO_UART3);
// void Interrupt_UART4(void) __interrupt(ISR_NO_UART4);
// void Interrupt_TIMER3(void) __interrupt(ISR_NO_TIMER3);
// void Interrupt_TIMER4(void) __interrupt(ISR_NO_TIMER4);
// void Interrupt_CMP(void) __interrupt(ISR_NO_CMP);
// void Interrupt_I2C(void) __interrupt(ISR_NO_I2C);
// void Interrupt_USB(void) __interrupt(ISR_NO_USB);
// void Interrupt_PWMA(void) __interrupt(ISR_NO_PWMA);
// void Interrupt_PWMB(void) __interrupt(ISR_NO_PWMB);
// void Interrupt_TKSU(void) __interrupt(ISR_NO_TKSU);
// void Interrupt_RTC(void) __interrupt(ISR_NO_RTC);
// void Interrupt_P0INT(void) __interrupt(ISR_NO_P0INT);
// void Interrupt_P1INT(void) __interrupt(ISR_NO_P1INT);
// void Interrupt_P2INT(void) __interrupt(ISR_NO_P2INT);
// void Interrupt_P3INT(void) __interrupt(ISR_NO_P3INT);
// void Interrupt_P4INT(void) __interrupt(ISR_NO_P4INT);
// void Interrupt_P5INT(void) __interrupt(ISR_NO_P5INT);
// void Interrupt_P6INT(void) __interrupt(ISR_NO_P6INT);
// void Interrupt_P7INT(void) __interrupt(ISR_NO_P7INT);
// void Interrupt_DMA_M2M(void) __interrupt(ISR_NO_DMA_M2M);
// void Interrupt_DMA_ADC(void) __interrupt(ISR_NO_DMA_ADC);
// void Interrupt_DMA_SPI(void) __interrupt(ISR_NO_DMA_SPI);
// void Interrupt_DMA_UR1T(void) __interrupt(ISR_NO_DMA_UR1T);
// void Interrupt_DMA_UR1R(void) __interrupt(ISR_NO_DMA_UR1R);
// void Interrupt_DMA_UR2T(void) __interrupt(ISR_NO_DMA_UR2T);
// void Interrupt_DMA_UR2R(void) __interrupt(ISR_NO_DMA_UR2R);
// void Interrupt_DMA_UR3T(void) __interrupt(ISR_NO_DMA_UR3T);
// void Interrupt_DMA_UR3R(void) __interrupt(ISR_NO_DMA_UR3R);
// void Interrupt_DMA_UR4T(void) __interrupt(ISR_NO_DMA_UR4T);
// void Interrupt_DMA_UR4R(void) __interrupt(ISR_NO_DMA_UR4R);
// void Interrupt_DMA_LCM(void) __interrupt(ISR_NO_DMA_LCM);
// void Interrupt_LCM(void) __interrupt(ISR_NO_LCM);

void main(void)
{
    init();

    //!!!initVariant();

    setup();

    for (;;)
    {
        loop();
    }

    //    return 0;
}

unsigned char _sdcc_external_startup(void) __nonbanked
{
    return 0;
}

extern uint32_t uptime_millis;
void Interrupt_TIMER0(void) __interrupt(ISR_NO_TIMER0)
{
    // TI = 0;
    uptime_millis++;
}
