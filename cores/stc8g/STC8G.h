#ifndef __SYS_REGS_STC8H_H__
#define __SYS_REGS_STC8H_H__

#define SBIT(name, addr, bit) __sbit __at(addr + bit) name
#define SFR(name, addr) __sfr __at(addr) name
#define SFRX(name, addr) __xdata volatile unsigned char __at(addr) name
#define SFR16(name, addr) __sfr16 __at(((addr + 1U) << 8) | addr) name
#define SFR16E(name, fulladdr) __sfr16 __at(fulladdr) name
#define SFR32(name, addr) __sfr32 __at(((addr + 3UL) << 24) | ((addr + 2UL) << 16) | ((addr + 1UL) << 8) | addr) name
#define SFR32E(name, fulladdr) __sfr32 __at(fulladdr) name

// #ifdef __cplusplus
// extern "C"
// {
// #endif

/*******************************************************************************
 * Interrupt Number
 ******************************************************************************/
#define ISR_NO_INT0 0    //
#define ISR_NO_TIMER0 1  //
#define ISR_NO_INT1 2    //
#define ISR_NO_TIMER1 3  //
#define ISR_NO_UART1 4   //
#define ISR_NO_ADC 5     //
#define ISR_NO_LVD 6     //
#define ISR_NO_PCA 7     //
#define ISR_NO_UART2 8   //
#define ISR_NO_SPI 9     //
#define ISR_NO_INT2 10   //
#define ISR_NO_INT3 11   //
#define ISR_NO_TIMER2 12 //
#define ISR_NO_INT4 16   //
#define ISR_NO_UART3 17  //
#define ISR_NO_UART4 18  //
#define ISR_NO_TIMER3 19 //
#define ISR_NO_TIMER4 20 //
#define ISR_NO_CMP 21    //
#define ISR_NO_PWM0 22   //
#define ISR_NO_PWM0FD 23 //
#define ISR_NO_I2C 24    //
#define ISR_NO_USB 25    //
#define ISR_NO_PWM1 28   //
#define ISR_NO_PWM2 29   //
#define ISR_NO_PWM3 30   //
#define ISR_NO_PWM4 31   //
#define ISR_NO_PWM5 32   //
#define ISR_NO_PWM2FD 33 //
#define ISR_NO_PWM4FD 34 //
#define ISR_NO_TKSU 35   //

/*******************************************************************************
 * Interrupt Registers
 ******************************************************************************/
SFR(IE, 0xA8);         // interrupt enable
SBIT(EA, 0xA8, 7);     //   global isr enable
SBIT(ELVD, 0xA8, 6);   //   low voltage delect
SBIT(EADC, 0xA8, 5);   //   adc
SBIT(ES, 0xA8, 4);     //   uart1
SBIT(ET1, 0xA8, 3);    //   timer1
SBIT(EX1, 0xA8, 2);    //   external interrupt 1
SBIT(ET0, 0xA8, 1);    //   timer0
SBIT(EX0, 0xA8, 0);    //   external interrupt 0
SFR(IE2, 0xAF);        // interrupt enable
SBIT(ETKSUI, 0xAF, 7); //    touchkey
SBIT(ET4, 0xAF, 6);    //    timer4
SBIT(ET3, 0xAF, 5);    //    timer3
SBIT(ES4, 0xAF, 4);    //    uart4
SBIT(ES3, 0xAF, 3);    //    uart3
SBIT(ET2, 0xAF, 2);    //    timer2
SBIT(ESPI, 0xAF, 1);   //    spi
SBIT(ES2, 0xAF, 0);    //    uart2

/*******************************************************************************
 * GPIO Registers
 ******************************************************************************/
SFR(P0, 0x80);       // GPIO 0
SBIT(P0_0, 0x80, 0); //   .0
SBIT(P0_1, 0x80, 1); //   .1
SBIT(P0_2, 0x80, 2); //   .2
SBIT(P0_3, 0x80, 3); //   .3
SBIT(P0_4, 0x80, 4); //   .4
SBIT(P0_5, 0x80, 5); //   .5
SBIT(P0_6, 0x80, 6); //   .6
SBIT(P0_7, 0x80, 7); //   .7
SFR(P1, 0x90);       // GPIO 1
SBIT(P1_0, 0x90, 0); //   .0
SBIT(P1_1, 0x90, 1); //   .1
SBIT(P1_2, 0x90, 2); //   .2
SBIT(P1_3, 0x90, 3); //   .3
SBIT(P1_4, 0x90, 4); //   .4
SBIT(P1_5, 0x90, 5); //   .5
SBIT(P1_6, 0x90, 6); //   .6
SBIT(P1_7, 0x90, 7); //   .7
SFR(P2, 0xA0);       // GPIO 2
SBIT(P2_0, 0xA0, 0); //   .0
SBIT(P2_1, 0xA0, 1); //   .1
SBIT(P2_2, 0xA0, 2); //   .2
SBIT(P2_3, 0xA0, 3); //   .3
SBIT(P2_4, 0xA0, 4); //   .4
SBIT(P2_5, 0xA0, 5); //   .5
SBIT(P2_6, 0xA0, 6); //   .6
SBIT(P2_7, 0xA0, 7); //   .7
SFR(P3, 0xB0);       // GPIO 3
SBIT(P3_0, 0xB0, 0); //   .0
SBIT(P3_1, 0xB0, 1); //   .1
SBIT(P3_2, 0xB0, 2); //   .2
SBIT(P3_3, 0xB0, 3); //   .3
SBIT(P3_4, 0xB0, 4); //   .4
SBIT(P3_5, 0xB0, 5); //   .5
SBIT(P3_6, 0xB0, 6); //   .6
SBIT(P3_7, 0xB0, 7); //   .7
SFR(P4, 0xC0);       // GPIO 4
SBIT(P4_0, 0xC0, 0); //   .0
SBIT(P4_1, 0xC0, 1); //   .1
SBIT(P4_2, 0xC0, 2); //   .2
SBIT(P4_3, 0xC0, 3); //   .3
SBIT(P4_4, 0xC0, 4); //   .4
SBIT(P4_5, 0xC0, 5); //   .5
SBIT(P4_6, 0xC0, 6); //   .6
SBIT(P4_7, 0xC0, 7); //   .7
SFR(P5, 0xC8);       // GPIO 5
SBIT(P5_0, 0xC8, 0); //   .0
SBIT(P5_1, 0xC8, 1); //   .1
SBIT(P5_2, 0xC8, 2); //   .2
SBIT(P5_3, 0xC8, 3); //   .3
SBIT(P5_4, 0xC8, 4); //   .4
SBIT(P5_5, 0xC8, 5); //   .5
SBIT(P5_6, 0xC8, 6); //   .6
SBIT(P5_7, 0xC8, 7); //   .7



// #ifdef __cplusplus
// }
// #endif

#endif
