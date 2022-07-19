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
#define ISR_NO_INT0 0      //
#define ISR_NO_TIMER0 1    //
#define ISR_NO_INT1 2      //
#define ISR_NO_TIMER1 3    //
#define ISR_NO_UART0 4     //
#define ISR_NO_ADC 5       //
#define ISR_NO_LVD 6       //
#define ISR_NO_PCA 7       //
#define ISR_NO_UART2 8     //
#define ISR_NO_SPI 9       //
#define ISR_NO_INT2 10     //
#define ISR_NO_INT3 11     //
#define ISR_NO_TIMER2 12   //
#define ISR_NO_INT4 16     //
#define ISR_NO_UART3 17    //
#define ISR_NO_UART4 18    //
#define ISR_NO_TIMER3 19   //
#define ISR_NO_TIMER4 20   //
#define ISR_NO_CMP 21      //
#define ISR_NO_I2C 24      //
#define ISR_NO_USB 25      //
#define ISR_NO_PWMA 26     //
#define ISR_NO_PWMB 27     //
#define ISR_NO_TKSU 35     //
#define ISR_NO_RTC 36      //
#define ISR_NO_P0INT 37    //
#define ISR_NO_P1INT 38    //
#define ISR_NO_P2INT 39    //
#define ISR_NO_P3INT 40    //
#define ISR_NO_P4INT 41    //
#define ISR_NO_P5INT 42    //
#define ISR_NO_P6INT 43    //
#define ISR_NO_P7INT 44    //
#define ISR_NO_DMA_M2M 47  // DMA: memory to memory
#define ISR_NO_DMA_ADC 48  // DMA: adc
#define ISR_NO_DMA_SPI 49  // DMA: spi
#define ISR_NO_DMA_UR1T 50 // DMA: uart 1 tx
#define ISR_NO_DMA_UR1R 51 // DMA: uart 1 rx
#define ISR_NO_DMA_UR2T 52 // DMA: uart 2 tx
#define ISR_NO_DMA_UR2R 53 // DMA: uart 2 rx
#define ISR_NO_DMA_UR3T 54 // DMA: uart 3 tx
#define ISR_NO_DMA_UR3R 55 // DMA: uart 3 rx
#define ISR_NO_DMA_UR4T 56 // DMA: uart 4 tx
#define ISR_NO_DMA_UR4R 57 // DMA: uart 4 rx
#define ISR_NO_DMA_LCM 58  // DMA: lcm
#define ISR_NO_LCM 59      //

/*******************************************************************************
 * Interrupt Registers
 ******************************************************************************/
SFR(IE, 0xA8);       // interrupt enable
SBIT(EA, 0xA8, 7);   //   global isr enable
SBIT(ELVD, 0xA8, 6); //   low voltage delect
SBIT(EADC, 0xA8, 5); //   adc
SBIT(ES, 0xA8, 4);   //   uart1
SBIT(ET1, 0xA8, 3);  //   timer1
SBIT(EX1, 0xA8, 2);  //   external interrupt 1
SBIT(ET0, 0xA8, 1);  //   timer0
SBIT(EX0, 0xA8, 0);  //   external interrupt 0
SFR(IE2, 0xAF);      // interrupt enable
SBIT(EUSB, 0xAF, 7); //    usb
SBIT(ET4, 0xAF, 6);  //    timer4
SBIT(ET3, 0xAF, 5);  //    timer3
SBIT(ES4, 0xAF, 4);  //    uart4
SBIT(ES3, 0xAF, 3);  //    uart3
SBIT(ET2, 0xAF, 2);  //    timer2
SBIT(ESPI, 0xAF, 1); //    spi
SBIT(ES2, 0xAF, 0);  //    uart2

/*******************************************************************************
 * Clock Registers
 ******************************************************************************/
SFRX(MCLKOCR, 0xFE05); // clock output

/*******************************************************************************
 * Timer 0 Registers
 ******************************************************************************/
SFR(TCON, 0x88); // Timer0 and Timer1 control registers
#define Clr_Timer1_Trig() (TCON &= _BV(7))
#define Set_Timer1_Enable() (TCON |= _BV(6))
#define Set_Timer1_Disble() (TCON &= ~_BV(6))
#define Clr_Timer0_Trig() (TCON &= _BV(5))
#define Set_Timer0_Enable() (TCON |= _BV(4))
#define Set_Timer0_Disble() (TCON &= ~_BV(4))

SFR(TMOD, 0x89); // Timer0 and Timer1 mode registers
SFR(TL0, 0x8A);
SFR(TH0, 0x8C);
SFR(TL1, 0x8B);
SFR(TH1, 0x8D);
SFR(AUXR, 0x8E);
#define Set_Timer0_T(xTime)                            \
    {                                                  \
        TL0 = (unsigned char)(65536 - (xTime));        \
        TH0 = (unsigned char)((65536 - (xTime)) >> 8); \
    }
#define Set_Timer1_T(xTime)                            \
    {                                                  \
        TL1 = (unsigned char)(65536 - (xTime));        \
        TH1 = (unsigned char)((65536 - (xTime)) >> 8); \
    }

#define Set_Timer0_12T() (AUXR &= ~_BV(7))
#define Set_Timer0_1T() (AUXR |= _BV(7))

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
SFR(P6, 0xE8);       // GPIO 6
SBIT(P6_0, 0xE8, 0); //   .0
SBIT(P6_1, 0xE8, 1); //   .1
SBIT(P6_2, 0xE8, 2); //   .2
SBIT(P6_3, 0xE8, 3); //   .3
SBIT(P6_4, 0xE8, 4); //   .4
SBIT(P6_5, 0xE8, 5); //   .5
SBIT(P6_6, 0xE8, 6); //   .6
SBIT(P6_7, 0xE8, 7); //   .7
SFR(P7, 0xF8);       // GPIO 7
SBIT(P7_0, 0xF8, 0); //   .0
SBIT(P7_1, 0xF8, 1); //   .1
SBIT(P7_2, 0xF8, 2); //   .2
SBIT(P7_3, 0xF8, 3); //   .3
SBIT(P7_4, 0xF8, 4); //   .4
SBIT(P7_5, 0xF8, 5); //   .5
SBIT(P7_6, 0xF8, 6); //   .6
SBIT(P7_7, 0xF8, 7); //   .7

SFR(P0M0, 0x94); // P0 Cfg0
SFR(P0M1, 0x93); // P0 Cfg1
SFR(P1M0, 0x92); // P1 Cfg0
SFR(P1M1, 0x91); // P1 Cfg1
SFR(P2M0, 0x96); // P2 Cfg0
SFR(P2M1, 0x95); // P2 Cfg1
SFR(P3M0, 0xB2); // P3 Cfg0
SFR(P3M1, 0xB1); // P3 Cfg1
SFR(P4M0, 0xB4); // P4 Cfg0
SFR(P4M1, 0xB3); // P4 Cfg1
SFR(P5M0, 0xCA); // P5 Cfg0
SFR(P5M1, 0xC9); // P5 Cfg1
SFR(P6M0, 0xCC); // P6 Cfg0
SFR(P6M1, 0xCB); // P6 Cfg1
SFR(P7M0, 0xE2); // P7 Cfg0
SFR(P7M1, 0xE1); // P7 Cfg1

SFRX(P0PU, 0xFE10);
SFRX(P1PU, 0xFE11);
SFRX(P2PU, 0xFE12);
SFRX(P3PU, 0xFE13);
SFRX(P4PU, 0xFE14);
SFRX(P5PU, 0xFE15);
SFRX(P6PU, 0xFE16);
SFRX(P7PU, 0xFE17);

SFRX(P0IE, 0xFE30);
SFRX(P1IE, 0xFE31);
SFRX(P2IE, 0xFE32);
SFRX(P3IE, 0xFE33);
SFRX(P4IE, 0xFE34);
SFRX(P5IE, 0xFE35);
SFRX(P6IE, 0xFE36);
SFRX(P7IE, 0xFE37);

#define _Set_GPIO_In_Res_(port, bits) (P##port##M0 &= ~(bits), P##port##M1 |= (bits))
#define _Set_GPIO_Out_PP_Res_(port, bits) (P##port##M0 |= (bits), P##port##M1 &= ~(bits))
#define _Set_GPIO_Out_OD_Res_(port, bits) (P##port##M0 |= (bits), P##port##M1 |= (bits))

#define _Set_GPIO_Pullup_Res_(port, bits) (P##port##PU |= (bits))
#define _Clr_GPIO_Pullup_Res_(port, bits) (P##port##PU &= ~(bits))

#define _Set_GPIO_MODE_Digital_Res_(port, bits) (P##port##IE |= (bits))
#define _Set_GPIO_MODE_Analog_Res_(port, bits) (P##port##IE &= ~(bits))

#define Set_GPIO_In(port, bits) _Set_GPIO_In_Res_(port, bits)
#define Set_GPIO_Out_PP(port, bits) _Set_GPIO_Out_PP_Res_(port, bits)
#define Set_GPIO_Out_OD(port, bits) _Set_GPIO_Out_OD_Res_(port, bits)

#define Set_GPIO_Pullup(port, bits) _Set_GPIO_Pullup_Res_(port, bits)
#define Clr_GPIO_Pullup(port, bits) _Clr_GPIO_Pullup_Res_(port, bits)

#define Set_GPIO_MODE_Digital(port, bits) _Set_GPIO_MODE_Digital_Res_(port, bits)
#define Set_GPIO_MODE_Analog(port, bits) _Set_GPIO_MODE_Analog_Res_(port, bits)

// #ifdef __cplusplus
// }
// #endif

#endif
