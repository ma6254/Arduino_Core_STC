#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include "pins_arduino_include.h"

#ifndef _BV
#define _BV(X) (1 << (X))
#endif

__code uint8_t PROGMEM digital_pin_to_port_PGM[] = {
    NOT_A_PORT, // Placeholder 0
    // left
    P5PORT,     // 1 CCP2_2/CCP2/MCLKO/NRST/SCL_2/MOSI/RxD_3/T1CLKO/T0/INT2/ADC4/P5.4
    NOT_A_PORT, // 2 Vcc/AVcc
    P5PORT,     // 3 ECI_2/ECI/CCP2_3/SDA_2/SS/TxD_3/T0CLKO/T1/INT3/ADC5/P5.5
    NOT_A_PORT, // 4 Gnd/AGnd
    // right
    P3PORT, // 5 P3.0/ADC0/RxD/INT4
    P3PORT, // 6 P3.1/ADC1/TxD/CCP0_2/ECI_3
    P3PORT, // 7 P3.2/ADC2/RxD_2/INT0/SCLK/SCL/CCP0/CCP0_3
    P3PORT, // 8 P3.3/ADC3/TxD_2/INT1/MISO/SDA/CCP1/CCP1_2/CCP1_3
};

__code uint8_t digital_pin_to_bit_mask_PGM[] = {
    NOT_A_PIn, // Placeholder 0
    // left
    _BV(4),    // 1 CCP2_2/CCP2/MCLKO/NRST/SCL_2/MOSI/RxD_3/T1CLKO/T0/INT2/ADC4/P5.4
    NOT_A_PIN, // 2 Vcc/AVcc
    _BV(5),    // 3 ECI_2/ECI/CCP2_3/SDA_2/SS/TxD_3/T0CLKO/T1/INT3/ADC5/P5.5
    NOT_A_PIN, // 4 Gnd/AGnd
    // right
    _BV(0), // 5 P3.0/ADC0/RxD/INT4
    _BV(1), // 6 P3.1/ADC1/TxD/CCP0_2/ECI_3
    _BV(2), // 7 P3.2/ADC2/RxD_2/INT0/SCLK/SCL/CCP0/CCP0_3
    _BV(3), // 8 P3.3/ADC3/TxD_2/INT1/MISO/SDA/CCP1/CCP1_2/CCP1_3
};

__code uint8_t digital_pin_to_channel_PGM[] = {
    NOT_ANALOG, // Placeholder 0
    // left
    4,          // 1 CCP2_2/CCP2/MCLKO/NRST/SCL_2/MOSI/RxD_3/T1CLKO/T0/INT2/ADC4/P5.4
    NOT_ANALOG, // 2 Vcc/AVcc
    5,          // 3 ECI_2/ECI/CCP2_3/SDA_2/SS/TxD_3/T0CLKO/T1/INT3/ADC5/P5.5
    NOT_ANALOG, // 4 Gnd/AGnd
    // right
    0, // 5 P3.0/ADC0/RxD/INT4
    1, // 6 P3.1/ADC1/TxD/CCP0_2/ECI_3
    2, // 7 P3.2/ADC2/RxD_2/INT0/SCLK/SCL/CCP0/CCP0_3
    3, // 8 P3.3/ADC3/TxD_2/INT1/MISO/SDA/CCP1/CCP1_2/CCP1_3
};

#endif
