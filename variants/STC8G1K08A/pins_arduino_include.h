#ifndef Pins_Arduino_Include_h
#define Pins_Arduino_Include_h

#ifndef _BV
#define _BV(X) (1 << (X))
#endif

#define NOT_A_PIN 0
#define NOT_A_PORT 0

#define P0PORT 1
#define P1PORT 2
#define P2PORT 3
#define P3PORT 4
#define P4PORT 5

#define NOT_AN_INTERRUPT -1
#define NOT_ANALOG 255

extern __code uint8_t digital_pin_to_port_PGM[];
extern __code uint8_t digital_pin_to_bit_mask_PGM[];

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
//
#define digitalPinToPort(P) (digital_pin_to_port_PGM[(P)])
#define digitalPinToBitMask(P) (digital_pin_to_bit_mask_PGM[(P)])

#endif
