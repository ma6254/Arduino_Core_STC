#ifndef HardwareSerial_h
#define HardwareSerial_h

#define SERIAL0_RX_BUFFER_SIZE 32
#define SERIAL0_TX_BUFFER_SIZE 32

typedef uint8_t (*_Fn_HwSerial_write)(uint8_t SendDat);
typedef void (*_Fn_HwSerial_begin)(unsigned long baud);
typedef void (*_Fn_HwSerial_flush)(void);
typedef int (*_Fn_HwSerial_print)(const char *format, ...);

typedef struct
{
    _Fn_HwSerial_write write;
    _Fn_HwSerial_begin begin;
    _Fn_HwSerial_flush flush;
    _Fn_HwSerial_print print;
} Serial_t;

extern Serial_t Serial0;

void _Int_UART0_Rx_Handler();
void _Int_UART0_Tx_Handler();

void _HwSerialInit(void);

#endif
