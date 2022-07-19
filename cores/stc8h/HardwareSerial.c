#include <stdarg.h>

#include "Arduino.h"

#include "HardwareSerial.h"

Serial_t Serial0;

__xdata uint8_t Receive_Uart0_Buf[SERIAL0_RX_BUFFER_SIZE];
__xdata uint8_t Transmit_Uart0_Buf[SERIAL0_TX_BUFFER_SIZE];
volatile __xdata uint8_t uart0_rx_buffer_head = 0;
volatile __xdata uint8_t uart0_rx_buffer_tail = 0;
volatile __xdata uint8_t uart0_tx_buffer_head = 0;
volatile __xdata uint8_t uart0_tx_buffer_tail = 0;
volatile __bit uart0_flag_sending = 0;

void _Int_UART0_Rx_Handler()
{
    uint8_t nextHead = (uart0_rx_buffer_head + 1) % SERIAL0_RX_BUFFER_SIZE;

    if (nextHead != uart0_rx_buffer_tail)
    {
        Receive_Uart0_Buf[uart0_rx_buffer_head] = SBUF;
        uart0_rx_buffer_head = nextHead;
    }
}

void _Int_UART0_Tx_Handler()
{
    if (uart0_flag_sending)
    {
        if (uart0_tx_buffer_head == uart0_tx_buffer_tail)
        {
            // do no more
            uart0_flag_sending &= 0;
        }
        else
        {
            SBUF = Transmit_Uart0_Buf[uart0_tx_buffer_tail];
            uart0_tx_buffer_tail = (uart0_tx_buffer_tail + 1) % SERIAL0_TX_BUFFER_SIZE;
        }
    }
}

void _HwSerial0_begin(unsigned long baud)
{
    uint32_t timeer_set = 0;

    pinMode(PIN_UART0_TXD, OUTPUT);
    pinMode(PIN_UART0_RXD, INPUT_PULLUP);

    Set_UART0_clk_div2();
    Set_UART0_clkgen_T1();
    Set_Timer1_1T();

    SCON = 0x50;
    PCON = 0x00;

    timeer_set = 65535 - F_CPU / (baud * 4);
    TL1 = (uint8_t)(timeer_set);
    TH1 = (uint8_t)(timeer_set >> 8);

    ES = 1;
    Set_Timer1_Enable();
}

uint8_t _HwSerial0_write(uint8_t SendDat)
{
    uint8_t interruptOn = EA;
    EA = 0;

    if (interruptOn)
        EA = 1;

    // start to send
    if ((uart0_tx_buffer_head == uart0_tx_buffer_tail) && (uart0_flag_sending == 0))
    {
        uart0_flag_sending = 1;
        SBUF = SendDat;
        if (interruptOn)
            EA = 1;
        return 1;
    }

    uint8_t nextHeadPos = ((uint8_t)(uart0_tx_buffer_head + 1)) % SERIAL0_TX_BUFFER_SIZE;

    // wait max 100ms or discard
    uint16_t waitWriteCount = 0;
    while ((nextHeadPos == uart0_tx_buffer_tail))
    {
        if (interruptOn)
            EA = 1;
        delay(10);
        waitWriteCount++;
        if (waitWriteCount >= 10)
            return 0;
    }
    Transmit_Uart0_Buf[uart0_tx_buffer_head] = SendDat;

    uart0_tx_buffer_head = nextHeadPos;

    return 1;
}

void _HwSerial0_flush(void)
{
    while (uart0_flag_sending)
        ;
}

void _HwSerial_putchar(char c, void *p)
{
    __xdata uint8_t serial_port = (uint8_t)(p);

    switch (serial_port)
    {
    case 0:
        _HwSerial0_write(c);
        break;
    }
}

int _HwSerial0_print(const char *format, ...)
{
    va_list argptr;
    int cnt;

    va_start(argptr, format);
    cnt = _print_format(_HwSerial_putchar, 0, format, argptr);
    va_end(argptr);

    return cnt;
}

void _HwSerialInit(void)
{
    Serial0.write = _HwSerial0_write;
    Serial0.begin = _HwSerial0_begin;
    Serial0.flush = _HwSerial0_flush;
    Serial0.print = _HwSerial0_print;
}
