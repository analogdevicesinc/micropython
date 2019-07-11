/*
 * Copyright (c) 2018 Analog Devices, Inc.  All rights reserved.
 *
 * UART Simple provides a simple, bare-metal UART driver
 */

#ifndef _BM_UART_H
#define _BM_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef F_SCLK0
#ifdef CLOCK_SPEED_500MHZ
#define F_SCLK0 (125000000)
#else
#define F_SCLK0 (112500000)
#endif
#endif

typedef enum _BM_UART_BAUD_RATE {
    UART_BAUD_RATE_110     = (F_SCLK0 / 16 / 110),
    UART_BAUD_RATE_300     = (F_SCLK0 / 16 / 300),
    UART_BAUD_RATE_600     = (F_SCLK0 / 16 / 600),
    UART_BAUD_RATE_1200    = (F_SCLK0 / 16 / 1200),
    UART_BAUD_RATE_2400    = (F_SCLK0 / 16 / 2400),
    UART_BAUD_RATE_4800    = (F_SCLK0 / 16 / 4800),
    UART_BAUD_RATE_9600    = (F_SCLK0 / 16 / 9600),
    UART_BAUD_RATE_14400   = (F_SCLK0 / 16 / 14400),
    UART_BAUD_RATE_19200   = (F_SCLK0 / 16 / 19200),
    UART_BAUD_RATE_28800   = (F_SCLK0 / 16 / 28800),
    UART_BAUD_RATE_31250   = (F_SCLK0 / 16 / 31250),
    UART_BAUD_RATE_MIDI    = (F_SCLK0 / 16 / 31250),
    UART_BAUD_RATE_38400   = (F_SCLK0 / 16 / 38400),
    UART_BAUD_RATE_56000   = (F_SCLK0 / 16 / 56000),
    UART_BAUD_RATE_57600   = (F_SCLK0 / 16 / 57600),
    UART_BAUD_RATE_115200  = (F_SCLK0 / 16 / 115200),
    UART_BAUD_RATE_230400  = (F_SCLK0 / 16 / 230400),
    UART_BAUD_RATE_256000  = (F_SCLK0 / 16 / 256000),
    UART_BAUD_RATE_460800  = (F_SCLK0 / 16 / 460800),
    UART_BAUD_RATE_921600  = (F_SCLK0 / 16 / 921600),
    UART_BAUD_RATE_FASTEST = 1   // 112.5MHz/(16 * 1) = 7,031,250 Baud
} BM_UART_BAUD_RATE;

// word len / parity / stop bits presets - these map directly into bits 8-15 of the UART control register
typedef enum _BM_UART_CONFIG {
    UART_SERIAL_8N1 = 0x3
} BM_UART_CONFIG;

typedef enum _BM_UART_PERIPHERAL_NUMBER {
    UART0 = (0),
    UART1 = (1),
    UART2 = (2)
} BM_UART_PERIPHERAL_NUMBER;

// SHARC Audio Module / MIDI FIN presets for UART device number
#define UART_SAM_DEVICE_FTDI    (UART0)
#define UART_AUDIOPROJ_DEVICE_MIDI    (UART1)

// Size of the UART TX and RX buffers / FIFOs
#define UART_BUFFER_SIZE        (1024)

// Results from UART operations
typedef enum
{
    UART_SUCCESS,              // The API call is success
    UART_TX_FIFO_FULL,         // The TX FIFO is full
    UART_TX_FIFO_EMPTY,        // The TX FIFO is empty
    UART_RX_FIFO_FULL,         // The RX FIFO is full
    UART_RX_FIFO_EMPTY,        // The RX FIFO is empty
    UART_INVALID_DEVICE,       // Not a valid UART peripheral
} BM_UART_RESULT;

// Structure definition for UART driver
typedef struct _BM_UART
{

    // pointers into the various control registers
    volatile uint32_t *pREG_UART_CTL;
    volatile uint32_t *pREG_UART_CLK;
    volatile uint32_t *pREG_UART_STAT;
    volatile uint32_t *pREG_UART_THR;
    volatile uint32_t *pREG_UART_RBR;
    volatile uint32_t *pREG_UART_IMSK_CLR;
    volatile uint32_t *pREG_UART_IMSK_SET;

    // UART receive buffer
    uint8_t rx_buffer[UART_BUFFER_SIZE];
    uint16_t rx_buffer_readptr;
    uint16_t rx_buffer_writeptr;

    // UART transmit buffer
    uint8_t tx_buffer[UART_BUFFER_SIZE];
    uint16_t tx_buffer_readptr;
    uint16_t tx_buffer_writeptr;

    // initialization parameter
    uint8_t device_num;
    uint32_t baud;
    uint8_t config;

    void (*rx_callback_function)(void);
} BM_UART;

#ifdef __cplusplus
extern "C" {
#endif

// Starts the driver
BM_UART_RESULT uart_initialize(BM_UART *device,
                               BM_UART_BAUD_RATE baud,
                               BM_UART_CONFIG config,
                               BM_UART_PERIPHERAL_NUMBER device_num);

// Reads a byte from the RX buffer / FIFO
BM_UART_RESULT uart_read_byte(BM_UART *device,
                              uint8_t *value);

// Writes a single byte to the UART TX buffer / FIFO
BM_UART_RESULT uart_write_byte(BM_UART *device,
                               uint8_t tx_byte);

// Writes a block to the UART TX buffer / FIFO
BM_UART_RESULT uart_write_block(BM_UART *device,
                                uint8_t *tx_bytes,
                                uint16_t len);

// Sets a callback function
void uart_set_rx_callback(BM_UART *device,
                          void (*callback)(void) );

// Clears callback function
void uart_clear_rx_callback(BM_UART *device);

// Returns number of bytes in RX buffer / FIFO
uint16_t uart_available(BM_UART *device);

// Returns amount of space in TX buffer / FIFO
uint16_t uart_available_for_write(BM_UART *device);

#ifdef __cplusplus
} // extern "C"
#endif

#endif  // _BM_UART_H
