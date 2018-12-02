#ifndef __SERIAL_UART_IMPL_H
#define __SERIAL_UART_IMPL_H

#include <stdint.h>
#include "serial.h"
#include "serial_uart.h"
#include "target.h"
#include "RCCTypes.h"
#include "io.h"
#include "rcc.h"

#define UART_RX_BUFFER_SIZE 512
#define UART_TX_BUFFER_SIZE 512

typedef struct uartDevice_s {
    USART_TypeDef* dev;
    uartPort_t port;
    uint32_t DMAChannel;
    DMA_Stream_TypeDef *txDMAStream;
    DMA_Stream_TypeDef *rxDMAStream;
    ioTag_t rx;
    ioTag_t tx;
    volatile uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
    volatile uint8_t txBuffer[UART_TX_BUFFER_SIZE];
    RccPeriphTag_t rcc_uart;
    uint8_t af;
    uint8_t rxIrq;
    uint32_t txPriority;
    uint32_t rxPriority;
} uartDevice_t;

extern const struct serialPortVTable uartVTable[];

uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUART6(uint32_t baudRate, portMode_t mode, portOptions_t options);

#endif	// __SERIAL_UART_IMPL_H
