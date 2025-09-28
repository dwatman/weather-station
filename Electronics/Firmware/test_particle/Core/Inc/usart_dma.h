#ifndef INC_USART_DMA_H_
#define INC_USART_DMA_H_

#include <stdint.h>

#include "stm32wbaxx_ll_usart.h"
#include "stm32wbaxx_ll_dma.h"

#define UART2_TX_BUFFER_SIZE 256

// return codes
#define UART_DMA_OK 0
#define UART_DMA_BUSY -1
#define UART_DMA_TOO_LARGE -2
#define UART_DMA_FAIL -3

// Structure to hold info for one USART channel
typedef struct {
    USART_TypeDef *usart;               // e.g. USART2
    DMA_TypeDef *dma;                  // e.g. GPDMA1
    uint32_t dma_channel;              // e.g. LL_DMA_CHANNEL_0
    uint32_t dma_request;              // e.g. LL_GPDMA1_REQUEST_USART2_TX
    volatile uint8_t busy;             // 0 = idle, 1 = busy
    volatile uint8_t err;              // 0 = no error
    volatile size_t len;               // active transfer length
    uint8_t buf[UART2_TX_BUFFER_SIZE]; // pointer to preallocated DMA-safe buffer
} uart_dma_tx_t;

void uart_dma_init(uart_dma_tx_t *h, USART_TypeDef *usart, DMA_TypeDef *dma, uint32_t dma_channel, uint32_t dma_request);
int uart_dma_send(uart_dma_tx_t *h, const uint8_t *data, size_t len);
void uart_dma_irq_handler(uart_dma_tx_t *h);

#endif
