#ifndef INC_USART_DMA_H_
#define INC_USART_DMA_H_

#include <stdint.h>

#include "stm32wbaxx_ll_usart.h"
#include "stm32wbaxx_ll_dma.h"

#define UART_TX_BUFFER_SIZE 256
#define UART_RX_BUFFER_SIZE 32 // TODO: increase after testing

// mask for indexing (power of two buffer size)
#define UART_RX_MASK (UART_RX_BUFFER_SIZE - 1U)

// return codes
#define UART_DMA_OK 0
#define UART_DMA_BUSY -1
#define UART_DMA_TOO_LARGE -2
#define UART_DMA_FAIL -3

// TX structure to hold info for one USART channel
typedef struct {
    USART_TypeDef *usart;  // e.g. USART2
    DMA_TypeDef *dma;      // e.g. GPDMA1
    uint32_t dma_channel;  // e.g. LL_DMA_CHANNEL_0
    uint32_t dma_request;  // e.g. LL_GPDMA1_REQUEST_USART2_TX

    volatile uint8_t busy; // 0 = idle, 1 = busy
    volatile uint8_t err;  // 0 = no error
    volatile size_t len;   // active transfer length
    uint8_t buf[UART_TX_BUFFER_SIZE];  // pointer to preallocated DMA-safe buffer
} uart_dma_tx_t;

// RX structure: internal buffer, masked indexing, monotonic head/tail counters
typedef struct {
	USART_TypeDef *usart;       // USART instance
	DMA_TypeDef *dma;           // DMA instance
	uint32_t dma_channel;       // DMA channel identifier
	uint32_t dma_request;       // DMA request mapping (if used)

	volatile uint32_t head;     // monotonic write counter
	volatile uint32_t tail;     // monotonic read counter
	uint32_t last_raw;          // last raw DMA index (0..N-1)

	volatile uint8_t new_data;  // set when ISR/idle reports new data
	volatile uint8_t burst_end; // set on IDLE interrupt
	volatile uint8_t overflow;  // set on overwrite

	uint8_t buf[UART_RX_BUFFER_SIZE]; // internal circular buffer
} uart_dma_rx_t;

void uart_dma_tx_init(uart_dma_tx_t *h, USART_TypeDef *usart, DMA_TypeDef *dma, uint32_t dma_channel, uint32_t dma_request);
void uart_dma_rx_init(uart_dma_rx_t *h, USART_TypeDef *usart, DMA_TypeDef *dma, uint32_t dma_channel, uint32_t dma_request);
void uart_dma_rx_start(uart_dma_rx_t *h);
void uart_dma_rx_rearm(uart_dma_rx_t *h);
int uart_dma_tx_send(uart_dma_tx_t *h, const uint8_t *data, size_t len);

size_t uart_rx_available(uart_dma_rx_t *h);
uint8_t uart_rx_peek(uart_dma_rx_t *h, uint32_t offset);
size_t uart_rx_read(uart_dma_rx_t *h, uint8_t *dst, size_t len);
void uart_rx_skip(uart_dma_rx_t *h, size_t n);

void uart_dma_tx_irq_handler(uart_dma_tx_t *h);
void uart_dma_rx_irq_handler(uart_dma_rx_t *h);
void uart_dma_rx_idle_irq_handler(uart_dma_rx_t *h);

#endif
