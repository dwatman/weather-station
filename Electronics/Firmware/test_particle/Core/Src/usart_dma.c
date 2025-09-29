#include <stdio.h> // for printf
#include <string.h> // for memcpy

#include "usart_dma.h"

// helper: read DMA hardware written position (0..capacity-1)
static inline uint32_t uart_dma_rx_hw_pos(uart_dma_rx_t *h) {
	uint32_t rem = LL_DMA_GetBlkDataLength(h->dma, h->dma_channel);
	return (uint32_t)(UART_RX_BUFFER_SIZE - rem) & UART_RX_MASK;
}

// update head by delta between raw and last_raw (masked)
static inline void uart_dma_rx_update_head_by_delta(uart_dma_rx_t *h, uint32_t raw) {
	uint32_t delta = (raw - h->last_raw) & UART_RX_MASK;
	if (delta != 0U) {
		h->last_raw = raw;
		h->head += delta; // monotonic

		// detect overwrite: if available >= capacity, advance tail and mark overflow
		uint32_t available = h->head - h->tail;
		if (available >= UART_RX_BUFFER_SIZE) {
			// advance tail to retain newest data (keep full buffer of newest bytes)
			h->tail = h->head - (UART_RX_BUFFER_SIZE - 1U);
			h->overflow = 1U;
		}

		h->new_data = 1U;
	}
}

void uart_dma_tx_init(uart_dma_tx_t *h, USART_TypeDef *usart, DMA_TypeDef *dma, uint32_t dma_channel, uint32_t dma_request) {
	h->usart = usart;
	h->dma = dma;
	h->dma_channel = dma_channel;
	h->dma_request = dma_request;
	h->busy = 0;
	h->err = 0;
	h->len = 0;

	// Ensure DMA channel is disabled before config
	LL_DMA_DisableChannel(h->dma, h->dma_channel);

	// Program static DMA parameters that do not change per transfer
	LL_DMA_SetDataTransferDirection(h->dma, h->dma_channel, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetPeriphRequest(h->dma, h->dma_channel, h->dma_request);
	LL_DMA_SetChannelPriorityLevel(h->dma, h->dma_channel, LL_DMA_LOW_PRIORITY_MID_WEIGHT);

	// Source parameters
	LL_DMA_SetSrcIncMode(h->dma, h->dma_channel, LL_DMA_SRC_INCREMENT);
	LL_DMA_SetSrcDataWidth(h->dma, h->dma_channel, LL_DMA_SRC_DATAWIDTH_BYTE);

	// Destination parameters
	LL_DMA_SetDestAddress(h->dma, h->dma_channel, (uint32_t)&h->usart->TDR);
	LL_DMA_SetDestIncMode(h->dma, h->dma_channel, LL_DMA_DEST_FIXED);
	LL_DMA_SetDestDataWidth(h->dma, h->dma_channel, LL_DMA_DEST_DATAWIDTH_BYTE);

	// Clear interrupt flags
	LL_DMA_ClearFlag_TC(h->dma, h->dma_channel); // Transfer complete
	LL_DMA_ClearFlag_USE(h->dma, h->dma_channel); // User setting error
	LL_DMA_ClearFlag_ULE(h->dma, h->dma_channel); // Link transfer error
	LL_DMA_ClearFlag_DTE(h->dma, h->dma_channel); // Data transfer error

	// Enable transfer-complete and error interrupts
	LL_DMA_EnableIT_TC(h->dma, h->dma_channel);
	LL_DMA_EnableIT_USE(h->dma, h->dma_channel);
	LL_DMA_EnableIT_ULE(h->dma, h->dma_channel);
	LL_DMA_EnableIT_DTE(h->dma, h->dma_channel);
}

void uart_dma_rx_init(uart_dma_rx_t *h, USART_TypeDef *usart, DMA_TypeDef *dma, uint32_t dma_channel, uint32_t dma_request) {
	h->usart = usart;
	h->dma = dma;
	h->dma_channel = dma_channel;
	h->dma_request = dma_request;

	h->head = 0;
	h->tail = 0;
	h->last_raw = 0;
	h->new_data = 0;
	h->burst_end = 0;
	h->overflow = 0;

	// Ensure DMA channel is disabled before config
	LL_DMA_DisableChannel(h->dma, h->dma_channel);

	// Program static DMA parameters that do not change per transfer (circular mode expected)
	LL_DMA_SetDataTransferDirection(h->dma, h->dma_channel, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetPeriphRequest(h->dma, h->dma_channel, h->dma_request);
	LL_DMA_SetChannelPriorityLevel(h->dma, h->dma_channel, LL_DMA_LOW_PRIORITY_MID_WEIGHT);

	// Source parameters
	LL_DMA_SetSrcAddress(h->dma, h->dma_channel, (uint32_t)&h->usart->RDR);
	LL_DMA_SetSrcIncMode(h->dma, h->dma_channel, LL_DMA_SRC_FIXED);
	LL_DMA_SetSrcDataWidth(h->dma, h->dma_channel, LL_DMA_SRC_DATAWIDTH_BYTE);

	// Destination parameters
	LL_DMA_SetDestAddress(h->dma, h->dma_channel, (uint32_t)h->buf);
	LL_DMA_SetDestIncMode(h->dma, h->dma_channel, LL_DMA_DEST_INCREMENT);
	LL_DMA_SetDestDataWidth(h->dma, h->dma_channel, LL_DMA_DEST_DATAWIDTH_BYTE);

	// set block length to buffer capacity
	LL_DMA_SetBlkDataLength(h->dma, h->dma_channel, (uint32_t)UART_RX_BUFFER_SIZE);

	// Clear interrupt flags
	LL_DMA_ClearFlag_HT(h->dma, h->dma_channel); // Transfer half complete
	LL_DMA_ClearFlag_TC(h->dma, h->dma_channel); // Transfer complete
	LL_DMA_ClearFlag_USE(h->dma, h->dma_channel); // User setting error
	LL_DMA_ClearFlag_ULE(h->dma, h->dma_channel); // Link transfer error
	LL_DMA_ClearFlag_DTE(h->dma, h->dma_channel); // Data transfer error

	// Enable HT and TC and error interrupts
	LL_DMA_EnableIT_HT(h->dma, h->dma_channel);
	LL_DMA_EnableIT_TC(h->dma, h->dma_channel);
	LL_DMA_EnableIT_USE(h->dma, h->dma_channel);
	LL_DMA_EnableIT_ULE(h->dma, h->dma_channel);
	LL_DMA_EnableIT_DTE(h->dma, h->dma_channel);

	// Enable USART IDLE interrupt so short bursts are detected
	LL_USART_ClearFlag_IDLE(h->usart);
	LL_USART_EnableIT_IDLE(h->usart);
}

// start circular RX by enabling DMA channel and USART DMA request
void uart_dma_rx_start(uart_dma_rx_t *h) {
	// reset state
	h->head = 0;
	h->tail = 0;
	h->last_raw = 0;
	h->new_data = 0;
	h->burst_end = 0;
	h->overflow = 0;

	// Ensure DMA flags cleared
	LL_DMA_ClearFlag_TC(h->dma, h->dma_channel);
	LL_DMA_ClearFlag_HT(h->dma, h->dma_channel);

	// set destination address and block length (re-arm)
	LL_DMA_SetSrcAddress(h->dma, h->dma_channel, (uint32_t)&h->usart->RDR);
	LL_DMA_SetDestAddress(h->dma, h->dma_channel, (uint32_t)h->buf);
	LL_DMA_SetBlkDataLength(h->dma, h->dma_channel, (uint32_t)UART_RX_BUFFER_SIZE);

	// enable USART DMA request
	LL_USART_EnableDMAReq_RX(h->usart);

	// enable DMA channel to start circular reception
	LL_DMA_EnableChannel(h->dma, h->dma_channel);
}

int uart_dma_tx_send(uart_dma_tx_t *h, const uint8_t *data, size_t len) {
    if (h == NULL || data == NULL) return UART_DMA_FAIL;
    if (len == 0) return 0;

    if (len > sizeof(h->buf)) return UART_DMA_TOO_LARGE;

    // Critical section:
    // Check-and-set busy must be atomic with respect to DMA IRQ
    uint32_t primask = __get_PRIMASK(); // save PRIMASK
    __disable_irq(); // disable interrupts
    if (h->busy) {
        __set_PRIMASK(primask); // restore
        return UART_DMA_BUSY;
    }
    h->busy = 1;
    __set_PRIMASK(primask); // restore interrupts

    // copy payload into internal buffer
    memcpy(h->buf, data, len);
    h->len = len;

    // Clear flags for this transfer
    LL_DMA_ClearFlag_TC(h->dma, h->dma_channel);
    LL_DMA_ClearFlag_USE(h->dma, h->dma_channel);
    LL_DMA_ClearFlag_ULE(h->dma, h->dma_channel);
    LL_DMA_ClearFlag_DTE(h->dma, h->dma_channel);

    // Update memory address and data length only
    LL_DMA_SetSrcAddress(h->dma, h->dma_channel, (uint32_t)h->buf);
    LL_DMA_SetBlkDataLength(h->dma, h->dma_channel, (uint32_t)len);

    // Ensure USART TX DMA request is enabled
    LL_USART_EnableDMAReq_TX(h->usart);

    // Enable DMA channel to start transfer
    LL_DMA_EnableChannel(h->dma, h->dma_channel);

    return len;
}

// available bytes (snapshot)
size_t uart_rx_available(uart_dma_rx_t *h) {
	uint32_t raw;
	uint32_t head, tail;

	uint32_t prim = __get_PRIMASK();
	__disable_irq();

	raw = uart_dma_rx_hw_pos(h);
	uart_dma_rx_update_head_by_delta(h, raw);
	head = h->head;
	tail = h->tail;

	__set_PRIMASK(prim);

	return (size_t)(head - tail);
}

// peek single byte at offset from tail (returns 0 if empty)
uint8_t uart_rx_peek(uart_dma_rx_t *h, uint32_t offset) {
	uint32_t raw;
	uint32_t head, tail;

	uint32_t prim = __get_PRIMASK();
	__disable_irq();

	raw = uart_dma_rx_hw_pos(h);
	uart_dma_rx_update_head_by_delta(h, raw);
	head = h->head;
	tail = h->tail;

	__set_PRIMASK(prim);

	uint32_t available = head - tail;
	if (offset >= available) return 0U;

	uint32_t idx = (tail + offset) & UART_RX_MASK;
	return h->buf[idx];
}

// read up to len bytes into dst, advance tail
size_t uart_rx_read(uart_dma_rx_t *h, uint8_t *dst, size_t len) {
	uint32_t raw;
	uint32_t head, tail;

	uint32_t prim = __get_PRIMASK();
	__disable_irq();

	raw = uart_dma_rx_hw_pos(h);
	uart_dma_rx_update_head_by_delta(h, raw);
	head = h->head;
	tail = h->tail;

	uint32_t available = head - tail;
	uint32_t to_copy = (uint32_t)len;
	if (to_copy > available) to_copy = available;

	h->tail = tail + to_copy;

	// Update new_data flag: clear if we've consumed everything
	h->new_data = (h->head != h->tail) ? 1U : 0U;

	__set_PRIMASK(prim);

	uint32_t first_idx = tail & UART_RX_MASK;
	uint32_t first_chunk = UART_RX_BUFFER_SIZE - first_idx;
	if (first_chunk > to_copy) first_chunk = to_copy;

	memcpy(dst, &h->buf[first_idx], first_chunk);
	if (to_copy > first_chunk) {
		memcpy(dst + first_chunk, &h->buf[0], to_copy - first_chunk);
	}

	return (size_t)to_copy;
}

void uart_rx_skip(uart_dma_rx_t *h, size_t n) {
	uint32_t raw;
	uint32_t head, tail;

	uint32_t prim = __get_PRIMASK();
	__disable_irq();

	raw = uart_dma_rx_hw_pos(h);
	uart_dma_rx_update_head_by_delta(h, raw);
	head = h->head;
	tail = h->tail;

	uint32_t available = head - tail;
	uint32_t to_skip = (uint32_t)n;
	if (to_skip > available) to_skip = available;

	h->tail = tail + to_skip;

	// update new_data flag: clear if we've consumed everything
	h->new_data = (h->head != h->tail) ? 1U : 0U;

	__set_PRIMASK(prim);
}


// *** ISRs ***

void uart_dma_tx_irq_handler(uart_dma_tx_t *h) {
	if (h == NULL) return;

	// transfer complete
	if (LL_DMA_IsActiveFlag_TC(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_TC(h->dma, h->dma_channel);

		// disable channel and USART DMA request
		LL_DMA_DisableChannel(h->dma, h->dma_channel);
		LL_USART_DisableDMAReq_TX(h->usart);

		// clear busy and len
		h->busy = 0;
		h->len = 0;
		h->err = 0;
	}

	// transfer error variants
	if (LL_DMA_IsActiveFlag_USE(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_USE(h->dma, h->dma_channel);
		LL_DMA_DisableChannel(h->dma, h->dma_channel);
		LL_USART_DisableDMAReq_TX(h->usart);
		h->busy = 0;
		h->len = 0;
		h->err = 1;
		printf("INT TX USE\n");
	}
	if (LL_DMA_IsActiveFlag_ULE(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_ULE(h->dma, h->dma_channel);
		LL_DMA_DisableChannel(h->dma, h->dma_channel);
		LL_USART_DisableDMAReq_TX(h->usart);
		h->busy = 0;
		h->len = 0;
		h->err = 2;
		printf("INT TX ULE\n");
	}
	if (LL_DMA_IsActiveFlag_DTE(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_DTE(h->dma, h->dma_channel);
		LL_DMA_DisableChannel(h->dma, h->dma_channel);
		LL_USART_DisableDMAReq_TX(h->usart);
		h->busy = 0;
		h->len = 0;
		h->err = 3;
		printf("INT TX DTE\n");
	}
}

void uart_dma_rx_irq_handler(uart_dma_rx_t *h) {
	// Transfer half complete
	if (LL_DMA_IsActiveFlag_HT(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_HT(h->dma, h->dma_channel);
		uint32_t raw = UART_RX_BUFFER_SIZE / 2U;
		uart_dma_rx_update_head_by_delta(h, raw);
		printf("INT RX HT\n");
	}
	// Transfer complete
	if (LL_DMA_IsActiveFlag_TC(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_TC(h->dma, h->dma_channel);
		uint32_t raw = 0U;
		uart_dma_rx_update_head_by_delta(h, raw);
		printf("INT RX TC\n");
	}
	// User setting error
	if (LL_DMA_IsActiveFlag_USE(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_USE(h->dma, h->dma_channel);
		h->overflow = 1U;
		LL_DMA_DisableChannel(h->dma, h->dma_channel);
		LL_USART_DisableDMAReq_RX(h->usart);
		printf("INT RX USE\n");
	}
	// Link transfer error
	if (LL_DMA_IsActiveFlag_ULE(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_ULE(h->dma, h->dma_channel);
		h->overflow = 1U;
		LL_DMA_DisableChannel(h->dma, h->dma_channel);
		LL_USART_DisableDMAReq_RX(h->usart);
		printf("INT RX ULE\n");
	}
	// Data transfer error
	if (LL_DMA_IsActiveFlag_DTE(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_DTE(h->dma, h->dma_channel);
		h->overflow = 1U;
		LL_DMA_DisableChannel(h->dma, h->dma_channel);
		LL_USART_DisableDMAReq_RX(h->usart);
		printf("INT RX DTE\n");
	}
}

// USART IDLE: sample hardware position and update head
void uart_dma_rx_idle_irq_handler(uart_dma_rx_t *h) {
	LL_USART_ClearFlag_IDLE(h->usart);

	uint32_t raw = uart_dma_rx_hw_pos(h);
	uart_dma_rx_update_head_by_delta(h, raw);
	h->burst_end = 1U;
}
