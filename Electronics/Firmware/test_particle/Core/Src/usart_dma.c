#include <string.h>

#include "usart_dma.h"

void uart_dma_init(uart_dma_tx_t *h, USART_TypeDef *usart, DMA_TypeDef *dma, uint32_t dma_channel, uint32_t dma_request) {
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

int uart_dma_send(uart_dma_tx_t *h, const uint8_t *data, size_t len) {
    if (h == NULL || data == NULL) return UART_DMA_FAIL;
    if (len == 0) return UART_DMA_OK;

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

    return UART_DMA_OK;
}

void uart_dma_irq_handler(uart_dma_tx_t *h) {
	if (h == NULL) return;

	// transfer complete?
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
	}
	if (LL_DMA_IsActiveFlag_ULE(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_ULE(h->dma, h->dma_channel);
		LL_DMA_DisableChannel(h->dma, h->dma_channel);
		LL_USART_DisableDMAReq_TX(h->usart);
		h->busy = 0;
		h->len = 0;
		h->err = 2;
	}
	if (LL_DMA_IsActiveFlag_DTE(h->dma, h->dma_channel)) {
		LL_DMA_ClearFlag_DTE(h->dma, h->dma_channel);
		LL_DMA_DisableChannel(h->dma, h->dma_channel);
		LL_USART_DisableDMAReq_TX(h->usart);
		h->busy = 0;
		h->len = 0;
		h->err = 3;
	}
}
