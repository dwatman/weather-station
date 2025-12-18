#include <stdio.h> 		// For vsnprintf
#include <stdarg.h> 	// For va_list, va_start, va_end

#include "usart_util.h"
#include "circbuf.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_dma.h"

CircularBuffer_t tx1Buf;
CircularBuffer_t rx1Buf;
static CircularBuffer_t tx4Buf;

volatile uint32_t RxNewData = 0;
volatile uint32_t RxBurstEnd = 0;
volatile uint32_t RxOverflow = 0;

void initUsart1(void) {
	USART_TypeDef *uchannel = USART1;
	DMA_TypeDef *ddev = DMA1;
	uint32_t dstream = LL_DMA_STREAM_0;

	initCircBuffer(&tx1Buf);
	initCircBuffer(&rx1Buf);

	// Clear flags
	RxOverflow = 0;
	RxBurstEnd = 0;
	RxNewData = 0;

	// Set up DMA for receive
	LL_USART_EnableDMAReq_RX(uchannel);

	// Make sure stream is disabled for setup
	if (LL_DMA_IsEnabledStream(ddev, dstream)) {
		LL_DMA_DisableStream(ddev, dstream);
		while (LL_DMA_IsEnabledStream(ddev, dstream)) { } // wait
	}

	// Configure transfer
	LL_DMA_SetPeriphRequest(ddev, dstream, LL_DMAMUX1_REQ_USART1_RX);
	LL_DMA_SetPeriphAddress(ddev, dstream, (uint32_t)&(uchannel->RDR));
	LL_DMA_SetMemoryAddress(ddev, dstream, (uint32_t)(rx1Buf.data));
	LL_DMA_SetDataLength(ddev, dstream, BUF_LENGTH);
	LL_DMA_SetMode(ddev, dstream, LL_DMA_MODE_CIRCULAR);

	LL_DMA_SetPeriphSize(ddev, dstream, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(ddev, dstream, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetDataTransferDirection(ddev, dstream, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	LL_DMA_SetPeriphIncMode(ddev, dstream, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(ddev, dstream, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_EnableStream(ddev, dstream);

	// Ensure old data is ignored if DMA has been in use previously
	uint32_t write_pos = (BUF_LENGTH - LL_DMA_GetDataLength(ddev, dstream)) & BUF_MASK;
	rx1Buf.head = write_pos;
	rx1Buf.tail = write_pos;

	// Clear flag then enable USART IDLE interrupt
	LL_USART_ClearFlag_IDLE(uchannel);
	LL_USART_EnableIT_IDLE(uchannel);

	// Enable DMA interrupts
	LL_DMA_EnableIT_HT(ddev, dstream); // half-transfer
	LL_DMA_EnableIT_TC(ddev, dstream); // transfer complete
	//LL_DMA_EnableIT_TE(ddev, dstream); // transfer error
}

void initUsart4(void) {
	initCircBuffer(&tx4Buf);

}

// Ignore any stored data by moving the tail up to the head
void emptyRx1Buffer(void) {
	emptyCircBuffer(&rx1Buf);

	// Clear flags
	RxOverflow = 0;
	RxBurstEnd = 0;
	RxNewData = 0;
}

uint32_t printfCircBuf(CircularBuffer_t *buf, const char *format, ...) {
    char tempBuf[PRINT_BUF_LENGTH];
    va_list args;

    va_start(args, format);
    int len = vsnprintf(tempBuf, sizeof(tempBuf), format, args);
    va_end(args);

    if (len < 0)
        return 0;  // Formatting error

    // Limit to buffer size
    uint32_t toWrite = (len < PRINT_BUF_LENGTH) ? len : PRINT_BUF_LENGTH - 1;

    // Add to circular transmit buffer
    uint32_t written = writeToCircBuf(buf, (const uint8_t *)tempBuf, toWrite);

	// Make sure interrupt is enabled if there is data to send
	if (!isCircBufEmpty(buf))
		LL_USART_EnableIT_TXFE(USART1);

	return written;
}

void USART1_ISR(void) {
	uint8_t ch;

	// Check TX FIFO threshold interrupt
	if (LL_USART_IsActiveFlag_TXFE(USART1) && LL_USART_IsEnabledIT_TXFE(USART1)) {
		// Keep filling FIFO until full or buffer empty
		while (LL_USART_IsActiveFlag_TXE_TXFNF(USART1) && !isCircBufEmpty(&tx1Buf)) {
			if (readFromCircBuf(&tx1Buf, &ch))
				LL_USART_TransmitData8(USART1, ch);
		}

		// Disable interrupt if buffer is empty
		if (isCircBufEmpty(&tx1Buf))
			LL_USART_DisableIT_TXFE(USART1);
	}

	// Check IDLE interrupt
	if (LL_USART_IsActiveFlag_IDLE(USART1) && LL_USART_IsEnabledIT_IDLE(USART1)) {
		LL_USART_ClearFlag_IDLE(USART1);

		// Get DMA pointer position
		uint32_t new_pos = (uint32_t)(BUF_LENGTH - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_0));

		// Set correct head position and check if buffer has overflowed
		int ovf = advanceCircBufByDMApos(&rx1Buf, new_pos);

		RxNewData = (inCircBuf(&rx1Buf) != 0);
		RxBurstEnd = 1;
		RxOverflow |= ovf;

		//printf("IDLE\n");
	}
}

void UART4_ISR(void) {
	uint8_t ch;

	// Check TX FIFO threshold interrupt
	if (LL_USART_IsActiveFlag_TXFE(UART4) && LL_USART_IsEnabledIT_TXFE(UART4)) {
		// Keep filling FIFO until full or buffer empty
		while (LL_USART_IsActiveFlag_TXE_TXFNF(UART4) && !isCircBufEmpty(&tx4Buf)) {
			if (readFromCircBuf(&tx4Buf, &ch))
				LL_USART_TransmitData8(UART4, ch);
		}

		// Disable interrupt if buffer is empty
		if (isCircBufEmpty(&tx4Buf))
			LL_USART_DisableIT_TXFE(UART4);
	}
}

// Redirect printf tu UART4
int _write(int file, char *ptr, int len) {
	(void)file;
	int written = (int)writeToCircBuf(&tx4Buf, (uint8_t *)ptr, len);

	// Make sure interrupt is enabled if there is data to send
	if (!isCircBufEmpty(&tx4Buf))
		LL_USART_EnableIT_TXFE(UART4);

	return written;
}
