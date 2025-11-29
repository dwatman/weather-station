#ifndef APPLICATION_USER_INCLUDE_CIRCBUF_H_
#define APPLICATION_USER_INCLUDE_CIRCBUF_H_

#include <stdint.h>
#include <stdbool.h>

#define BUF_LENGTH 1024U // Must be a power of 2
#define BUF_MASK ((uint32_t)BUF_LENGTH - 1U)

#define PRINT_BUF_LENGTH 256U

typedef struct {
	uint8_t data[BUF_LENGTH];
	volatile uint32_t head;
	volatile uint32_t tail;
} CircularBuffer_t;

void initCircBuffer(CircularBuffer_t *buf);
bool isCircBufFull(const CircularBuffer_t *buf);
bool isCircBufEmpty(const CircularBuffer_t *buf);
uint32_t inCircBuf(const CircularBuffer_t *buf);
uint32_t writeToCircBuf(CircularBuffer_t *buf, const uint8_t *str, uint32_t length);
uint8_t peekCircBuf(const CircularBuffer_t *buf, int position);
int readFromCircBuf(CircularBuffer_t *buf, uint8_t *ch);
uint32_t printfCircBuf(CircularBuffer_t *buf, const char *format, ...);
int advanceCircBufByDMApos(CircularBuffer_t *buf, uint32_t dma_write_pos);

#endif
