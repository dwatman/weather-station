/*
 * circbuf.c
 * Circular buffer for sending UART messages
 * <head> points to the next empty position
 * <tail> points to the oldest valid data
 */
#include "circbuf.h"
#include "stm32h7xx.h" // for __get_PRIMASK() and __set_PRIMASK()

// Enter critical section, return previous PRIMASK
static inline uint32_t enterCritical(void) {
	uint32_t primask = __get_PRIMASK();
	__set_PRIMASK(1); // disable interrupts
	return primask;
}

// Exit critical section, restore previous PRIMASK
static inline void exitCritical(uint32_t primask) {
	__set_PRIMASK(primask);
}

// Initialise circular buffer and fill with zeros
void initCircBuffer(CircularBuffer_t *buf) {
	buf->head = 0;
	buf->tail = 0;
	for (int i=0; i<BUF_LENGTH; i++)
		buf->data[i] = 0;
}

// Ignore any stored data by moving the tail up to the head
void emptyCircBuffer(CircularBuffer_t *buf) {
	uint32_t primask = enterCritical();
	buf->tail = buf->head;
	exitCritical(primask);
}

// Check if circular buffer is full
bool isCircBufFull(const CircularBuffer_t *buf) {
	uint32_t primask = enterCritical();
	bool full = ((buf->head - buf->tail) >= (uint32_t)(BUF_LENGTH - 1U));
	exitCritical(primask);

	return full;
}

// Check if circular buffer is empty
bool isCircBufEmpty(const CircularBuffer_t *buf) {
	uint32_t primask = enterCritical();
	bool empty = (buf->head == buf->tail);
	exitCritical(primask);

	return empty;
}

// Return the number of bytes currently in a circular buffer
uint32_t inCircBuf(const CircularBuffer_t *buf) {
	uint32_t primask = enterCritical();
	uint32_t count = buf->head - buf->tail;
	exitCritical(primask);

	return count;
}

// Write multiple characters to a circular buffer
// Stop writing if the buffer becomes full
// Returns the number of characters successfully written
uint32_t writeToCircBuf(CircularBuffer_t *buf, const uint8_t *str, uint32_t length) {
	if (length == 0) return 0;

	uint32_t primask = enterCritical();

	// Check available space in the buffer (one slot reserved to distinguish full/empty)
	uint32_t used = buf->head - buf->tail; // Unsigned handles overflows
	uint32_t free_space = (uint32_t)(BUF_LENGTH - 1U) - used;

	// Write up to 'length' bytes into the ring buffer
	uint32_t to_write = (length < free_space) ? length : free_space;

	for (uint32_t i = 0; i < to_write; i++) {
		buf->data[buf->head & BUF_MASK] = str[i];
		buf->head++;   // Monotonic increment
	}
	exitCritical(primask);

	return to_write;
}

// Check the value at a position in the buffer without modification
// Positive values read from the beginning of the buffer (0 = oldest data)
// Negative values read from the end of the buffer (-1 = newest data)
// Returns 0 if requested position is not valid data
uint8_t peekCircBuf(const CircularBuffer_t *buf, int position) {
	uint32_t primask = enterCritical();

	// Get the number of bytes in the buffer
	int32_t available = (int32_t)(buf->head - buf->tail);

	// Check if buffer is empty or out of range
	if ((available == 0) || (position >= available) || (position < -available)) {
		exitCritical(primask);
		return 0;
	}

	// Calculate positive index from tail
	uint32_t pos_u = (position >= 0) ? (uint32_t)position : (uint32_t)(available + position);

	// Compute physical index and read
	uint32_t idx = (buf->tail + pos_u) & BUF_MASK;
	uint8_t ret = buf->data[idx];

	exitCritical(primask);
	return ret;
}

// Read and remove a single character from a circular buffer
// Returns 0 if the buffer is empty or 1 on success
int readFromCircBuf(CircularBuffer_t *buf, uint8_t *ch) {
	uint32_t primask = enterCritical();
	int success = 0;

	if (buf->head == buf->tail) { // Buffer empty
		*ch = '\0';
	} else {
		*ch = buf->data[buf->tail & BUF_MASK];
		buf->tail = buf->tail + 1U;
		success = 1;
	}
	exitCritical(primask);

	return success;
}

/*  ----------------------------------------------------------------------
	Helper for DMA/IDLE: advance head based on DMA write index
	- dma_write_pos: current DMA write index (0 .. BUF_LENGTH-1),
		i.e. number of bytes written since stream started modulo BUF_LENGTH,
		computed as (BUF_LENGTH - NDTR).
	- Returns:
		0 -> OK, no overflow
		1 -> overflow detected (unread bytes would exceed BUF_LENGTH)
	Notes:
	- Function does not touch any usart_t flags; caller should set RxOverflow/RxNewData etc.
	- head is monotonic within the modulo arithmetic used by your buffer implementation.
	- oldest data will be overwritten if buffer gets full
	---------------------------------------------------------------------- */
int advanceCircBufByDMApos(CircularBuffer_t *buf, uint32_t dma_write_pos) {
	uint32_t primask = enterCritical();
	int overflow = 0;

	uint32_t prev_write_idx = buf->head & BUF_MASK;
	uint32_t new_pos = dma_write_pos & BUF_MASK;
	uint32_t delta = (new_pos >= prev_write_idx) ? (new_pos - prev_write_idx) : (BUF_LENGTH - prev_write_idx + new_pos);

	buf->head += delta; // monotonic increment, wraps naturally for uint32_t

	// detect overflow
	if ((buf->head - buf->tail) > (uint32_t)(BUF_LENGTH - 1U))
		overflow = 1;

	exitCritical(primask);
	return overflow;
}
