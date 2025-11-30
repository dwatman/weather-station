#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32h7xx.h" // for __get_PRIMASK() and __set_PRIMASK()

#include "network.h"
#include "circbuf.h"
#include "shared.h"

extern CircularBuffer_t rx1Buf;

extern SharedData_t *sharedMem;

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

// Remove delimiter characters and get the first message in the circular buffer
// Returns the number of characters put into the data buffer
uint32_t extractMessage(CircularBuffer_t *buf, char *out) {
	if (isCircBufEmpty(buf)) return 0;

	uint32_t primask = enterCritical();

	uint32_t available = buf->head - buf->tail;
	uint32_t startPos = 0;
	uint32_t msgLen = 0;
	bool endFound = false;

	// Skip leading CR/LF if present
	while (available > 0) {
		uint8_t c = buf->data[(buf->tail + startPos) & BUF_MASK];
		if (c != '\r' && c != '\n') break;
		startPos++;
		available--;
	}

	if (available == 0) {
		// Only delimiters in buffer
		buf->tail += startPos;
		exitCritical(primask);
		return 0;
	}

	// Search for end delimiter (\r\n)
	for (uint32_t i = startPos; i + 1 < available; i++) {
		uint8_t c1 = buf->data[(buf->tail + i) & BUF_MASK];
		uint8_t c2 = buf->data[(buf->tail + i + 1) & BUF_MASK];
		if (c1 == '\r' && c2 == '\n') {
			msgLen = i - startPos;
			endFound = true;
			break;
		}
	}

	if (!endFound) {
		// Discard incomplete data if no terminator found and buffer is almost full
		if (msgLen >= (NINA_BUF_LENGTH - 3))
			buf->tail = buf->head;
		exitCritical(primask);
		return 0;
	}

	// Copy message body
	uint32_t toCopy = (msgLen < NINA_BUF_LENGTH) ? msgLen : NINA_BUF_LENGTH;
	for (uint32_t i = 0; i < toCopy; i++) {
		out[i] = buf->data[(buf->tail + startPos + i) & BUF_MASK];
	}

	// Advance tail past trailing \r\n characters
	buf->tail = buf->tail + startPos + msgLen + 2;

	exitCritical(primask);
	return toCopy;
}

//
// Return 1 if a message was extracted, 0 if not
int getNinaMsg(NinaMessage_t *msg) {
	int err;
	//printf("buf: %3lu", inCircBuf(&rx1Buf));
	memset(msg, 0, sizeof(*msg));

	msg->length = extractMessage(&rx1Buf, msg->data);

	/*if (msg->length) {
		msg->data[msg->length] = 0;
		printf(" <%s>", msg->data);
	}
	printf("\n");*/

	if (msg->length) {
		err = parseNinaMsg(msg);
		/*printf("  %i (%lu) %s %u:", err, msg->length, msg->type, msg->field_count);
		for (int i=0; i<msg->field_count; i++)
			printf(" %s", msg->fields[i]);
		printf("\n");*/
	}

	// Return 1 if valid message was read, 0 otherwise
	if ((msg->length) && (err == 0))
		return 1;
	else
		return 0;
}

// Parses a NINA message in-place, modifying the input buffer.
// Returns 0 on success, -1 on invalid length, -2 on too many fields.
int parseNinaMsg(NinaMessage_t *msg) {
	if (msg->length == 0 || msg->length >= NINA_BUF_LENGTH)
		return -1;

	// Ensure null termination
	msg->data[msg->length] = '\0';

	// Find first colon separating type from data
	char *colon = strchr(msg->data, ':');
	if (colon) {
		*colon = '\0';
		msg->type = msg->data;

		// Sanity check type length
		if (strlen(msg->type) >= NINA_MAX_TYPE_LEN)
		return -1;

		char *data = colon + 1;
		char *tok = strtok(data, ",");
		while (tok) {
			if (msg->field_count >= NINA_MAX_FIELDS)
			return -2; // too many fields
			msg->fields[msg->field_count++] = tok;
			tok = strtok(NULL, ",");
		}
	}
	else {
		// No colon → whole string is the type
		msg->type = msg->data;
		if (strlen(msg->type) >= NINA_MAX_TYPE_LEN)
		return -1;
	}

	return 0; // success
}

void handle_OK(NinaMessage_t *msg) {
	//printf("NINA OK\n");
}

void handle_ERROR(NinaMessage_t *msg) {
	printf("NINA ERROR\n");
}

void handle_UWSSTAT(NinaMessage_t *msg) {
	//printf("NINA UWSSTAT\n");

	int id = atoi(msg->fields[0]);

	if ((msg->field_count == 2) && (id == 3)) {
		int value = atoi(msg->fields[1]);
		printf("Connection state %i\n", value);
	}
	else if ((msg->field_count == 2) && (id == 6)) {
		int value = atoi(msg->fields[1]);
		printf("RSSI %i\n", value);
		sharedMem->rssi = value;
	}
	else
		printf("Unhandled UWSSTAT id %i\n", id);
}



int processNinaMsg(NinaMessage_t *msg) {
	const char *type = msg->type;
	size_t len = strlen(type);
	int err = 0;

	// Most common first for speed
	if (len == 2 && type[0] == 'O' && type[1] == 'K') {
		handle_OK(msg);
	}
	else if (len == 5 && strncmp(type, "ERROR", 5) == 0) {
		handle_ERROR(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UWSSTAT", 8) == 0) {
		handle_UWSSTAT(msg);
	}/*
	else if (type[0] == '+' && strncmp(type, "+UDCP", 5) == 0) {
		handle_UDCP(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UUDATA", 7) == 0) {
		handle_UUDATA(msg);
	}*/
	else {
		printf("Unhandled message: <%s>\n", msg->data);
		err = -1;
	}

	return err;
}
