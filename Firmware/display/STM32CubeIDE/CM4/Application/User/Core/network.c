#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32h7xx.h" // for __get_PRIMASK() and __set_PRIMASK()

#include "network.h"
#include "usart_util.h"
#include "circbuf.h"
#include "shared.h"

extern CircularBuffer_t tx1Buf;
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


// Extract a single complete message from the circular buffer.
// Returns the number of bytes extracted (header + payload) or 0 if incomplete.
uint32_t extractMessage(CircularBuffer_t *buf, NinaMessage_t *msg) {
    if (isCircBufEmpty(buf)) return 0;

    uint32_t primask = enterCritical();
    uint32_t available = buf->head - buf->tail;
    uint32_t startPos = 0;

    // Skip any leading CR/LF
    while (available > 0) {
        uint8_t c = buf->data[(buf->tail + startPos) & BUF_MASK];
        if (c != '\r' && c != '\n') break;
        startPos++;
        available--;
    }

    if (available == 0) {
        buf->tail += startPos;
        exitCritical(primask);
        return 0;
    }

    // Detect +UDATR
    bool isUDATR = false;
    if (available >= 6) {
        char header[8] = {0};
        for (uint32_t i = 0; i < 7 && i < available; i++)
            header[i] = buf->data[(buf->tail + startPos + i) & BUF_MASK];
        if (strncmp(header, "+UDATR", 6) == 0)
            isUDATR = true;
    }

    // ---- Handle +UDATR message ----
    if (isUDATR) {
        // Find CRLF after "+UDATR:<len>"
        uint32_t lineEnd = startPos;
        while (lineEnd + 1 < available) {
            if (buf->data[(buf->tail + lineEnd) & BUF_MASK] == '\r' &&
                buf->data[(buf->tail + lineEnd + 1) & BUF_MASK] == '\n') {
                break;
            }
            lineEnd++;
        }

        if (lineEnd + 1 >= available) {
            exitCritical(primask);
            return 0; // incomplete header line
        }

        // Extract numeric length after "+UDATR:"
        char lenStr[12] = {0};
        uint32_t lenStart = startPos + 7; // after "+UDATR:"
        uint32_t lenCount = 0;
        for (uint32_t i = lenStart; i < lineEnd && lenCount < sizeof(lenStr)-1; i++, lenCount++)
            lenStr[lenCount] = buf->data[(buf->tail + i) & BUF_MASK];
        uint32_t payloadLen = atoi(lenStr);

        if (payloadLen > NINA_PAYLOAD_SIZE) {
            printf("[NINA] Payload too large: %lu bytes (max %d)\n", payloadLen, NINA_PAYLOAD_SIZE);
            buf->tail = buf->head; // discard entire buffer safely
            exitCritical(primask);
            return 0;
        }

        // Calculate total bytes required for complete message
        // header line + CRLF + payload + final CRLF
        uint32_t totalNeeded = (lineEnd - startPos + 2) + payloadLen + 2;
        if (available < totalNeeded) {
            exitCritical(primask);
            return 0; // not yet complete
        }

        // Copy payload
        uint32_t payloadStart = buf->tail + lineEnd + 2; // after first CRLF
        for (uint32_t i = 0; i < payloadLen; i++)
            msg->payload[i] = buf->data[(payloadStart + i) & BUF_MASK];

        msg->payload_length = payloadLen;
        msg->length = payloadLen;
        strcpy(msg->type, "+UDATR");
        msg->is_binary = true;

        // Advance tail past message
        buf->tail += totalNeeded + startPos;

        exitCritical(primask);
        return payloadLen;
    }

    // ---- Handle normal text-based message ----
    uint32_t msgLen = 0;
    bool endFound = false;

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
        exitCritical(primask);
        return 0; // incomplete text message
    }

    uint32_t toCopy = (msgLen < sizeof(msg->payload)) ? msgLen : sizeof(msg->payload) - 1;
    for (uint32_t i = 0; i < toCopy; i++)
        msg->payload[i] = buf->data[(buf->tail + startPos + i) & BUF_MASK];

    msg->payload[toCopy] = '\0';
    msg->length = toCopy;
    msg->payload_length = 0;
    msg->is_binary = false;

    // Determine message type (everything before ':')
    char *colon = strchr(msg->payload, ':');
    if (colon) {
        size_t tlen = (colon - msg->payload);
        tlen = (tlen < sizeof(msg->type)-1) ? tlen : sizeof(msg->type)-1;
        memcpy(msg->type, msg->payload, tlen);
        msg->type[tlen] = 0;
    } else {
        strncpy(msg->type, msg->payload, sizeof(msg->type)-1);
    }

    buf->tail += startPos + msgLen + 2;

    exitCritical(primask);
    return msg->length;
}

// Retrieve and parse a message from buffer.
// Returns 1 if valid message extracted, 0 if not.
int getNinaMsg(NinaMessage_t *msg) {
    memset(msg, 0, sizeof(*msg));

    uint32_t len = extractMessage(&rx1Buf, msg);
    if (!len) return 0;

    if (!msg->is_binary) {
        // Tokenize fields for text commands
        char *data = strchr(msg->payload, ':');
        if (data) {
            data++;
            char *tok = strtok(data, ",");
            while (tok && msg->field_count < NINA_MAX_FIELDS) {
                msg->fields[msg->field_count++] = tok;
                tok = strtok(NULL, ",");
            }
        }
    }

    return 1;
}


void handle_OK(NinaMessage_t *msg) {
	//printf("NINA OK\n");
}

void handle_ERROR(NinaMessage_t *msg) {
	printf("NINA ERROR\n");
}

// Network status
void handle_UWSSTAT(NinaMessage_t *msg) {
	//printf("NINA UWSSTAT\n");

	int id = atoi(msg->fields[0]);

	if ((msg->field_count == 2) && (id == 3)) {
		int value = atoi(msg->fields[1]);
		printf("Connection state %i\n", value);
		if (value == 2)
			sharedMem->status |= 1; // Connected to WiFi
		else
			sharedMem->status &= ~1; // Not connected
	}
	else if ((msg->field_count == 2) && (id == 6)) {
		int value = atoi(msg->fields[1]);
		printf("RSSI %i\n", value);
		sharedMem->rssi = value;
	}
	else
		printf("Unhandled UWSSTAT id %i\n", id);
}

// Network up
void handle_UUNU(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	//printf("NINA UUNU (%i)\n", id);
	printf("Network up (%i)\n", id);
}

// Network down
void handle_UUND(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	//printf("NINA UUND (%i)\n", id);
	printf("Network down (%i)\n", id);
}

// Connect peer response
void handle_UDCP(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	printf("NINA UDCP (%i)\n", id);
}

// Peer connected
void handle_UUDPC(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	//printf("NINA UUDPC (%i)\n", value);
	printf("Connected to peer: %i\n", id);
}

// Peer disconnected
void handle_UUDPD(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	//printf("NINA UUDPD (%i)\n", value);
	printf("Disconnected from peer: %i\n", id);
}

// Data from remote peer is available
void handle_UUDATA(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	int length = atoi(msg->fields[1]);
	//printf("NINA UUDATA (%i:%i)\n", id, length);
	printf("Data available from peer %i: %i bytes\n", id, length);
	if (length > 0)
		printfCircBuf(&tx1Buf, "AT+UDATR=%i,2,%i\r\n", id, length); // Request all data
}

// Read received data from peer
void handle_UDATR(NinaMessage_t *msg) {
	int length = atoi(msg->fields[0]);
	printf("NINA UDATR (%i): %li\n", length, msg->payload_length);
	printf("*** Payload: <%s>\n", msg->payload);

	// Get time
	uint8_t time_hr = (uint8_t)atoi((char[]){msg->payload[0], msg->payload[1], '\0'});
	uint8_t time_min = (uint8_t)atoi((char[]){msg->payload[2], msg->payload[3], '\0'});
	sharedMem->time = ((uint16_t)time_hr<<8) | time_min;
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
	else if (type[0] == '+' && strncmp(type, "+UWSSTAT", 8) == 0) { // Wi-Fi station status
		handle_UWSSTAT(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UUNU", 5) == 0) { // Network up
		handle_UUNU(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UUND", 5) == 0) { // Network down
		handle_UUND(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UDCP", 5) == 0) { // Connect peer response
		handle_UDCP(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UUDPC", 6) == 0) { // Peer connected
		handle_UUDPC(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UUDPD", 6) == 0) { // Peer disconnected
		handle_UUDPD(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UUDATA", 7) == 0) { // Data from remote peer is available
		handle_UUDATA(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UDATR", 6) == 0) { // Read received data from peer
		handle_UDATR(msg);
	}
	else {
		printf("Unhandled message: <%s>\n", msg->payload);
		err = -1;
	}

	return err;
}
