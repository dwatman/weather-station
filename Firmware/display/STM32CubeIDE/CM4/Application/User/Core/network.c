#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32h7xx.h" // for __get_PRIMASK() and __set_PRIMASK()

#include "network.h"
#include "wifi_sm.h"
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

	// Skip leading CR/LF before message start
	while (startPos < available) {
		uint8_t c = buf->data[(buf->tail + startPos) & BUF_MASK];
		if (c != '\r' && c != '\n') break;
		startPos++;
	}

	if (startPos >= available) {
		// only CR/LF so far; don't advance tail yet
		exitCritical(primask);
		return 0;
	}

	// Handle +UDATR (binary)
	bool isUDATR = false;
	if (available - startPos >= 6) {
		char header[8] = {0};
		for (uint32_t i = 0; i < 6; i++)
			header[i] = buf->data[(buf->tail + startPos + i) & BUF_MASK];
		if (strncmp(header, "+UDATR", 6) == 0)
			isUDATR = true;
	}

	if (isUDATR) {
		// Find CRLF after "+UDATR:<len>"
		uint32_t i = startPos;
		while (i + 1 < available) {
			if (buf->data[(buf->tail + i) & BUF_MASK] == '\r' &&
				buf->data[(buf->tail + i + 1) & BUF_MASK] == '\n')
				break;
			i++;
		}
		if (i + 1 >= available) {
			exitCritical(primask);
			return 0; // header not complete yet
		}

		// Extract payload length
		char lenStr[12] = {0};
		uint32_t lenStart = startPos + 7;
		uint32_t lenCount = 0;
		for (uint32_t j = lenStart; j < i && lenCount < sizeof(lenStr) - 1; j++, lenCount++)
			lenStr[lenCount] = buf->data[(buf->tail + j) & BUF_MASK];
		uint32_t payloadLen = atoi(lenStr);

		if (payloadLen > NINA_PAYLOAD_SIZE) {
			printf("[NINA] Payload too large: %lu bytes\n", (unsigned long)payloadLen);
			buf->tail = buf->head; // discard all to recover
			exitCritical(primask);
			return 0;
		}

		// header + CRLF + payload + trailing CRLF
		uint32_t totalNeeded = (i - startPos + 2) + payloadLen + 2;
		if (available < totalNeeded + startPos) {
			exitCritical(primask);
			return 0; // incomplete
		}

		// Copy payload
		uint32_t payloadStart = buf->tail + i + 2;
		for (uint32_t j = 0; j < payloadLen; j++)
			msg->payload[j] = buf->data[(payloadStart + j) & BUF_MASK];
		msg->payload_length = payloadLen;
		msg->length = payloadLen;
		strcpy(msg->type, "+UDATR");
		msg->is_binary = true;

		buf->tail += totalNeeded + startPos;
		exitCritical(primask);
		return payloadLen;
	}

	// Handle ">" prompt
	uint8_t firstChar = buf->data[(buf->tail + startPos) & BUF_MASK];
	if (firstChar == '>') {
		msg->payload[0] = '\0';  // empty payload
		strcpy(msg->type, ">");
		msg->is_binary = false;
		msg->length = 1;
		msg->payload_length = 0;

		// Advance tail past the '>' only
		buf->tail += startPos + 1;

		exitCritical(primask);
		return 1;
	}

	// Normal text message
	uint32_t msgEnd = startPos;
	bool foundEnd = false;

	for (; msgEnd + 1 < available; msgEnd++) {
		uint8_t c1 = buf->data[(buf->tail + msgEnd) & BUF_MASK];
		uint8_t c2 = buf->data[(buf->tail + msgEnd + 1) & BUF_MASK];
		if (c1 == '\r' && c2 == '\n') {
			foundEnd = true;
			break;
		}
	}

	if (!foundEnd) {
		exitCritical(primask);
		return 0; // incomplete message
	}

	uint32_t msgLen = msgEnd - startPos;
	uint32_t toCopy = (msgLen < sizeof(msg->payload) - 1) ? msgLen : sizeof(msg->payload) - 1;

	for (uint32_t i = 0; i < toCopy; i++)
		msg->payload[i] = buf->data[(buf->tail + startPos + i) & BUF_MASK];
	msg->payload[toCopy] = '\0';
	msg->is_binary = false;
	msg->length = toCopy;
	msg->payload_length = 0;

	// Determine message type (up to ':')
	char *colon = strchr(msg->payload, ':');
	if (colon) {
		size_t tlen = (size_t)(colon - msg->payload);
		tlen = (tlen < sizeof(msg->type) - 1) ? tlen : sizeof(msg->type) - 1;
		memcpy(msg->type, msg->payload, tlen);
		msg->type[tlen] = '\0';
	} else {
		strncpy(msg->type, msg->payload, sizeof(msg->type) - 1);
		msg->type[sizeof(msg->type) - 1] = '\0';
	}

	// Advance tail after CRLF
	buf->tail += startPos + msgLen + 2;

	exitCritical(primask);
	return msg->length;
}

// Retrieve and parse a message from buffer.
// Returns 1 if valid message extracted, 0 if not.
int getNinaMsg(NinaMessage_t *msg) {
	memset(msg, 0, sizeof(*msg));

	uint32_t len = extractMessage(&rx1Buf, msg);
	if (!len) return 0;  // no complete message available

	if (!msg->is_binary) {
		// Tokenize fields safely without corrupting subsequent parses
		char *data = strchr(msg->payload, ':');
		if (data) {
			data++;
			char *saveptr = NULL;
			char *tok = strtok_r(data, ",", &saveptr);
			while (tok && msg->field_count < NINA_MAX_FIELDS) {
				msg->fields[msg->field_count++] = tok;
				tok = strtok_r(NULL, ",", &saveptr);
			}
		}
	}

	return 1;  // one complete message extracted
}

void handle_OK(NinaMessage_t *msg) {
	//printf("NINA OK\n");
	wifi_events.EV_OK = 1;
}

void handle_ERROR(NinaMessage_t *msg) {
	printf("NINA ERROR\n");
	wifi_events.EV_ERROR = 1;
}

void handle_GT(NinaMessage_t *msg) {
	//printf("NINA PROMPT\n");
	wifi_events.EV_PROMPT = 1;
}

void handle_STARTUP(NinaMessage_t *msg) {
	printf("NINA STARTUP\n");
	wifi_events.EV_STARTUP = 1;
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
		//printf("RSSI %i\n", value);
		sharedMem->rssi = value;
	}
	else
		printf("Unhandled UWSSTAT id %i\n", id);
}

// Wi-Fi connection established
void handle_UUWLE(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	char *ssid = msg->fields[1];
	int ch = atoi(msg->fields[2]);
	//printf("NINA UUWLE (%i)\n", id);
	printf("WiFi connected (%i) SSID: <%s> on ch %i\n", id, ssid, ch);
	wifi_events.EV_UUWLE = 1;
}

// Wi-Fi connection disconnected
void handle_UUWLD(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	int cause = atoi(msg->fields[1]);
	//printf("NINA UUWLD (%i)\n", id);
	printf("WiFi disconnected (%i) code: %i\n", id, cause);
	wifi_events.EV_DISCONNECT = 1;
}

// Network up
void handle_UUNU(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	//printf("NINA UUNU (%i)\n", id);
	printf("Network up (%i)\n", id);
	wifi_events.EV_UUNU = 1;
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
	//printf("NINA UDCP (%i)\n", id);
	wifi_events.EV_UDCP = 1;
	wifi_ctx.peer_handle = id;
}

// Peer connected
void handle_UUDPC(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	//printf("NINA UUDPC (%i)\n", value);
	printf("Connected to peer: %i\n", id);
	wifi_events.EV_UUDPC = 1;
}

// Peer disconnected
void handle_UUDPD(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	//printf("NINA UUDPD (%i)\n", value);
	printf("Disconnected from peer: %i\n", id);
	wifi_events.EV_PEER_CLOSED = 1;
}

// Data from remote peer is available
void handle_UUDATA(NinaMessage_t *msg) {
	int id = atoi(msg->fields[0]);
	int length = atoi(msg->fields[1]);
	//printf("NINA UUDATA (%i:%i)\n", id, length);
	//printf("Data available from peer %i: %i bytes\n", id, length);
	if (length > 0)
		printfCircBuf(&tx1Buf, "AT+UDATR=%i,2,%i\r\n", id, length); // Request all data
}

// Read received data from peer
void handle_UDATR(NinaMessage_t *msg) {
	//int length = atoi(msg->fields[0]);
	//printf("NINA UDATR (%i): %li\n", length, msg->payload_length);
	printf("Payload (%li): <%s>\n", msg->payload_length, msg->payload);

	// Process received data and put into shared memory
	parseRxMessage(msg);
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
	else if (len == 1 && type[0] == '>') {
		handle_GT(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+STARTUP", 8) == 0) { // Module start
		handle_STARTUP(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UWSSTAT", 8) == 0) { // Wi-Fi station status
		handle_UWSSTAT(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UUWLE", 6) == 0) { // Wi-Fi connection established
		handle_UUWLE(msg);
	}
	else if (type[0] == '+' && strncmp(type, "+UUWLD", 6) == 0) { // Wi-Fi connection disconnected
		handle_UUWLD(msg);
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
	else if (len >= 2 && type[0] == 'A' && type[1] == 'T') {
		printf("Sent: <%s>\n", msg->payload);
	}
	else {
		printf("Unhandled message (len %d): <%s>\n", len, msg->payload);
		err = -1;
	}

	return err;
}

int parseRxMessage(const NinaMessage_t* msg) {
    int pos = 0;
    const char *data = msg->payload;
    char buffer[10];

    // Example: "1435,+23.5,-05.2,45.3,62.1,025,030,045,050,055,060,1"

    if (msg->payload_length != MQTT_EXPECTED_MSG_LENGTH) {
    	printf("Invalid MQTT message length (%li), ignored\n", msg->payload_length);
    	return -1;
    }

    // Parse time (HHMM)
    strncpy(buffer, data + pos, 2); buffer[2] = '\0';
    uint8_t time_hr = (uint8_t)atoi(buffer);
    pos += 2;
    strncpy(buffer, data + pos, 2); buffer[2] = '\0';
    uint8_t time_min = (uint8_t)atoi(buffer);
	sharedMem->time = ((uint16_t)time_hr<<8) | time_min;
    pos += 3;  // +1 for comma

    // Parse indoor temp (+/-XX.X)
    strncpy(buffer, data + pos, 5); buffer[5] = '\0';
    sharedMem->t1 = atof(buffer);
    pos += 6;

    // Parse outdoor temp
    strncpy(buffer, data + pos, 5); buffer[5] = '\0';
    sharedMem->t2 = atof(buffer);
    pos += 6;

    // Parse indoor humidity (XX.X)
    strncpy(buffer, data + pos, 4); buffer[4] = '\0';
    sharedMem->h1 = atof(buffer);
    pos += 5;

    // Parse outdoor humidity
    strncpy(buffer, data + pos, 4); buffer[4] = '\0';
    sharedMem->h2 = atof(buffer);
    pos += 5;

    // Parse soil moisture (6 values)
    for (int i = 0; i < 6; i++) {
        strncpy(buffer, data + pos, 3); buffer[3] = '\0';
        sharedMem->soil[i] = atoi(buffer);
        pos += 4;
    }

    // Parse garage door state
    if (data[pos] == '0')
    	sharedMem->status |= STATUS_GARAGE;
    else
    	sharedMem->status &= ~STATUS_GARAGE;

    return 0;
}
