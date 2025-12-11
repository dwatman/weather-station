#ifndef APPLICATION_USER_INCLUDE_NETWORK_H_
#define APPLICATION_USER_INCLUDE_NETWORK_H_

#include <stdint.h>
#include "circbuf.h"

#define NINA_PAYLOAD_SIZE 256U

#define NINA_MAX_FIELDS     8
#define NINA_MAX_TYPE_LEN  16
#define NINA_MAX_FIELD_LEN 32

typedef struct {
	char type[NINA_MAX_TYPE_LEN]; 		// Message type, e.g. "+UDATR", "OK"
	uint32_t length; 					// Total message length
	uint32_t payload_length; 			// Payload length (if any)
	char payload[NINA_PAYLOAD_SIZE]; 	// Static payload buffer
	char *fields[NINA_MAX_FIELDS]; 		// For text command fields
	uint8_t field_count;
	bool is_binary; 					// True if message has binary payload
} NinaMessage_t;

typedef enum nina_response_e {
	RESPONSE_NONE,
	RESPONSE_OK,
	RESPONSE_ERROR,
	RESPONSE_DATA
} NinaResponseType;


uint32_t extractMessage(CircularBuffer_t *buf, NinaMessage_t *msg);
int getNinaMsg(NinaMessage_t *msg);
int parseNinaMsg(NinaMessage_t *msg);
int processNinaMsg(NinaMessage_t *msg);
int parseRxMessage(const char* msg);

#endif
