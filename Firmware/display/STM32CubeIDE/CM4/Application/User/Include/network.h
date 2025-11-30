#ifndef APPLICATION_USER_INCLUDE_NETWORK_H_
#define APPLICATION_USER_INCLUDE_NETWORK_H_

#include <stdint.h>
#include "circbuf.h"

#define NINA_BUF_LENGTH 256U

#define NINA_MAX_FIELDS     4
#define NINA_MAX_TYPE_LEN  16
#define NINA_MAX_FIELD_LEN 32

typedef struct {
	uint32_t length;
	char data[NINA_BUF_LENGTH];
	char *type;
	char *fields[NINA_MAX_FIELDS];
	int field_count;
} NinaMessage_t;

typedef enum nina_response_e {
  RESPONSE_NONE,
  RESPONSE_OK,
  RESPONSE_ERROR,
  RESPONSE_DATA
} NinaResponseType;


uint32_t extractMessage(CircularBuffer_t *buf, char *out);
int getNinaMsg(NinaMessage_t *msg);
int parseNinaMsg(NinaMessage_t *msg);
int processNinaMsg(NinaMessage_t *msg);

#endif
