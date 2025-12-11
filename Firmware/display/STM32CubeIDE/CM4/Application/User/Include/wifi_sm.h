#ifndef APPLICATION_USER_INCLUDE_WIFI_SM_H_
#define APPLICATION_USER_INCLUDE_WIFI_SM_H_

#include <stdint.h>

// Timeout and retry defaults
#define WIFI_SM_DEFAULT_TIMEOUT_MS   10000
#define WIFI_SM_MAX_RETRIES          3

// Progress flags
#define FLAG_OK     (1 << 0)
#define FLAG_UUWLE  (1 << 1)
#define FLAG_UUNU   (1 << 2)
#define FLAG_UDCP   (1 << 3)
#define FLAG_UUDPC  (1 << 4)
#define FLAG_UUDPD  (1 << 5)

// Buffers for MQTT setup
#define TOPIC_BUF_LEN   128
#define PAYLOAD_BUF_LEN 512

// MQTT topics
#define TOPIC_DISC_PRES "homeassistant/sensor/DW_display/pressure/config"
#define TOPIC_DISC_LITE "homeassistant/sensor/DW_display/light/config"
#define TOPIC_DATA      "test_data"
#define TOPIC_LISTEN    "display"

// Sequence states
#define SEQ_CONFIG_PRESSURE 0
#define SEQ_CONFIG_LIGHT    1
#define SEQ_DATA_CONNECT    2

// Payload types for MQTT discovery messages
#define PAYLOAD_JSON_NONE 0
#define PAYLOAD_JSON_PRES 1
#define PAYLOAD_JSON_LITE 2

// State machine states
typedef enum {
	SM_IDLE = 0,
	SM_RESET_POWER_OFF,
	SM_WAIT_STARTUP,
	SM_WAIT_NET_JOIN,
	SM_MQTT_SEQ_CONNECT,
	SM_MQTT_SEQ_WRITE_CMD,
	SM_MQTT_SEQ_WRITE_DATA,
	SM_MQTT_SEQ_DISCONNECT,
	SM_OPERATIONAL,
	SM_ERROR_RECOVERY
} WifiSM_State_t;

// Event flags (set by parser handle_XXX functions)
typedef struct {
	volatile uint8_t EV_STARTUP;
	volatile uint8_t EV_UUWLE;
	volatile uint8_t EV_UUNU;
	volatile uint8_t EV_UDCP;
	volatile uint8_t EV_UUDPC;
	volatile uint8_t EV_OK;
	volatile uint8_t EV_ERROR;
	volatile uint8_t EV_DISCONNECT;
	volatile uint8_t EV_PEER_CLOSED;
	volatile uint8_t EV_PROMPT;
} WifiEvents_t;

// State machine context
typedef struct {
	uint8_t waiting;       // waiting for OK/ERROR after command
	uint8_t retries;       // retry counter
	uint8_t progress;      // progress flags (UUWLE, UUNU)
	uint8_t state;         // current state (WifiSM_State_t)
	uint8_t sequence_step; // step in sub-sequence
	uint32_t timeout_ms;   // timeout for current command
	int peer_handle;       // store peer handle from +UDCP
	uint8_t payload_type;   // Payload type currently in the buffer
	int payload_len;       // Length of data in the payload
	uint8_t payload[PAYLOAD_BUF_LEN];  // Payload data
} WifiSM_Ctx_t;

// Global instances (shared with network.c)
extern WifiEvents_t wifi_events;
extern WifiSM_Ctx_t wifi_ctx;

// Public API
void wifi_state_machine_step(void);
void wifi_sm_init(void);

#endif
