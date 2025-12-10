#ifndef APPLICATION_USER_INCLUDE_WIFI_SM_H_
#define APPLICATION_USER_INCLUDE_WIFI_SM_H_

#include <stdint.h>

// Timeout and retry defaults
#define WIFI_SM_DEFAULT_TIMEOUT_MS   5000
#define WIFI_SM_MAX_RETRIES          3

// Progress flags
#define FLAG_UUWLE  (1 << 0)
#define FLAG_UUNU   (1 << 1)
#define FLAG_UDCP   (1 << 2)
#define FLAG_UUDPC  (1 << 3)

// State machine states
typedef enum {
	SM_IDLE = 0,
	SM_RESET_POWER_OFF,
	SM_WAIT_STARTUP,
	SM_WAIT_NET_JOIN,
	SM_MQTT_CFG1_CONNECT,
	SM_MQTT_CFG1_SEND_JSON,
	SM_MQTT_CFG1_DISCONNECT,
	SM_MQTT_CFG2_CONNECT,
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
} WifiEvents_t;

// State machine context
typedef struct {
	uint8_t waiting;      // waiting for OK/ERROR after command
	uint8_t retries;      // retry counter
	uint8_t progress;     // progress flags (UUWLE, UUNU)
	uint8_t state;        // current state (WifiSM_State_t)
	uint32_t timeout_ms;  // timeout for current command
	int peer_handle;      // store peer handle from +UDCP
} WifiSM_Ctx_t;

// Global instances (shared with network.c)
extern WifiEvents_t wifi_events;
extern WifiSM_Ctx_t wifi_ctx;

// Public API
void wifi_state_machine_step(void);
void wifi_sm_init(void);

#endif
