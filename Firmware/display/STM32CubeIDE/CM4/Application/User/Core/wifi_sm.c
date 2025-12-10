#include <stdio.h>
#include "wifi_sm.h"
#include "network.h"  // for tx1Buf, printfCircBuf(), emptyRx1Buffer()
#include "usart_util.h"

extern CircularBuffer_t tx1Buf;
extern volatile uint32_t wifi_timeout;

WifiEvents_t wifi_events = {0};
WifiSM_Ctx_t wifi_ctx = {0};

// Initialize state machine
void wifi_sm_init(void) {
	wifi_ctx.state = SM_IDLE;
	wifi_ctx.waiting = 0;
	wifi_ctx.retries = 0;
	wifi_ctx.progress = 0;
	wifi_ctx.timeout_ms = WIFI_SM_DEFAULT_TIMEOUT_MS;
}

// Main step function to call in main loop
void wifi_state_machine_step(void) {
	uint8_t event_ok = 0;
	uint8_t event_error = 0;

	// Consume events
	if (wifi_events.EV_OK) { event_ok = 1; wifi_events.EV_OK = 0; }
	if (wifi_events.EV_ERROR) { event_error = 1; wifi_events.EV_ERROR = 0; }

	// Check for unexpected module restart
	if (wifi_events.EV_STARTUP && wifi_ctx.state != SM_WAIT_STARTUP && wifi_ctx.state != SM_RESET_POWER_OFF) {
		wifi_events.EV_STARTUP = 0;
		printf("SM: Unexpected +STARTUP detected, restarting sequence\n");
		wifi_ctx.state = SM_WAIT_STARTUP;
		wifi_ctx.waiting = 0;
		wifi_ctx.retries = 0;
		wifi_ctx.progress = 0;
		return;  // restart processing next loop iteration
	}

	switch (wifi_ctx.state) {
	case SM_IDLE:
		// Send separator and wait for any response before proceeding
		if (!wifi_ctx.waiting) {
			//emptyRx1Buffer();
			//printfCircBuf(&tx1Buf, "\r\n");
			wifi_ctx.waiting = 1;
			wifi_timeout = 1000;  // short wait (~1s)
			wifi_ctx.retries = 0;
			printf("SM: Idle -> sent initial CRLF, waiting for response\n");
		} else if (event_ok || event_error) {
			// Either OK or ERROR means UART link is clear
			wifi_ctx.waiting = 0;
			wifi_ctx.state = SM_RESET_POWER_OFF;
			wifi_ctx.progress = 0;
			wifi_ctx.retries = 0;
			wifi_ctx.timeout_ms = WIFI_SM_DEFAULT_TIMEOUT_MS;
			printf("SM: Initial CRLF acknowledged, proceeding to reset\n");
		} else if (wifi_timeout == 0) {
			// Timeout waiting for a response; continue anyway
			wifi_ctx.waiting = 0;
			wifi_ctx.state = SM_RESET_POWER_OFF;
			printf("SM: Initial CRLF timeout, continuing to reset\n");
		}
		break;

	case SM_RESET_POWER_OFF:
		if (!wifi_ctx.waiting) {
			printfCircBuf(&tx1Buf, "AT+CPWROFF\r\n");
			wifi_ctx.waiting = 1;
			wifi_timeout = wifi_ctx.timeout_ms;
			wifi_ctx.retries = 0;
			wifi_ctx.progress = 0;
			printf("SM: Sent CPWROFF\n");
		} else if (event_ok || event_error) {
			// Accept either OK or ERROR as valid, since CPWROFF may return ERROR before reboot
			wifi_ctx.waiting = 0;
			wifi_ctx.state = SM_WAIT_STARTUP;
			printf("SM: CPWROFF response (%s), waiting for +STARTUP\n",
				   event_ok ? "OK" : "ERROR");
		} else if (wifi_timeout == 0) {
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) {
				wifi_ctx.waiting = 0;
				printf("SM: CPWROFF timeout, retry %d\n", wifi_ctx.retries);
			} else {
				wifi_ctx.state = SM_ERROR_RECOVERY;
				wifi_ctx.waiting = 0;
			}
		}
		break;


	case SM_WAIT_STARTUP:
		if (wifi_events.EV_STARTUP) {
			wifi_events.EV_STARTUP = 0;
			wifi_ctx.state = SM_WAIT_NET_JOIN;
			wifi_ctx.progress = 0;
			printf("SM: +STARTUP received, waiting for network join\n");
			printfCircBuf(&tx1Buf, "ATE1\r\n"); // Set echo on
		}
		break;

	case SM_WAIT_NET_JOIN:
		if (wifi_events.EV_UUWLE) { wifi_ctx.progress |= FLAG_UUWLE; wifi_events.EV_UUWLE = 0; }
		if (wifi_events.EV_UUNU) { wifi_ctx.progress |= FLAG_UUNU; wifi_events.EV_UUNU = 0; }
		if ((wifi_ctx.progress & (FLAG_UUWLE | FLAG_UUNU)) == (FLAG_UUWLE | FLAG_UUNU)) {
			wifi_ctx.state = SM_MQTT_CFG1_CONNECT;
			wifi_ctx.waiting = 0;
			wifi_ctx.retries = 0;
			printf("SM: Network joined, proceeding to MQTT CFG1 connect\n");
		}
		break;

	case SM_MQTT_CFG1_CONNECT:
		if (!wifi_ctx.waiting) {
			// Connect to MQTT to send descriptor
			printfCircBuf(&tx1Buf, "AT+UDCP=at-mqtt://192.168.0.200:1883/?client=NINA-W132&user=mqtt_user&passwd=MQ.jaygram&pt=test2&st=test&encr=0&qos=0\r\n");
			wifi_ctx.waiting = 1;
			wifi_timeout = wifi_ctx.timeout_ms;
			printf("SM: MQTT CFG1 connect (descriptor)\n");
		} else if (wifi_events.EV_UDCP) {
			wifi_events.EV_UDCP = 0;
			wifi_ctx.waiting = 0;
			wifi_ctx.state = SM_MQTT_CFG1_SEND_JSON;
			printf("SM: MQTT CFG1 connected, sending JSON\n");
		} else if (event_error || wifi_timeout == 0) {
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) wifi_ctx.waiting = 0;
			else wifi_ctx.state = SM_ERROR_RECOVERY;
		}
		break;

	case SM_MQTT_CFG1_SEND_JSON:
		wifi_ctx.state = 99;
		wifi_ctx.waiting = 0;
		/*
		if (!wifi_ctx.waiting) {
			// Only send once per entry
			wifi_ctx.waiting = 1;
			wifi_timeout = wifi_ctx.timeout_ms;
			// Send JSON configuration
			printfCircBuf(&tx1Buf, "AT+UDATW=1,0,testcfg\r\n");
			printf("SM: Sent JSON (placeholder)\n");
		} else if (event_ok) {
			wifi_ctx.waiting = 0;
			wifi_ctx.state = SM_MQTT_CFG1_DISCONNECT;
			wifi_ctx.retries = 0;
			printf("SM: JSON acknowledged, disconnecting MQTT CFG1\n");
		} else if (event_error) {
			wifi_ctx.waiting = 0;
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES)
				printf("SM: JSON send error, retry %d\n", wifi_ctx.retries);
			else
				wifi_ctx.state = SM_ERROR_RECOVERY;
		} else if (wifi_timeout == 0) {
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) {
				wifi_ctx.waiting = 0;
				printf("SM: JSON send timeout, retry %d\n", wifi_ctx.retries);
			} else {
				wifi_ctx.state = SM_ERROR_RECOVERY;
				wifi_ctx.waiting = 0;
			}
		}*/
		break;

	case SM_MQTT_CFG1_DISCONNECT:
		wifi_ctx.state = 99;
		wifi_ctx.waiting = 0;
		/*
		if (!wifi_ctx.waiting) {
			// Close MQTT connection
			printfCircBuf(&tx1Buf, "AT+UDCPC=1\r\n");
			wifi_ctx.waiting = 1;
			wifi_timeout = wifi_ctx.timeout_ms;
			printf("SM: MQTT CFG1 disconnect (descriptor)\n");
		} else if (event_ok) {
			wifi_ctx.waiting = 0;
			wifi_ctx.state = SM_MQTT_CFG2_CONNECT;
			printf("SM: MQTT CFG1 disconnected, connecting CFG2\n");
		} else if (event_error || wifi_timeout == 0) {
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) wifi_ctx.waiting = 0;
			else wifi_ctx.state = SM_ERROR_RECOVERY;
		}*/
		break;

	case SM_MQTT_CFG2_CONNECT:
		wifi_ctx.state = 99;
		wifi_ctx.waiting = 0;/*
		if (!wifi_ctx.waiting) {
			// Connect to MQTT (main)
			printfCircBuf(&tx1Buf, "AT+UDCP=at-mqtt://192.168.0.200:1883/?client=NINA-W132&user=mqtt_user&passwd=MQ.jaygram&pt=test2&st=display&encr=0&qos=0\r\n");
			wifi_ctx.waiting = 1;
			wifi_timeout = wifi_ctx.timeout_ms;
			printf("SM: MQTT CFG2 connect (main)\n");
		} else if (wifi_events.EV_UDCP) {
			wifi_events.EV_UDCP = 0;
			wifi_ctx.waiting = 0;
			wifi_ctx.state = SM_OPERATIONAL;
			printf("SM: MQTT CFG2 connected, entering operational\n");
		} else if (event_error || wifi_timeout == 0) {
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) wifi_ctx.waiting = 0;
			else wifi_ctx.state = SM_ERROR_RECOVERY;
		}*/
		break;

	case SM_OPERATIONAL:
		// Monitor disconnects and other runtime events
		if (wifi_events.EV_DISCONNECT) {
			wifi_events.EV_DISCONNECT = 0;
			printf("SM: Disconnected, reconnecting CFG2\n");
			wifi_ctx.state = SM_MQTT_CFG2_CONNECT;
			wifi_ctx.waiting = 0;
			wifi_ctx.retries = 0;
		}
		break;

	case SM_ERROR_RECOVERY:
		printf("SM: Error recovery, restarting...\n");
		wifi_ctx.state = SM_RESET_POWER_OFF;
		wifi_ctx.waiting = 0;
		wifi_ctx.retries = 0;
		wifi_ctx.progress = 0;
		emptyRx1Buffer();
		printfCircBuf(&tx1Buf, "\r\n");
		break;
	case 99://STOP for test
		if (!wifi_ctx.waiting) {
			printf("SM: Debug STOP\n");
			wifi_ctx.waiting = 1;
		}
		break;

	default:
		wifi_ctx.state = SM_ERROR_RECOVERY;
		break;
	}
}
