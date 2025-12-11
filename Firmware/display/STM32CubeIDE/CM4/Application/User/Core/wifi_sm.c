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
			//printfCircBuf(&tx1Buf, "ATE1\r\n"); // Set echo on for debug
		}
		break;

	case SM_WAIT_NET_JOIN:
		if (wifi_events.EV_UUWLE) { wifi_ctx.progress |= FLAG_UUWLE; wifi_events.EV_UUWLE = 0; }
		if (wifi_events.EV_UUNU) { wifi_ctx.progress |= FLAG_UUNU; wifi_events.EV_UUNU = 0; }
		if ((wifi_ctx.progress & (FLAG_UUWLE | FLAG_UUNU)) == (FLAG_UUWLE | FLAG_UUNU)) {
			wifi_ctx.state = SM_MQTT_CFG1_CONNECT;
			wifi_ctx.waiting = 0;
			wifi_ctx.retries = 0;
			wifi_ctx.progress = 0;
			printf("SM: Network joined, proceeding to MQTT CFG1 connect\n");
		}
		break;

	case SM_MQTT_CFG1_CONNECT:
		if (!wifi_ctx.waiting) {
			// Connect to MQTT to send descriptor
			printfCircBuf(&tx1Buf,
				"AT+UDCP=at-mqtt://192.168.0.200:1883/"
				"?client=NINA-W132_PUB&user=mqtt_user&passwd=MQ.jaygram"
				"&pt=testcfg&st=test&encr=0&qos=0\r\n");
			wifi_ctx.waiting   = 1;
			wifi_ctx.progress = 0;
			wifi_ctx.peer_handle = -1;
			wifi_ctx.retries   = 0;
			wifi_timeout       = wifi_ctx.timeout_ms;
			printf("SM: MQTT CFG1 connect (sending command)\n");
		}

		// Handle responses
		if (wifi_events.EV_UDCP) {
			wifi_events.EV_UDCP = 0;
			wifi_ctx.progress  |= FLAG_UDCP;
			printf("SM: +UDCP received, peer=%d\n", wifi_ctx.peer_handle);
		}
		if (wifi_events.EV_UUDPC) {
			wifi_events.EV_UUDPC = 0;
			wifi_ctx.progress  |= FLAG_UUDPC;
			printf("SM: +UUDPC received\n");
		}

		// Success: both replies arrived
		if ((wifi_ctx.progress & (FLAG_UDCP | FLAG_UUDPC)) ==
			(FLAG_UDCP | FLAG_UUDPC)) {
			wifi_ctx.waiting = 0;
			wifi_ctx.retries = 0;
			wifi_ctx.progress = 0;
			wifi_ctx.state   = SM_MQTT_CFG1_SEND_JSON;
			printf("SM: MQTT CFG1 fully connected, ready to send JSON\n");
		}

		// Timeout/retry
		if (wifi_timeout == 0 && wifi_ctx.waiting) {
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) {
				wifi_ctx.waiting = 0;
				wifi_ctx.progress = 0;
				printf("SM: MQTT CFG1 connect timeout, retry %d\n",
					   wifi_ctx.retries);
			} else {
				wifi_ctx.state   = SM_ERROR_RECOVERY;
				wifi_ctx.waiting = 0;
				printf("SM: MQTT CFG1 connect failed, recovery\n");
			}
		}
		break;

	case SM_MQTT_CFG1_SEND_JSON:
		if (!wifi_ctx.waiting) {
			if (wifi_ctx.peer_handle < 0) {
				printf("SM: No valid peer handle, recovery\n");
				wifi_ctx.state = SM_ERROR_RECOVERY;
				break;
			}
			// Send JSON configuration
			printfCircBuf(&tx1Buf, "AT+UDATW=%d,0,testcfg\r\n", wifi_ctx.peer_handle);
			wifi_ctx.waiting   = 1;
			wifi_ctx.retries   = 0;
			wifi_timeout       = wifi_ctx.timeout_ms;
			printf("SM: Sent configuration JSON (peer %d)\n", wifi_ctx.peer_handle);
		}

		if (event_ok) {
			wifi_ctx.waiting = 0;
			wifi_ctx.state   = SM_MQTT_CFG1_DISCONNECT;
			printf("SM: JSON send acknowledged, disconnecting CFG1\n");
		} else if (event_error) {
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) {
				wifi_ctx.waiting = 0;
				printf("SM: JSON send error, retry %d\n", wifi_ctx.retries);
			} else {
				wifi_ctx.state = SM_ERROR_RECOVERY;
				wifi_ctx.waiting = 0;
				printf("SM: JSON send failed, recovery\n");
			}
		} else if (wifi_timeout == 0 && wifi_ctx.waiting) {
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) {
				wifi_ctx.waiting = 0;
				printf("SM: JSON send timeout, retry %d\n", wifi_ctx.retries);
			} else {
				wifi_ctx.state = SM_ERROR_RECOVERY;
				wifi_ctx.waiting = 0;
				printf("SM: JSON send timed out, recovery\n");
			}
		}
		break;

	case SM_MQTT_CFG1_DISCONNECT:
		if (!wifi_ctx.waiting) {
			if (wifi_ctx.peer_handle < 0) {
				printf("SM: Invalid peer handle, skipping disconnect\n");
				wifi_ctx.state = SM_MQTT_CFG2_CONNECT;
				break;
			}
			// Close MQTT connection
			printfCircBuf(&tx1Buf, "AT+UDCPC=%d\r\n", wifi_ctx.peer_handle);
			wifi_ctx.waiting   = 1;
			wifi_ctx.retries   = 0;
			wifi_ctx.progress  = 0;;
			wifi_timeout       = wifi_ctx.timeout_ms;

			printf("SM: MQTT CFG1 disconnect (peer %d)\n", wifi_ctx.peer_handle);
			break; // wait for peer-closed
		}

		// Handle OK/ERROR/PEER_CLOSED responses
		if (event_ok) {
			event_ok = 0;
			wifi_ctx.progress |= FLAG_OK;
			printf("SM: Disconnect OK received\n");
		}

		if (event_error) {
			event_error = 0;
			wifi_ctx.waiting = 0;
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) {
				wifi_ctx.progress = 0;
				printf("SM: Disconnect ERROR, retry %d\n", wifi_ctx.retries);
			} else {
				wifi_ctx.state = SM_ERROR_RECOVERY;
				printf("SM: Disconnect failed, recovery\n");
			}
			break;
		}

		// Wait for EV_PEER_CLOSED
		if (wifi_events.EV_PEER_CLOSED) {
			wifi_events.EV_PEER_CLOSED = 0;
			wifi_ctx.progress |= FLAG_UUDPD;
			printf("SM: Peer closed event received\n");
		}

		// Success: both OK and PEER_CLOSED received
		if ((wifi_ctx.progress & (FLAG_OK | FLAG_UUDPD)) == (FLAG_OK | FLAG_UUDPD)) {
			wifi_ctx.waiting = 0;
			wifi_ctx.peer_handle = -1;
			wifi_ctx.retries = 0;
			wifi_ctx.state = SM_MQTT_CFG2_CONNECT;
			printf("SM: MQTT CFG1 disconnect complete, switching to CFG2\n");
			break;
		}

		// Timeout handling: still waiting for peer closed
		if (wifi_ctx.waiting && wifi_timeout == 0) {
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) {
				wifi_ctx.waiting = 0;
				wifi_ctx.progress &= ~(FLAG_OK | FLAG_UUDPD);
				printf("SM: Disconnect timeout, retry %d\n", wifi_ctx.retries);
			} else {
				wifi_ctx.state = SM_ERROR_RECOVERY;
				wifi_ctx.waiting = 0;
				wifi_ctx.state = SM_ERROR_RECOVERY;
				printf("SM: Disconnect timed out, recovery\n");
			}
		}
		break;

	case SM_MQTT_CFG2_CONNECT:
		if (!wifi_ctx.waiting) {
			// Connect to MQTT for publishing data
			printfCircBuf(&tx1Buf,
				"AT+UDCP=at-mqtt://192.168.0.200:1883/"
				"?client=NINA-W132_PUB&user=mqtt_user&passwd=MQ.jaygram"
				"&pt=testdata&st=display&encr=0&qos=0\r\n");
			wifi_ctx.waiting   = 1;
			wifi_ctx.progress = 0;
			wifi_ctx.peer_handle = -1;
			wifi_ctx.retries   = 0;
			wifi_timeout       = wifi_ctx.timeout_ms;
			printf("SM: MQTT CFG2 connect (sending command)\n");
		}

		// Handle responses
		if (wifi_events.EV_UDCP) {
			wifi_events.EV_UDCP = 0;
			wifi_ctx.progress  |= FLAG_UDCP;
			printf("SM: +UDCP received, peer=%d\n", wifi_ctx.peer_handle);
		}
		if (wifi_events.EV_UUDPC) {
			wifi_events.EV_UUDPC = 0;
			wifi_ctx.progress  |= FLAG_UUDPC;
			printf("SM: +UUDPC received\n");
		}

		// Success: both replies arrived
		if ((wifi_ctx.progress & (FLAG_UDCP | FLAG_UUDPC)) ==
			(FLAG_UDCP | FLAG_UUDPC)) {
			wifi_ctx.waiting = 0;
			wifi_ctx.retries = 0;
			wifi_ctx.progress = 0;
			wifi_ctx.state   = SM_OPERATIONAL;
			printf("SM: MQTT CFG2 fully connected, entering operational mode\n");
		}

		// Timeout/retry
		if (wifi_timeout == 0 && wifi_ctx.waiting) {
			if (++wifi_ctx.retries < WIFI_SM_MAX_RETRIES) {
				wifi_ctx.waiting = 0;
				wifi_ctx.progress = 0;
				printf("SM: MQTT CFG2 connect timeout, retry %d\n",
					   wifi_ctx.retries);
			} else {
				wifi_ctx.state   = SM_ERROR_RECOVERY;
				wifi_ctx.waiting = 0;
				printf("SM: MQTT CFG2 connect failed, recovery\n");
			}
		}
		break;

	case SM_OPERATIONAL:
		// Passive monitoring mode
		if (wifi_events.EV_DISCONNECT) {
			wifi_events.EV_DISCONNECT = 0;
			wifi_ctx.state = SM_WAIT_NET_JOIN;
			printf("SM: Network disconnected, waiting for reconnection\n");
		}

		// Optional: detect lost MQTT session (if peer closed)
		if (wifi_events.EV_PEER_CLOSED) {
			wifi_events.EV_PEER_CLOSED = 0;
			wifi_ctx.state = SM_MQTT_CFG2_CONNECT;
			printf("SM: MQTT peer closed, reconnecting CFG2\n");
		}

		// Detect successful network reconnect
		if ((wifi_ctx.progress & (FLAG_UUWLE | FLAG_UUNU)) ==
			(FLAG_UUWLE | FLAG_UUNU)) {
			wifi_ctx.progress &= ~(FLAG_UUWLE | FLAG_UUNU);
			wifi_ctx.state = SM_MQTT_CFG2_CONNECT;
			printf("SM: Network rejoined, reconnecting MQTT CFG2\n");
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

	default:
		wifi_ctx.state = SM_ERROR_RECOVERY;
		break;
	}
}
