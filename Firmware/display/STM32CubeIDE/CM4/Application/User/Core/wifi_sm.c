#include <stdio.h>
#include <string.h>

#include "wifi_sm.h"
#include "network.h"  // for tx1Buf, printfCircBuf(), emptyRx1Buffer()
#include "usart_util.h"
#include "stm32h7xx_ll_usart.h"

const char pressure_config[] =
"{\"name\":\"Pressure\","
"\"uniq_id\":\"DW_display_pressure\","
"\"stat_t\":\"dw_display/state\","
"\"val_tpl\":\"{{ value_json.pressure }}\","
"\"unit_of_meas\":\"hPa\","
"\"dev_cla\":\"pressure\","
"\"stat_cla\":\"measurement\","
"\"dev\":{\"ids\":[\"DW_display\"],\"name\":\"DW_display\",\"mf\":\"DW Custom\",\"mdl\":\"Display v1\"}}";

const char light_config[] =
"{\"name\":\"Light\","
"\"uniq_id\":\"DW_display_light\","
"\"stat_t\":\"dw_display/state\","
"\"val_tpl\":\"{{ value_json.light }}\","
"\"unit_of_meas\":\"lx\","
"\"dev_cla\":\"illuminance\","
"\"stat_cla\":\"measurement\","
"\"dev\":{\"ids\":[\"DW_display\"],\"name\":\"DW_display\",\"mf\":\"DW Custom\",\"mdl\":\"Display v1\"}}";

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
			// NETWORK JOINED: Start the Sequence
			wifi_ctx.sequence_step = SEQ_CONFIG_PRESSURE; // Start with Pressure
			wifi_ctx.state = SM_MQTT_SEQ_CONNECT;

			wifi_ctx.waiting = 0;
			wifi_ctx.retries = 0;
			wifi_ctx.progress = 0;
			printf("SM: Network joined, starting MQTT sequence\n");
		}
		break;

	// -------------------------------------------------------------------------
	// GENERIC CONNECT STATE
	// Selects topic based on wifi_ctx.sequence_step
	// -------------------------------------------------------------------------
	case SM_MQTT_SEQ_CONNECT:
		if (!wifi_ctx.waiting) {
			char topic_buf[TOPIC_BUF_LEN];

			// 1. Determine Topic and Next Payload based on Step
			switch(wifi_ctx.sequence_step) {
				case SEQ_CONFIG_PRESSURE:
					snprintf(topic_buf, TOPIC_BUF_LEN, TOPIC_DISC_PRES);
					wifi_ctx.payload_type = PAYLOAD_JSON_PRES;
					snprintf((char *)wifi_ctx.payload, PAYLOAD_BUF_LEN, pressure_config);
					wifi_ctx.payload_len = strlen((char *)wifi_ctx.payload);
					break;
				case SEQ_CONFIG_LIGHT:
					snprintf(topic_buf, TOPIC_BUF_LEN, TOPIC_DISC_LITE);
					wifi_ctx.payload_type = PAYLOAD_JSON_LITE;
					snprintf((char *)wifi_ctx.payload, PAYLOAD_BUF_LEN, light_config);
					wifi_ctx.payload_len = strlen((char *)wifi_ctx.payload);
					break;
				case SEQ_DATA_CONNECT:
					snprintf(topic_buf, TOPIC_BUF_LEN, TOPIC_DATA);
					wifi_ctx.payload_type = PAYLOAD_JSON_NONE; // No initial payload for data connection
					break;
				default:
					wifi_ctx.state = SM_OPERATIONAL; // Should not happen
					return;
			}

			// 2. Send Connect Command
			printfCircBuf(&tx1Buf,
				"AT+UDCP=at-mqtt://192.168.0.200:1883/"
				"?client=NINA_PUB&user=mqtt_user&passwd=MQ.jaygram"
				"&pt=%s&st=%s&encr=0&qos=0\r\n", topic_buf, TOPIC_LISTEN);

			wifi_ctx.waiting = 1;
			wifi_ctx.progress = 0;
			wifi_ctx.peer_handle = -1;
			wifi_ctx.retries = 0;
			wifi_timeout = wifi_ctx.timeout_ms;
			printf("SM: Connect SEQ Step %d (Topic: %s)\n", wifi_ctx.sequence_step, topic_buf);
		}

		// Handle Responses (+UDCP:x and +UUDPC)
		if (wifi_events.EV_UDCP) {
			wifi_events.EV_UDCP = 0;
			wifi_ctx.progress |= FLAG_UDCP;
			printf("SM: +UDCP received, peer=%d\n", wifi_ctx.peer_handle);
		}
		if (wifi_events.EV_UUDPC) {
			wifi_events.EV_UUDPC = 0;
			wifi_ctx.progress |= FLAG_UUDPC;
		}

		if ((wifi_ctx.progress & (FLAG_UDCP | FLAG_UUDPC)) == (FLAG_UDCP | FLAG_UUDPC)) {
			wifi_ctx.waiting = 0;

			// Check if initialisation is complete
			if (wifi_ctx.sequence_step == SEQ_DATA_CONNECT) {
				wifi_ctx.state = SM_OPERATIONAL; // We are done!
				printf("SM: Data Connection Established. Operational.\n");
			} else {
				wifi_ctx.state = SM_MQTT_SEQ_WRITE_CMD; // Go to Binary Write
				printf("SM: Connected, preparing to write config JSON\n");
			}
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

	// -------------------------------------------------------------------------
	// BINARY WRITE: Step 1 - Send AT+UDATW and Length
	// -------------------------------------------------------------------------
	case SM_MQTT_SEQ_WRITE_CMD:
		if (!wifi_ctx.waiting) {
			printfCircBuf(&tx1Buf, "AT+UDATW=%d,2,%d\r\n", // Binary mode
						  wifi_ctx.peer_handle,
						  wifi_ctx.payload_len);

			wifi_ctx.waiting = 1;
			wifi_timeout = 5000; // Wait for prompt
			wifi_ctx.retries = 0;
			printf("SM: Sent UDATW command, waiting for '>'\n");
		}

		// Waiting for the prompt '>'
		if (wifi_events.EV_PROMPT) {
			wifi_events.EV_PROMPT = 0;
			wifi_ctx.waiting = 0;
			wifi_ctx.state = SM_MQTT_SEQ_WRITE_DATA;
			printf("SM: Prompt received, sending raw data\n");
		}
		else if (event_error) {
			// Handle error
			wifi_ctx.state = SM_ERROR_RECOVERY;
		}
		// ... (Timeout logic) ...
		break;

	// -------------------------------------------------------------------------
	// BINARY WRITE: Step 2 - Send Raw Data
	// -------------------------------------------------------------------------
	case SM_MQTT_SEQ_WRITE_DATA:
		if (!wifi_ctx.waiting) {
			// Send the raw buffer.
			writeToCircBuf(&tx1Buf, wifi_ctx.payload, wifi_ctx.payload_len);
			LL_USART_EnableIT_TXFE(USART1);

			wifi_ctx.waiting = 1;
			wifi_timeout = 5000;
			printf("SM: Raw data sent, waiting for OK\n");
		}

		if (event_ok) {
			wifi_ctx.waiting = 0;
			wifi_ctx.state = SM_MQTT_SEQ_DISCONNECT;
			printf("SM: Data write OK. Disconnecting this handle.\n");
		}
		else if (event_error) {
			 wifi_ctx.state = SM_ERROR_RECOVERY;
		}
		break;

	// -------------------------------------------------------------------------
	// GENERIC DISCONNECT
	// -------------------------------------------------------------------------
	case SM_MQTT_SEQ_DISCONNECT:
		if (!wifi_ctx.waiting) {
			printfCircBuf(&tx1Buf, "AT+UDCPC=%d\r\n", wifi_ctx.peer_handle);
			wifi_ctx.waiting = 1;
			wifi_ctx.progress = 0;
			wifi_timeout = 2000;
		}

		if (event_ok) { event_ok = 0; wifi_ctx.progress |= FLAG_OK; }
		if (wifi_events.EV_PEER_CLOSED) { wifi_events.EV_PEER_CLOSED = 0; wifi_ctx.progress |= FLAG_UUDPD; }

		if ((wifi_ctx.progress & (FLAG_OK | FLAG_UUDPD)) == (FLAG_OK | FLAG_UUDPD)) {
			wifi_ctx.waiting = 0;
			wifi_ctx.peer_handle = -1;

			// ADVANCE THE SEQUENCE
			wifi_ctx.sequence_step++;
			wifi_ctx.state = SM_MQTT_SEQ_CONNECT; // Loop back to connect for next step
			printf("SM: Disconnect complete, advancing to Step %d\n", wifi_ctx.sequence_step);
		}
		// ... (Timeout logic) ...
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
			wifi_ctx.sequence_step = SEQ_DATA_CONNECT; // Reconnect to data topic
			wifi_ctx.state = SM_MQTT_SEQ_CONNECT;
			printf("SM: MQTT peer closed, reconnecting CFG2\n");
		}

		// Detect successful network reconnect
		if ((wifi_ctx.progress & (FLAG_UUWLE | FLAG_UUNU)) ==
			(FLAG_UUWLE | FLAG_UUNU)) {
			wifi_ctx.progress &= ~(FLAG_UUWLE | FLAG_UUNU);
			wifi_ctx.sequence_step = SEQ_DATA_CONNECT; // Reconnect to data topic
			wifi_ctx.state = SM_MQTT_SEQ_CONNECT;
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
