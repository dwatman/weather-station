#include <stdio.h>
#include <string.h>
#include "vars.h"
#include "stm32h7xx_hal.h"
#include "shared.h"

// Strings to set LVGL labels
char test_int[100] = { 0 };
char t1_str[100] = { 0 };
char t2_str[100] = { 0 };
char h1_str[100] = { 0 };
char h2_str[100] = { 0 };
char lux_str[100] = { 0 };
char rssi_str[100] = { 0 };
int32_t test_bar;

// Fixed EEZ Studio functions
const char *get_var_test_int() { return test_int; }
const char *get_var_t1_str() { return t1_str; }
const char *get_var_t2_str() { return t2_str; }
const char *get_var_h1_str() { return h1_str; }
const char *get_var_h2_str() { return h2_str; }
const char *get_var_lux_str() { return lux_str; }
const char *get_var_rssi_str() { return rssi_str; }
int32_t get_var_test_bar() { return test_bar; }

void set_var_test_int(const char *value) {
    strncpy(test_int, value, sizeof(test_int) / sizeof(char));
    test_int[sizeof(test_int) / sizeof(char) - 1] = 0;
}

void set_var_t1_str(const char *value) {
    strncpy(t1_str, value, sizeof(t1_str) / sizeof(char));
    t1_str[sizeof(t1_str) / sizeof(char) - 1] = 0;
}

void set_var_t2_str(const char *value) {
    strncpy(t2_str, value, sizeof(t2_str) / sizeof(char));
    t2_str[sizeof(t2_str) / sizeof(char) - 1] = 0;
}

void set_var_h1_str(const char *value) {
    strncpy(h1_str, value, sizeof(h1_str) / sizeof(char));
    h1_str[sizeof(h1_str) / sizeof(char) - 1] = 0;
}

void set_var_h2_str(const char *value) {
    strncpy(h2_str, value, sizeof(h2_str) / sizeof(char));
    h2_str[sizeof(h2_str) / sizeof(char) - 1] = 0;
}

void set_var_lux_str(const char *value) {
    strncpy(lux_str, value, sizeof(lux_str) / sizeof(char));
    lux_str[sizeof(lux_str) / sizeof(char) - 1] = 0;
}

void set_var_rssi_str(const char *value) {
    strncpy(rssi_str, value, sizeof(rssi_str) / sizeof(char));
    rssi_str[sizeof(rssi_str) / sizeof(char) - 1] = 0;
}

void set_var_test_bar(int32_t value) {
    test_bar = value;
}

// Functions to create formatted text for the labels

void set_test(int value) {
	char tmp[32];

	snprintf(tmp, 32, "Test %i", value);
	set_var_test_int(tmp);
}

void set_t1(float val) {
	char tmp[32];

	snprintf(tmp, 32, "%.1f", val);
	set_var_t1_str(tmp);
}

void set_t2(float val) {
	char tmp[32];

	snprintf(tmp, 32, "%.1f", val);
	set_var_t2_str(tmp);
}

void set_h1(float val) {
	char tmp[32];

	snprintf(tmp, 32, "%.1f", val);
	set_var_h1_str(tmp);
}

void set_h2(float val) {
	char tmp[32];

	snprintf(tmp, 32, "%.1f", val);
	set_var_h2_str(tmp);
}

void set_lux(float val) {
	char tmp[32];

	snprintf(tmp, 32, "%.1f", val);
	set_var_lux_str(tmp);
}

void set_rssi(int val) {
	char tmp[32];

	snprintf(tmp, 32, "%+i", val);
	set_var_rssi_str(tmp);
}

#include "gpio.h"
// Triggered from interrupt when M4 has updated shared memory
void HAL_HSEM_FreeCallback(uint32_t SemMask) {
	if (SemMask & __HAL_HSEM_SEMID_TO_MASK(HSEM_ID_1)) 	{
		// Clear flag just in case
		__HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_1));
		// Reactivate notification
		HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_1));
	}
	HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_10); // User LED
}
