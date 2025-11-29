#include <stdio.h>
#include <string.h>
#include "screens.h"
#include "vars.h"
#include "stm32h7xx_hal.h"
#include "shared.h"

// Strings to set LVGL labels
char test_int[100] = { 0 };
char time_str[8] = { 0 };
char ampm_str[4] = { 0 };
char t1_str[16] = { 0 };
char t2_str[16] = { 0 };
char h1_str[16] = { 0 };
char h2_str[16] = { 0 };
char lux_str[16] = { 0 };
char rssi_str[8] = { 0 };
int32_t test_bar;

// Fixed EEZ Studio functions
const char *get_var_test_int() { return test_int; }
const char *get_var_time_str() { return time_str; }
const char *get_var_ampm_str() { return ampm_str; }
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

void set_var_time_str(const char *value) {
    strncpy(time_str, value, sizeof(time_str) / sizeof(char));
    time_str[sizeof(time_str) / sizeof(char) - 1] = 0;
}

void set_var_ampm_str(const char *value) {
    strncpy(ampm_str, value, sizeof(ampm_str) / sizeof(char));
    ampm_str[sizeof(ampm_str) / sizeof(char) - 1] = 0;
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

void set_time(uint32_t val) {
	char tmp[8];

	uint8_t hr = (val >> 8) & 0xFF;
	uint8_t min = val & 0xFF;

	// Print 12-hour time
	if (hr <= 12) {
		snprintf(tmp, 8, "%u:%02u", hr, min);
		set_var_time_str(tmp);
	}
	else {
		snprintf(tmp, 8, "%u:%02u", hr-12, min);
		set_var_time_str(tmp);
	}

	// Set AM/PM
	if (hr < 12) {
		snprintf(tmp, 4, "AM");
		set_var_ampm_str(tmp);
	}
	else {
		snprintf(tmp, 4, "PM");
		set_var_ampm_str(tmp);
	}
}

void set_t1(float val) {
	char tmp[16];

	snprintf(tmp, 16, "%.1f", val);
	set_var_t1_str(tmp);
}

void set_t2(float val) {
	char tmp[16];

	snprintf(tmp, 16, "%.1f", val);
	set_var_t2_str(tmp);
}

void set_h1(float val) {
	char tmp[16];

	snprintf(tmp, 16, "%.1f", val);
	set_var_h1_str(tmp);
}

void set_h2(float val) {
	char tmp[16];

	snprintf(tmp, 16, "%.1f", val);
	set_var_h2_str(tmp);
}

void set_lux(float val) {
	char tmp[16];

	snprintf(tmp, 16, "%.1f", val);
	set_var_lux_str(tmp);
}

void set_rssi(int val) {
	char tmp[8];

	snprintf(tmp, 8, "%+i", val);
	set_var_rssi_str(tmp);
}

void set_wifi_colour(lv_color_t new_colour) {
    lv_obj_t *obj = objects.obj1;

    // Change colour
    lv_obj_set_style_image_recolor(obj, new_colour, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Ensure recolor opacity is set (255 = fully applied)
    //lv_obj_set_style_image_recolor_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Optional: force refresh if needed
    lv_obj_refresh_style(obj, LV_PART_MAIN, LV_STYLE_PROP_ANY);
}
