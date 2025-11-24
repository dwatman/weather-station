#include <stdio.h>
#include <string.h>
#include "vars.h"

char test_int[32] = { 0 };

const char *get_var_test_int() {
    return test_int;
}

void set_var_test_int(int32_t value) {
	char tmp[32];

	//snprintf(test_int, 32, "Test %i", (int)value);

	snprintf(tmp, 32, "Test %i", (int)value);
    strncpy(test_int, tmp, sizeof(test_int) / sizeof(char));
    test_int[sizeof(test_int) / sizeof(char) - 1] = 0;
}

int32_t test_bar;

int32_t get_var_test_bar() {
    return test_bar;
}

void set_var_test_bar(int32_t value) {
    test_bar = value;
}
