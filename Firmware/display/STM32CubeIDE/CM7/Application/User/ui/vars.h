#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations



// Flow global variables

enum FlowGlobalVariables {
    FLOW_GLOBAL_VARIABLE_TEST_INT = 0,
    FLOW_GLOBAL_VARIABLE_TEST_BAR = 1,
    FLOW_GLOBAL_VARIABLE_T1_STR = 2,
    FLOW_GLOBAL_VARIABLE_T2_STR = 3,
    FLOW_GLOBAL_VARIABLE_H1_STR = 4,
    FLOW_GLOBAL_VARIABLE_H2_STR = 5,
    FLOW_GLOBAL_VARIABLE_LUX_STR = 6,
    FLOW_GLOBAL_VARIABLE_RSSI_STR = 7
};

// Native global variables

extern const char *get_var_test_int();
extern void set_var_test_int(const char *value);
extern int32_t get_var_test_bar();
extern void set_var_test_bar(int32_t value);
extern const char *get_var_t1_str();
extern void set_var_t1_str(const char *value);
extern const char *get_var_t2_str();
extern void set_var_t2_str(const char *value);
extern const char *get_var_h1_str();
extern void set_var_h1_str(const char *value);
extern const char *get_var_h2_str();
extern void set_var_h2_str(const char *value);
extern const char *get_var_lux_str();
extern void set_var_lux_str(const char *value);
extern const char *get_var_rssi_str();
extern void set_var_rssi_str(const char *value);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/