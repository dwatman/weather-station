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
    FLOW_GLOBAL_VARIABLE_TIME_STR = 0,
    FLOW_GLOBAL_VARIABLE_AMPM_STR = 1,
    FLOW_GLOBAL_VARIABLE_T1_STR = 2,
    FLOW_GLOBAL_VARIABLE_T2_STR = 3,
    FLOW_GLOBAL_VARIABLE_H1_STR = 4,
    FLOW_GLOBAL_VARIABLE_H2_STR = 5,
    FLOW_GLOBAL_VARIABLE_PRESS_STR = 6,
    FLOW_GLOBAL_VARIABLE_LUX_STR = 7,
    FLOW_GLOBAL_VARIABLE_BL_STR = 8,
    FLOW_GLOBAL_VARIABLE_RSSI_STR = 9,
    FLOW_GLOBAL_VARIABLE_SOIL1_INT = 10,
    FLOW_GLOBAL_VARIABLE_SOIL2_INT = 11,
    FLOW_GLOBAL_VARIABLE_SOIL3_INT = 12,
    FLOW_GLOBAL_VARIABLE_SOIL4_INT = 13
};

// Native global variables

extern const char *get_var_time_str();
extern void set_var_time_str(const char *value);
extern const char *get_var_ampm_str();
extern void set_var_ampm_str(const char *value);
extern const char *get_var_t1_str();
extern void set_var_t1_str(const char *value);
extern const char *get_var_t2_str();
extern void set_var_t2_str(const char *value);
extern const char *get_var_h1_str();
extern void set_var_h1_str(const char *value);
extern const char *get_var_h2_str();
extern void set_var_h2_str(const char *value);
extern const char *get_var_press_str();
extern void set_var_press_str(const char *value);
extern const char *get_var_lux_str();
extern void set_var_lux_str(const char *value);
extern const char *get_var_bl_str();
extern void set_var_bl_str(const char *value);
extern const char *get_var_rssi_str();
extern void set_var_rssi_str(const char *value);
extern int32_t get_var_soil1_int();
extern void set_var_soil1_int(int32_t value);
extern int32_t get_var_soil2_int();
extern void set_var_soil2_int(int32_t value);
extern int32_t get_var_soil3_int();
extern void set_var_soil3_int(int32_t value);
extern int32_t get_var_soil4_int();
extern void set_var_soil4_int(int32_t value);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/