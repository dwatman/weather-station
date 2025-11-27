#ifndef APPLICATION_USER_UI_UI_IO_H_
#define APPLICATION_USER_UI_UI_IO_H_

const char *get_var_test_int();
const char *get_var_t1_str();
const char *get_var_t2_str();
const char *get_var_h1_str();
const char *get_var_h2_str();
const char *get_var_lux_str();
const char *get_var_rssi_str();
int32_t get_var_test_bar();

void set_var_test_int(const char *value);
void set_var_t1_str(const char *value);
void set_var_t2_str(const char *value);
void set_var_h1_str(const char *value);
void set_var_h2_str(const char *value);
void set_var_lux_str(const char *value);
void set_var_rssi_str(const char *value);
void set_var_test_bar(int32_t value);

void set_test(int value);
void set_t1(float val);
void set_t2(float val);
void set_h1(float val);
void set_h2(float val);
void set_lux(float val);
void set_rssi(int val);

#endif
