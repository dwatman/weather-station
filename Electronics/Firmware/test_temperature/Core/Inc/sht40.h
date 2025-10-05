#ifndef INC_SHT40_H_
#define INC_SHT40_H_

#include <stdint.h>

#include "i2c_util.h"

#define I2C_SHT40_ADDR 0x44

#define I2C_SHT40_CMD_MEAS_HI_ACC 0xFD
#define I2C_SHT40_CMD_MEAS_MED_ACC 0xF6
#define I2C_SHT40_CMD_MEAS_LO_ACC 0xE0
#define I2C_SHT40_CMD_RESET 0x94

void sht40_reset(i2c_t *h);
int sht40_start_measurement(i2c_t *h, uint8_t duration);

#endif
