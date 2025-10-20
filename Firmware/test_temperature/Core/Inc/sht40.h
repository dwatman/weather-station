#ifndef INC_SHT40_H_
#define INC_SHT40_H_

#include <stdint.h>

#include "i2c_util.h"

#define I2C_SHT40_ADDR (0x44 << 1)
#define I2C_SHT40_DATA_BYTES 6

#define I2C_SHT40_CMD_MEAS_HI_ACC 0xFD
#define I2C_SHT40_CMD_MEAS_MED_ACC 0xF6
#define I2C_SHT40_CMD_MEAS_LO_ACC 0xE0
#define I2C_SHT40_CMD_RESET 0x94

typedef enum {
	SHT40_STATE_READY = 0,
	SHT40_STATE_WRITING = 1,
	SHT40_STATE_READING = 2,
	SHT40_STATE_DONE = 3,
	SHT40_STATE_ERROR = 4,
} SHT40_STATE;

// Structure to hold data for SHT40 sensor
typedef struct {
	i2c_t *i2c;  // Pointer to I2C structure

    //volatile SHT40_STATE state;  // Current transaction state
    volatile uint8_t timeout;    // Counter decremented in ISR, used for delays (ms)
    volatile uint8_t new_data;   // New data is available
    volatile uint8_t err;        // 0 = no error
    volatile uint16_t temp_raw;  // Raw temperature value
    volatile uint16_t humid_raw; // Raw humidity value
} sht40_t;

void sht40_init(sht40_t *h, i2c_t *d);
void sht40_reset(sht40_t *h);
int sht40_start_measurement(sht40_t *h, uint8_t duration);
int sht40_read_measurement(sht40_t *h);
int sht40_process_measurement(sht40_t *h);

#endif
