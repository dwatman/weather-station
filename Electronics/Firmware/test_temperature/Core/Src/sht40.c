#include "main.h"
#include "i2c_util.h"
#include "sht40.h"

void sht40_reset(i2c_t *h) {
	LL_I2C_HandleTransfer(h->i2c, (I2C_SHT40_ADDR << 1), LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_I2C_TransmitData8(h->i2c, I2C_SHT40_CMD_RESET);
}

// Set duration to I2C_SHT40_CMD_MEAS_HI_ACC / I2C_SHT40_CMD_MEAS_MED_ACC / I2C_SHT40_CMD_MEAS_LO_ACC
int sht40_start_measurement(i2c_t *h, uint8_t duration) {

	if (h->state != I2C_STATE_IDLE) {
		printf("ERROR: Called sht40_start_measurement but I2C not idle (%u)\n", h->state);
		return -1;
	}

	// Set the timeout according to sensor measurement time
	switch (duration) {
		case I2C_SHT40_CMD_MEAS_LO_ACC:
			h->timeout = 3; // At least 1.6 ms
			break;
		case I2C_SHT40_CMD_MEAS_MED_ACC:
			h->timeout = 6; // At least 4.5 ms
			break;
		case I2C_SHT40_CMD_MEAS_HI_ACC:
			h->timeout = 10; // At least 8.3 ms
			break;
		default:
			printf("ERROR: Invalid duration in sht40_start_measurement\n");
			return -2;
			break;
	}

	// Update state
	h->state = I2C_STATE_WRITING;
	h->done = 0;
	h->err = 0;
	h->index = 0;

	// Begin the transfer
	LL_I2C_HandleTransfer(h->i2c, (I2C_SHT40_ADDR << 1), LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_I2C_TransmitData8(h->i2c, duration);
	return 0;
}
/*
i2c_sht40.address = (I2C_SHT40_ADDR << 1);
i2c_sht40.command = I2C_SHT40_CMD_RESET;
LL_I2C_HandleTransfer(i2c_sht40.i2c, i2c_sht40.address, LL_I2C_ADDRSLAVE_7BIT, 0, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
LL_I2C_TransmitData8(i2c_sht40.i2c, i2c_sht40.command);
*/
