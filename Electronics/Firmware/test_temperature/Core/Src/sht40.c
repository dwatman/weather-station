#include "main.h"
#include "i2c_util.h"
#include "sht40.h"

void sht40_init(sht40_t *h, i2c_t *d) {

	static_assert(I2C_SHT40_DATA_BYTES <= I2C_BUF_SIZE, "I2C buffer is too small to fit SHT40 data");

	h->i2c = d;
	//h->state = SHT40_STATE_READY;
	h->timeout = 0;
	h->new_data = 0;
	h->err = 0;
	h->temp_raw = 0;
	h->humid_raw = 0;
}

void sht40_reset(sht40_t *h) {
	LL_I2C_HandleTransfer(h->i2c->device, I2C_SHT40_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_I2C_TransmitData8(h->i2c->device, I2C_SHT40_CMD_RESET);
}

// Set duration to I2C_SHT40_CMD_MEAS_HI_ACC / I2C_SHT40_CMD_MEAS_MED_ACC / I2C_SHT40_CMD_MEAS_LO_ACC
int sht40_start_measurement(sht40_t *h, uint8_t duration) {

	if (h->i2c->state != I2C_STATE_IDLE) {
		printf("WARNING: Called sht40_start_measurement but I2C not idle (%u)\n", h->i2c->state);
		return -1;
	}

	// Update state
	h->new_data = 0;
	h->err = 0; //unused?

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

	// Update I2C state
	h->i2c->state = I2C_STATE_WRITING;
	h->i2c->done = 0;

	// Begin the transfer
	LL_I2C_HandleTransfer(h->i2c->device, I2C_SHT40_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_I2C_TransmitData8(h->i2c->device, duration);

	return 0;
}

int sht40_read_measurement(sht40_t *h) {
	// TODO: check for error state

	h->i2c->state = I2C_STATE_READING;
	h->i2c->length = I2C_SHT40_DATA_BYTES;
	h->i2c->index = 0;
	LL_I2C_HandleTransfer(h->i2c->device, I2C_SHT40_ADDR, LL_I2C_ADDRSLAVE_7BIT, I2C_SHT40_DATA_BYTES, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

	return 0;
}

int sht40_process_measurement(sht40_t *h) {
	uint8_t crc_temp, crc_humid;

	crc_temp = h->i2c->buf[2];
	crc_humid = h->i2c->buf[5];
	// TODO: CRC check

	h->temp_raw  = ((uint16_t)h->i2c->buf[0] << 8) | (uint16_t)h->i2c->buf[1];
	h->humid_raw = ((uint16_t)h->i2c->buf[3] << 8) | (uint16_t)h->i2c->buf[4];

	h->new_data = 1;

	return 0;
}
