#include <stdio.h>
#include "i2c.h"
#include "opt4001.h"

volatile uint8_t opt4001_data[I2C_BUFSZ];
volatile uint32_t opt4001_newdata = 0;

// Reset and initialise the sensor
int opt4001_init(void) {
	int err;
	uint8_t txBuf[3];

	// Reset using general call
	txBuf[0] = 0x06;
	err = SendBytes(0x00, txBuf, 1);
	if (err != I2C_OK) return err;

	// Small delay after reset just to be safe
	HAL_Delay(2);

	// ----------------------------------------------------------------
	// Set conversion settings:
	// Continuous, 800ms, auto range, active low INT
	txBuf[0] = OPT4001_REG_SETUP0;
	txBuf[1] = 0x32;
	txBuf[2] = 0xF0;
	err = SendBytes(OPT4001_I2C_ADDR, txBuf, 3);
	if (err != I2C_OK) return err;

	// ----------------------------------------------------------------
	// Set interrupt settings:
	// Int after every conversion, enable I2C burst mode
	txBuf[0] = OPT4001_REG_SETUP1;
	txBuf[1] = 0x80;
	txBuf[2] = 0x15;
	err = SendBytes(OPT4001_I2C_ADDR, txBuf, 3);
	if (err != I2C_OK) return err;

	// ----------------------------------------------------------------
	// Set read pointer for data reads
	txBuf[0] = OPT4001_REG_DATA;
	err = SendBytes(OPT4001_I2C_ADDR, txBuf, 1);
	if (err != I2C_OK) return err;

	return I2C_OK;
}

// Begin the read (uses ISR)
void opt4001_StartRead(I2c_Status_t *i2c) {
	// Make sure data ready flag is cleared
	opt4001_newdata = 0;

	// Set up I2C parameters
	i2c->device = I2C_DEVICE_OPT4001;
	i2c->state = I2C_STATE_BUSY;
	i2c->phase = I2C_PHASE_READ_DATA;
	i2c->index = 0;
	i2c->rxDst = opt4001_data;

	// Wait until I2C bus is free
	while (LL_I2C_IsActiveFlag_BUSY(I2C4));

	// Configure I2C read transfer: 4 bytes, autoend, start read
	LL_I2C_HandleTransfer(I2C4, OPT4001_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 4, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

	// Enable I2C interrupts for RX and STOP events
	LL_I2C_EnableIT_RX(I2C4);
	LL_I2C_EnableIT_STOP(I2C4);
}

// Convert raw data to lux
float opt4001_Convert(void) {
	uint8_t exponent = opt4001_data[0] >> 4;
	//uint8_t count = opt4001_data[3] >> 4;
	//uint8_t crc = opt4001_data[3] & 0x0F;

	uint32_t mantissa = ((uint32_t)(opt4001_data[0] & 0x0F) << 16) |
					((uint32_t)opt4001_data[1] << 8) |
					opt4001_data[2];

	// Calculate float value
	float lux = 0.0004375 * (mantissa << exponent);

	return lux;
}
