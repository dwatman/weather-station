#include <stdio.h>
#include "i2c.h"
#include "opt4001.h"

volatile uint8_t opt4001_data[4];
volatile uint32_t opt4001_newdata = 0;

// Write a variable number of bytes (blocking)
static int SendBytes(uint8_t addr, const uint8_t *data, uint8_t len) {
	// Wait until I2C bus is free
	while (LL_I2C_IsActiveFlag_BUSY(I2C4));

	// Start transmission
	LL_I2C_HandleTransfer(I2C4, addr, LL_I2C_ADDRSLAVE_7BIT, len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	for (uint32_t i=0; i<len; i++) {
		// Wait for TX ready or detect error
		while (!LL_I2C_IsActiveFlag_TXIS(I2C4)) {

			if (LL_I2C_IsActiveFlag_NACK(I2C4)) {
				LL_I2C_ClearFlag_NACK(I2C4);
				LL_I2C_GenerateStopCondition(I2C4);
				return OPT4001_ERR_NACK;
			}

			if (LL_I2C_IsActiveFlag_BERR(I2C4) || LL_I2C_IsActiveFlag_ARLO(I2C4)) {
				LL_I2C_GenerateStopCondition(I2C4);
				return OPT4001_ERR_BUS;
			}
		}
		LL_I2C_TransmitData8(I2C4, data[i]);
	}

	// Wait for STOP flag (transfer complete)
	while (!LL_I2C_IsActiveFlag_STOP(I2C4)) {

		if (LL_I2C_IsActiveFlag_NACK(I2C4)) {
			LL_I2C_ClearFlag_NACK(I2C4);
			LL_I2C_GenerateStopCondition(I2C4);
			return OPT4001_ERR_NACK;
		}

		if (LL_I2C_IsActiveFlag_BERR(I2C4) || LL_I2C_IsActiveFlag_ARLO(I2C4)) {
			LL_I2C_GenerateStopCondition(I2C4);
			return OPT4001_ERR_BUS;
		}
	}

	LL_I2C_ClearFlag_STOP(I2C4);
	return OPT4001_OK;
}

// Reset and initialise the sensor
int opt4001_init(void) {
	int err;
	uint8_t txBuf[3];

	// Reset using general call
	txBuf[0] = 0x06;
	err = SendBytes(0x00, txBuf, 1);
	if (err != OPT4001_OK) return err;

	// Small delay after reset just to be safe
	HAL_Delay(2);

	// ----------------------------------------------------------------
	// Set conversion settings:
	// Continuous, 800ms, auto range, active low INT
	txBuf[0] = OPT4001_REG_SETUP0;
	txBuf[1] = 0x32;
	txBuf[2] = 0xF0;
	err = SendBytes(OPT4001_I2C_ADDR, txBuf, 3);
	if (err != OPT4001_OK) return err;

	// ----------------------------------------------------------------
	// Set interrupt settings:
	// Int after every conversion, enable I2C burst mode
	txBuf[0] = OPT4001_REG_SETUP1;
	txBuf[1] = 0x80;
	txBuf[2] = 0x15;
	err = SendBytes(OPT4001_I2C_ADDR, txBuf, 3);
	if (err != OPT4001_OK) return err;

	// ----------------------------------------------------------------
	// Set read pointer for data reads
	txBuf[0] = OPT4001_REG_DATA;
	err = SendBytes(OPT4001_I2C_ADDR, txBuf, 1);
	if (err != OPT4001_OK) return err;

	return OPT4001_OK;
}

// Begin the read (uses ISR)
void opt4001_StartRead(void) {
	// Make sure ready flag is cleared
	opt4001_newdata = 0;

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

	float lux = 0.0004375 * (mantissa << exponent);

	return lux;
}

// Handle interrupts for the read process
void opt4001_ISR(void) {
	static uint32_t idx = 0;

	// RX data ready
	if (LL_I2C_IsActiveFlag_RXNE(I2C4)) {
		opt4001_data[idx++] = LL_I2C_ReceiveData8(I2C4);
		if (idx >= 4) idx = 0;  // Wrap after 4 bytes to prevent overrun
	}

	// Transfer complete (STOP condition)
	if (LL_I2C_IsActiveFlag_STOP(I2C4)) {
		LL_I2C_ClearFlag_STOP(I2C4);

		LL_I2C_DisableIT_RX(I2C4);
		LL_I2C_DisableIT_STOP(I2C4);

		opt4001_newdata = 1;
		idx = 0;
	}
}
