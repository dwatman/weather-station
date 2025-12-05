#include <stdio.h>
#include "i2c.h"
#include "i2c_util.h"
#include "lps25hb.h"

volatile uint8_t lps25hb_data[I2C_BUFSZ];
volatile uint32_t lps25hb_newdata = 0;

// Reset and initialise the sensor
int lps25hb_init(void) {
	int err;
	uint8_t txBuf[3];

	// Reset device
	txBuf[0] = LPS25HB_REG_CTRL_REG2;
	txBuf[1] = 0x04;
	err = SendBytes(LPS25HB_I2C_ADDR, txBuf, 2);
	if (err != I2C_OK) return err;
	HAL_Delay(2); // Small delay

	// Set BOOT to load calibration etc
	txBuf[0] = LPS25HB_REG_CTRL_REG2;
	txBuf[1] = 0x80;
	err = SendBytes(LPS25HB_I2C_ADDR, txBuf, 2);
	if (err != I2C_OK) return err;
	HAL_Delay(4); // Small delay (>2.2ms)

	// ----------------------------------------------------------------

	// Set resolution to max
	txBuf[0] = LPS25HB_REG_RES_CONF;
	txBuf[1] = 0x0F;
	err = SendBytes(LPS25HB_I2C_ADDR, txBuf, 2);
	if (err != I2C_OK) return err;

	// Turn on device, set output rate to 1 Hz
	txBuf[0] = LPS25HB_REG_CTRL_REG1;
	txBuf[1] = 0x90;
	err = SendBytes(LPS25HB_I2C_ADDR, txBuf, 2);
	if (err != I2C_OK) return err;

	// Disable FIFO
	txBuf[0] = LPS25HB_REG_CTRL_REG2;
	txBuf[1] = 0x00;
	err = SendBytes(LPS25HB_I2C_ADDR, txBuf, 2);
	if (err != I2C_OK) return err;

	// Set interrupt active low, push-pull
	txBuf[0] = LPS25HB_REG_CTRL_REG3;
	txBuf[1] = 0x80;
	err = SendBytes(LPS25HB_I2C_ADDR, txBuf, 2);
	if (err != I2C_OK) return err;

	// Set interrupt on data ready
	txBuf[0] = LPS25HB_REG_CTRL_REG4;
	txBuf[1] = 0x01;
	err = SendBytes(LPS25HB_I2C_ADDR, txBuf, 2);
	if (err != I2C_OK) return err;

	// Set FIFO mode to bypass
	txBuf[0] = LPS25HB_REG_FIFO_CTRL;
	txBuf[1] = 0x00;
	err = SendBytes(LPS25HB_I2C_ADDR, txBuf, 2);
	if (err != I2C_OK) return err;

	return I2C_OK;
}

// Begin the read (uses ISR)
void lps25hb_StartRead(I2c_Status_t *i2c) {
	// Make sure data ready flag is cleared
	lps25hb_newdata = 0;

	// Set up I2C parameters
	i2c->device = I2C_DEVICE_LPS25HB;
	i2c->state = I2C_STATE_BUSY;
	i2c->phase = I2C_PHASE_WRITE_REG;
	i2c->index = 0;
	i2c->rxDst = lps25hb_data;

	// Wait until I2C bus is free
	while (LL_I2C_IsActiveFlag_BUSY(I2C4));

	// Configure I2C write transfer for data register address
	LL_I2C_HandleTransfer(I2C4, LPS25HB_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

	// Set high bit of register address for consecutive reads
	LL_I2C_TransmitData8(I2C4, LPS25HB_REG_PRESS_OUT_XL | 0x80);

	// Enable I2C interrupts for RX, TC, STOP events
	LL_I2C_EnableIT_TC(I2C4);
	LL_I2C_EnableIT_RX(I2C4);
	LL_I2C_EnableIT_STOP(I2C4);
}

// Convert raw data to hPa
float lps25hb_Convert(void) {
	//uint8_t count = opt4001_data[3] >> 4;
	//uint8_t crc = opt4001_data[3] & 0x0F;

	uint32_t raw = ((uint32_t)(lps25hb_data[2] & 0x7F) << 16) |
					((uint32_t)lps25hb_data[1] << 8) |
					lps25hb_data[0];

	float hpa = raw/4096.0;

	return hpa;
}

