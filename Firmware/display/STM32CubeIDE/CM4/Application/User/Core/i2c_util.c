#include <stdio.h>
#include "i2c.h"
#include "i2c_util.h"
#include "lps25hb.h"

I2c_Status_t i2c4_status;
extern volatile uint32_t opt4001_newdata;
extern volatile uint32_t lps25hb_newdata;

// Write a variable number of bytes (blocking)
int SendBytes(uint8_t addr, const uint8_t *data, uint8_t len) {
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
				return I2C_ERR_NACK;
			}

			if (LL_I2C_IsActiveFlag_BERR(I2C4) || LL_I2C_IsActiveFlag_ARLO(I2C4)) {
				LL_I2C_GenerateStopCondition(I2C4);
				return I2C_ERR_BUS;
			}
		}
		LL_I2C_TransmitData8(I2C4, data[i]);
	}

	// Wait for STOP flag (transfer complete)
	while (!LL_I2C_IsActiveFlag_STOP(I2C4)) {

		if (LL_I2C_IsActiveFlag_NACK(I2C4)) {
			LL_I2C_ClearFlag_NACK(I2C4);
			LL_I2C_GenerateStopCondition(I2C4);
			return I2C_ERR_NACK;
		}

		if (LL_I2C_IsActiveFlag_BERR(I2C4) || LL_I2C_IsActiveFlag_ARLO(I2C4)) {
			LL_I2C_GenerateStopCondition(I2C4);
			return I2C_ERR_BUS;
		}
	}

	LL_I2C_ClearFlag_STOP(I2C4);
	return I2C_OK;
}

// Handle interrupts for the read process
void i2c_ISR(I2c_Status_t *i2c) {
	// Transfer complete
	if (LL_I2C_IsEnabledIT_TC(I2C4) && LL_I2C_IsActiveFlag_TC(I2C4)) {
		if (i2c->phase == I2C_PHASE_WRITE_REG) {
			LL_I2C_DisableIT_TC(I2C4);
			// Register sent, now trigger repeated start for read
			i2c->phase = I2C_PHASE_READ_DATA;
			LL_I2C_HandleTransfer(I2C4, LPS25HB_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 3, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_RESTART_7BIT_READ);
		}
	}

	// RX data ready
	if (LL_I2C_IsEnabledIT_RX(I2C4) && LL_I2C_IsActiveFlag_RXNE(I2C4)) {
		i2c->rxDst[i2c->index++] = LL_I2C_ReceiveData8(I2C4);
		if (i2c->index >= I2C_BUFSZ) i2c->index = 0;  // Wrap to prevent overrun
	}

	// Transfer complete (STOP condition)
	if (LL_I2C_IsEnabledIT_STOP(I2C4) && LL_I2C_IsActiveFlag_STOP(I2C4)) {
		LL_I2C_ClearFlag_STOP(I2C4);

		LL_I2C_DisableIT_RX(I2C4);
		LL_I2C_DisableIT_STOP(I2C4);

		if (i2c->device == I2C_DEVICE_OPT4001)
			opt4001_newdata = 1;
		else
			lps25hb_newdata = 1;

		i2c->index = 0;
		i2c->state = I2C_STATE_IDLE;
	}
}
