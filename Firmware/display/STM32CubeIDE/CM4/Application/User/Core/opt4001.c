#include "i2c.h"
#include "opt4001.h"

int opt4001_init(void) {
	// Reset using general call
	LL_I2C_HandleTransfer(I2C4, 0x00, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_I2C_TransmitData8(I2C4, 0x06);

	// Wait for transfer complete and clear STOP flag
	while (!LL_I2C_IsActiveFlag_STOP(I2C4));
	LL_I2C_ClearFlag_STOP(I2C4);

	// Small delay after reset just to be safe
	HAL_Delay(5);

	// ----------------------------------------------------------------
	// Set conversion settings:
	// Continuous, 800ms, auto range, active low INT
	LL_I2C_HandleTransfer(I2C4, OPT4001_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 3, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	// Wait for TX ready (TXIS) and send byte
	while (!LL_I2C_IsActiveFlag_TXIS(I2C4));
	LL_I2C_TransmitData8(I2C4, OPT4001_REG_SETUP0);

	// Wait for TX ready (TXIS) and send byte
	while (!LL_I2C_IsActiveFlag_TXIS(I2C4));
	LL_I2C_TransmitData8(I2C4, 0x32);

	// Wait for TX ready (TXIS) and send byte
	while (!LL_I2C_IsActiveFlag_TXIS(I2C4));
	LL_I2C_TransmitData8(I2C4, 0xF0);

	// Wait for transfer complete and clear STOP flag
	while (!LL_I2C_IsActiveFlag_STOP(I2C4));
	LL_I2C_ClearFlag_STOP(I2C4);

	// ----------------------------------------------------------------
	// Set interrupt settings:
	// Int after every conversion, enable I2C burst mode
	LL_I2C_HandleTransfer(I2C4, OPT4001_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 3, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	// Wait for TX ready (TXIS) and send byte
	while (!LL_I2C_IsActiveFlag_TXIS(I2C4));
	LL_I2C_TransmitData8(I2C4, OPT4001_REG_SETUP1);

	// Wait for TX ready (TXIS) and send byte
	while (!LL_I2C_IsActiveFlag_TXIS(I2C4));
	LL_I2C_TransmitData8(I2C4, 0x80);

	// Wait for TX ready (TXIS) and send byte
	while (!LL_I2C_IsActiveFlag_TXIS(I2C4));
	LL_I2C_TransmitData8(I2C4, 0x15);

	// Wait for transfer complete and clear STOP flag
	while (!LL_I2C_IsActiveFlag_STOP(I2C4));
	LL_I2C_ClearFlag_STOP(I2C4);

	// ----------------------------------------------------------------
	// Set read pointer for data reads
	LL_I2C_HandleTransfer(I2C4, OPT4001_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	// Wait for TX ready (TXIS) and send byte
	while (!LL_I2C_IsActiveFlag_TXIS(I2C4));
	LL_I2C_TransmitData8(I2C4, OPT4001_REG_DATA);

	// Wait for transfer complete and clear STOP flag
	while (!LL_I2C_IsActiveFlag_STOP(I2C4));
	LL_I2C_ClearFlag_STOP(I2C4);

	return 0;
}
