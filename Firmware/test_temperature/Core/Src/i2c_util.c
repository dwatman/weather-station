#include "i2c_util.h"

void i2c_init(i2c_t *h, I2C_TypeDef *device) {
	h->device = device;
	h->state = I2C_STATE_IDLE;
	h->length = 0;
	h->done = 0;
	h->index = 0;

	// Clear all flags
	LL_I2C_ClearFlag_ARLO(device);
	LL_I2C_ClearFlag_BERR(device);
	LL_I2C_ClearFlag_NACK(device);
	LL_I2C_ClearFlag_OVR(device);
	LL_I2C_ClearFlag_STOP(device);
	LL_I2C_ClearFlag_TXE(device);

	// Enable used interrupts
	LL_I2C_EnableIT_ERR(device);
	LL_I2C_EnableIT_NACK(device);
	LL_I2C_EnableIT_RX(device);
	LL_I2C_EnableIT_STOP(device);
	LL_I2C_EnableIT_TC(device);
	//LL_I2C_EnableIT_TX(device);
}
