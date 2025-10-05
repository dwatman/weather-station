#include "i2c_util.h"

void i2c_init(i2c_t *h, I2C_TypeDef *i2c) {
	h->i2c = i2c;
	h->state = I2C_STATE_IDLE;
	h->timeout = 0;
	h->done = 0;
	h->err = 0;
	h->index = 0;

	// Clear all flags
	LL_I2C_ClearFlag_ARLO(i2c);
	LL_I2C_ClearFlag_BERR(i2c);
	LL_I2C_ClearFlag_NACK(i2c);
	LL_I2C_ClearFlag_OVR(i2c);
	LL_I2C_ClearFlag_STOP(i2c);
	LL_I2C_ClearFlag_TXE(i2c);

	// Enable used interrupts
	LL_I2C_EnableIT_ERR(i2c);
	LL_I2C_EnableIT_NACK(i2c);
	LL_I2C_EnableIT_RX(i2c);
	LL_I2C_EnableIT_STOP(i2c);
	LL_I2C_EnableIT_TC(i2c);
	//LL_I2C_EnableIT_TX(i2c);
}

//int i2c_send
//LL_I2C_EnableAutoClearFlag_ADDR
//LL_I2C_EnableAutoClearFlag_STOP
//LL_I2C_EnableAutoEndMode
//LL_I2C_IsActiveFlag_BUSY
