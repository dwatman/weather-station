#ifndef APPLICATION_USER_INCLUDE_I2C_UTIL_H_
#define APPLICATION_USER_INCLUDE_I2C_UTIL_H_

#define I2C_BUFSZ 4

#define I2C_OK           0
#define I2C_ERR_NACK    -1
#define I2C_ERR_BUS     -2

#define I2C_DEVICE_OPT4001 0
#define I2C_DEVICE_LPS25HB 1

#define I2C_STATE_IDLE 0
#define I2C_STATE_BUSY 1

#define I2C_PHASE_WRITE_REG 0
#define I2C_PHASE_READ_DATA 1

typedef struct {
	volatile uint8_t state;
	volatile uint8_t device;
	volatile uint8_t phase;
	volatile uint8_t index;
	volatile uint8_t *rxDst;
} I2c_Status_t;

int SendBytes(uint8_t addr, const uint8_t *data, uint8_t len);
void i2c_ISR(I2c_Status_t *i2c);

#endif
