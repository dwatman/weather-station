#ifndef INC_I2C_UTIL_H_
#define INC_I2C_UTIL_H_

#include <stdint.h>

#include "stm32wbaxx_ll_i2c.h"

typedef enum {
	// Send command for read
    I2C_STATE_IDLE = 0,
	I2C_STATE_SENT_START1 = 1,
	I2C_STATE_SENT_ADDRESS_WRITE = 2,
	I2C_STATE_SENT_COMMAND = 3,
	I2C_STATE_SENT_COMMAND_STOP = 4,

	// Read 3 data bytes
	I2C_STATE_SENT_START2 = 5,
	I2C_STATE_SENT_ADDRESS_READ = 6,
	I2C_STATE_RECV_DATA0 = 7,
	I2C_STATE_RECV_DATA1 = 8,
	I2C_STATE_RECV_DATA2 = 9,
	I2C_STATE_SENT_STOP = 10,

	I2C_STATE_ERROR = 255,
} I2C_STATE;

// Structure to hold info for one I2C channel
typedef struct {
	I2C_TypeDef *i2c;  // E.g. I2C1
	uint8_t address;   // Slave address to communicate with (7-bit in LSBs)
	uint8_t command;   // Command for slave

    volatile I2C_STATE state;  // Current transaction state
    volatile uint8_t done;      // Transaction complete
    volatile uint8_t err;      // 0 = no error
    volatile uint8_t rxbuf[3]; // Receive buffer
} i2c_t;

void i2c_init(i2c_t *h, I2C_TypeDef *i2c);



#endif
