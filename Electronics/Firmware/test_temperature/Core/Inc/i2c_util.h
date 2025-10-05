#ifndef INC_I2C_UTIL_H_
#define INC_I2C_UTIL_H_

#include <stdint.h>

#include "stm32wbaxx_ll_i2c.h"

typedef enum {
	// Send command for read
    I2C_STATE_IDLE = 0,
	I2C_STATE_WRITING = 1,
	I2C_STATE_READING = 2,

	I2C_STATE_ERROR = 3,
} I2C_STATE;

// Structure to hold info for one I2C channel
typedef struct {
	I2C_TypeDef *i2c;  // E.g. I2C1

    volatile I2C_STATE state;  // Current transaction state
    volatile uint8_t timeout;  // Counter decremented in ISR, used for delays (ms)
    volatile uint8_t done;     // Transaction complete
    volatile uint8_t err;      // 0 = no error
    volatile uint8_t index;    // Buffer index when writing or reading multiple bytes
    volatile uint8_t rxbuf[6]; // Receive buffer
} i2c_t;

void i2c_init(i2c_t *h, I2C_TypeDef *i2c);


#endif
