#ifndef APPLICATION_USER_CORE_OPT4001_H_
#define APPLICATION_USER_CORE_OPT4001_H_

#define OPT4001_I2C_ADDR (0x44 << 1)

#define OPT4001_REG_DATA    0x00
#define OPT4001_REG_FIFO0   0x02
#define OPT4001_REG_FIFO1   0x04
#define OPT4001_REG_FIFO2   0x06
#define OPT4001_REG_THRESHL 0x08
#define OPT4001_REG_THRESHH 0x09
#define OPT4001_REG_SETUP0  0x0A
#define OPT4001_REG_SETUP1  0x0B
#define OPT4001_REG_FLAGS   0x0C
#define OPT4001_REG_ID      0x11

#define OPT4001_OK           0
#define OPT4001_ERR_NACK    -1
#define OPT4001_ERR_BUS     -2

int opt4001_init(void);
void opt4001_StartRead(void);
void opt4001_ISR(void);

#endif
