#ifndef APPLICATION_USER_INCLUDE_LPS25HB_H_
#define APPLICATION_USER_INCLUDE_LPS25HB_H_

#define LPS25HB_I2C_ADDR (0x5D << 1)

#define LPS25HB_REG_RES_CONF      0x10
#define LPS25HB_REG_CTRL_REG1     0x20
#define LPS25HB_REG_CTRL_REG2     0x21
#define LPS25HB_REG_CTRL_REG3     0x22
#define LPS25HB_REG_CTRL_REG4     0x23
#define LPS25HB_REG_INTERRUPT_CFG 0x24
#define LPS25HB_REG_PRESS_OUT_XL  0x28
#define LPS25HB_REG_PRESS_OUT_L   0x29
#define LPS25HB_REG_PRESS_OUT_H   0x2A
#define LPS25HB_REG_FIFO_CTRL     0x2E

int lps25hb_init(void);
void lps25hb_StartRead(I2c_Status_t *i2c);
float lps25hb_Convert(void);

#endif
