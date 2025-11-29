#ifndef APPLICATION_USER_INCLUDE_USART_UTIL_H_
#define APPLICATION_USER_INCLUDE_USART_UTIL_H_

#include <stdint.h>
#include "circbuf.h"

void initUsart1(void);
void initUsart4(void);

uint32_t printfCircBuf(CircularBuffer_t *buf, const char *format, ...);

void USART1_ISR(void);
void UART4_ISR(void);


#endif
