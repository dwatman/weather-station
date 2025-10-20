#ifndef INC_AVERAGE_H_
#define INC_AVERAGE_H_

#include <stdint.h>

#define AVG_WINDOW 60

typedef struct {
    float data[AVG_WINDOW];
    uint8_t index;
    uint8_t full;
    float avg;
} avg_60_t;

void AvgInit(avg_60_t *avg);
void AvgAdd(avg_60_t *avg, float n);
void AvgCalc(avg_60_t *avg);

#endif
