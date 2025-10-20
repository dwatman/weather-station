#include "average.h"

void AvgInit(avg_60_t *avg) {
	avg->index = 0;
	avg->avg = 0;
	avg->full = 0;

	for (int i=0; i<60; i++)
		avg->data[i] = 0;
}

void AvgAdd(avg_60_t *avg, float n) {
	avg->data[avg->index] = n;
	avg->index++;

	if (avg->index >= AVG_WINDOW) {
		avg->index = 0;
		avg->full = 1;
	}
}

void AvgCalc(avg_60_t *avg) {
	float sum = 0.0f;

	for (uint8_t i=0; i<AVG_WINDOW; i++)
		sum += avg->data[i];

	avg->avg = sum/(float)AVG_WINDOW;
	avg->full = 0;
}
