#include "sensor_data.h"

// 32-entry sine lookup table, amplitude 100, offset 0.
// Values represent sin(i * 2*pi/32) * 100, rounded to int.
static const int16_t sine_lut[32] = {
	  0,  19,  38,  56,  71,  83,  92,  98,
	100,  98,  92,  83,  71,  56,  38,  19,
	  0, -19, -38, -56, -71, -83, -92, -98,
	-100, -98, -92, -83, -71, -56, -38, -19,
};

static uint32_t call_count = 0;

// Returns a sine value with given amplitude, phase offset, and period divisor.
// period_div controls how fast the wave cycles (higher = slower).
static int16_t sine_wave(int16_t amplitude, uint32_t phase, uint32_t period_div) {
	uint32_t idx = ((call_count / period_div) + phase) % 32;
	return (int16_t)((sine_lut[idx] * amplitude) / 100);
}

int16_t sensor_get_temperature(void) {
	call_count++;
	return (int16_t)(2250 + sine_wave(300, 0, 1));  // 19.5 - 25.5 C
}

uint16_t sensor_get_humidity(void) {
	return (uint16_t)(4500 + sine_wave(500, 8, 1));  // 40 - 50 %RH
}

uint16_t sensor_get_pressure(void) {
	return (uint16_t)(1013 + sine_wave(10, 4, 2));  // 1003 - 1023 hPa
}

uint16_t sensor_get_illuminance(void) {
	return (uint16_t)(300 + sine_wave(200, 12, 1));  // 100 - 500 lux
}

uint16_t sensor_get_co2(void) {
	return (uint16_t)(600 + sine_wave(150, 6, 2));  // 450 - 750 ppm
}

uint16_t sensor_get_co(void) {
	return (uint16_t)(2 + sine_wave(1, 16, 3));  // 1 - 3 ppm
}

uint16_t sensor_get_voc_index(void) {
	return (uint16_t)(100 + sine_wave(40, 20, 2));  // 60 - 140
}

uint16_t sensor_get_nox_index(void) {
	return (uint16_t)(50 + sine_wave(20, 24, 3));  // 30 - 70
}

uint16_t sensor_get_pm1(void) {
	return (uint16_t)(8 + sine_wave(4, 2, 2));  // 4 - 12 ug/m3
}

uint16_t sensor_get_pm25(void) {
	return (uint16_t)(12 + sine_wave(6, 4, 2));  // 6 - 18 ug/m3
}

uint16_t sensor_get_pm4(void) {
	return (uint16_t)(15 + sine_wave(7, 6, 2));  // 8 - 22 ug/m3
}

uint16_t sensor_get_pm10(void) {
	return (uint16_t)(20 + sine_wave(10, 8, 2));  // 10 - 30 ug/m3
}
