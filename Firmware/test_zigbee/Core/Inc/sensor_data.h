#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

// Placeholder sensor data functions.
// Each returns a slowly-varying synthetic value in ZCL-native units.
// Replace with real sensor drivers when hardware is available.

int16_t  sensor_get_temperature(void);   // 0.01 degC  (midpoint ~2250 = 22.50 C)
uint16_t sensor_get_humidity(void);      // 0.01 %RH   (midpoint ~4500 = 45.00%)
uint16_t sensor_get_pressure(void);      // hPa        (midpoint ~1013)
uint16_t sensor_get_illuminance(void);   // lux        (midpoint ~300)
uint16_t sensor_get_co2(void);           // ppm        (midpoint ~600)
uint16_t sensor_get_co(void);            // ppm        (midpoint ~2)
uint16_t sensor_get_voc_index(void);     // index 1-500 (midpoint ~100)
uint16_t sensor_get_nox_index(void);     // index 1-500 (midpoint ~50)
uint16_t sensor_get_pm1(void);           // ug/m3      (midpoint ~8)
uint16_t sensor_get_pm25(void);          // ug/m3      (midpoint ~12)
uint16_t sensor_get_pm4(void);           // ug/m3      (midpoint ~15)
uint16_t sensor_get_pm10(void);          // ug/m3      (midpoint ~20)
uint16_t sensor_get_sound(void);         // 0.01 dB    (midpoint ~4500 = 45.00 dB)

#endif // SENSOR_DATA_H
