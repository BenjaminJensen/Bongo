#ifndef SENSOR_HELPER_H_
#define SENSOR_HELPER_H_
#include <stdint.h>
float convert_bme280_t_raw_to_float(int32_t temp);

/**
 * Convert raw humidity from BME280 to float
 * See:
 */
float convert_bme280_h_raw_to_float(uint32_t humidity);

/**
 * Convert raw pressure from BME280 to float
 * See:
 */
float convert_bme280_p_raw_to_float(uint32_t pressure);

/**
 * Convert raw data from a DS18b20 temperature sensor to float
 */
float convert_ds18b20(int16_t raw);

#endif