#include "sensor_helper.h"

/**
 * Convert raw temperature from BME280 to float
 * See:
 */
float convert_bme280_t_raw_to_float(int32_t temp) {
    float val = 0;
    val = (float)temp / 100.0f;
    return val;
}

/**
 * Convert raw humidity from BME280 to float
 * See:
 */
float convert_bme280_h_raw_to_float(uint32_t humidity) {
    float val = 0;
    val = (float)humidity / 1024.0f;
    return val;
}

/**
 * Convert raw pressure from BME280 to float
 * See:
 */
float convert_bme280_p_raw_to_float(uint32_t pressure) {
    float val = 0;
    val = (float)pressure / 1000.0f; // Pa
    val /= 10; // Convert into hPa
    return val;
}

/**
 * Convert raw data from a DS18b20 temperature sensor to float
 */
float convert_ds18b20(int16_t raw) {
    float frac = 0.0f;
    
    if(raw & 0x08)
        frac += 0.5;
    if(raw & 0x04)
        frac += 0.25;
    if(raw & 0x02)
        frac += 0.125;
    if(raw & 0x01)
        frac += 0.065;

    return (raw >> 4) + frac;
}