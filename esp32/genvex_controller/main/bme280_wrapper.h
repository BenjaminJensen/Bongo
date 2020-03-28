#ifndef BME280_WRAPPER_H_
#define BME280_WRAPPER_H_

#include <stdint.h>
#include "BME280_driver/bme280_defs.h"
#include "BME280_driver/bme280.h"

int8_t setup_bme280();
void bme280_task();
void bme280_get_data(struct bme280_data*);

#endif