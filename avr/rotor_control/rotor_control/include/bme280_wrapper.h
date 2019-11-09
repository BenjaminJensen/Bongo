#ifndef BME280_WRAPPER_H_
#define BME280_WRAPPER_H_

#include <stdint.h>
#include "../bme280_defs.h"
#include "../bme280.h"

int8_t setup_bme280();
void bme280_task();

#endif