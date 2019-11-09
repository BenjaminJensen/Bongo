#ifndef DS18B20_WRAPPER_H_
#define DS18B20_WRAPPER_H_

#include <stdint.h>

uint8_t ds18b20_start_conversion(void);
uint8_t ds18b20_get_data(int16_t *temp);
void ds18b20_task();
void setup_ds18b20();

#endif