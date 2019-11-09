#ifndef COM_H_
#define COM_H_

#include <stdint.h>

void com_init(void);
void send_status();
void com_set_bme280_data(int32_t temp, uint32_t pres, uint32_t humi);
void com_set_ds18b20_data(int16_t temp);
void com_task();

#endif