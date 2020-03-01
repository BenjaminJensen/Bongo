#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
typedef struct {
    uint16_t temperature;
    uint8_t speed;
    uint8_t set_point;
} motor_data_t;

void motor_init(void);
void motor_set_speed(uint8_t);
uint16_t motor_get_temp(void);
void motor_get_data(motor_data_t*);
#endif