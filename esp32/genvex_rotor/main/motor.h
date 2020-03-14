#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
typedef struct {
    uint16_t temperature;
    uint8_t speed;
    uint8_t set_point;
    float rpm0;
    float rpm0_avg;
    float rpm1;
    float rpm1_avg;
} motor_data_t;

void motor_init(void);
void motor_set_speed(uint8_t);
void motor_get_data(motor_data_t*);
#endif