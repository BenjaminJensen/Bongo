#ifndef GENVEX_MQTT_H_
#define GENVEX_MQTT_H_
#include <stdint.h>
int mqtt_init(void);

typedef struct {
    float temp;
    float humi;
    float pres;
    float motor_temp;
    uint8_t rotor_rpm;
    uint8_t state;
} rotor_status_t;

int mqtt_update_rotor_status(rotor_status_t*);

#endif