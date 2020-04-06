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
    uint8_t rotor_rpm_avg;
    uint8_t state;
    uint8_t rotor_fault;
    uint8_t temp_fault;
} rotor_status_t;

typedef struct {
    uint8_t speed;
    /*! Compensated pressure */
    uint32_t pressure;

    /*! Compensated temperature */
    int32_t temperature;

    /*! Compensated humidity */
    uint32_t humidity;
} control_status_t;

int mqtt_update_rotor_status(rotor_status_t*);
int mqtt_update_control_status(control_status_t*);
int mqtt_update_bme280_status(float t, float p, float h);

#endif