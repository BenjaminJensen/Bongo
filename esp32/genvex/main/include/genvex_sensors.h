/*
 * genvex_sensors.h
 *
 *  Created on: 30. dec. 2017
 *      Author: Benjamin
 */

#ifndef MAIN_INCLUDE_GENVEX_SENSORS_H_
#define MAIN_INCLUDE_GENVEX_SENSORS_H_

#include "MQTTClient.h"

void init_genvex_sensor(void);

void genvex_wifi_connect(void);
void genvex_wifi_disconnect(void);

int sub(const char* topicFilter, messageHandler handler);

#endif /* MAIN_INCLUDE_GENVEX_SENSORS_H_ */
