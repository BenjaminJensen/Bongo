/*
 * asw_client.h
 *
 *  Created on: 14. feb. 2018
 *      Author: Benjamin
 */

#ifndef MAIN_INCLUDE_AWS_CLIENT_H_
#define MAIN_INCLUDE_AWS_CLIENT_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include "aws_iot_mqtt_client_interface.h"

void init_aws_client(EventGroupHandle_t *weg, EventBits_t bit);

IoT_Error_t aws_client_sub(const char *topic, pApplicationHandler_t handler, QoS q);
#endif /* MAIN_INCLUDE_AWS_CLIENT_H_ */
