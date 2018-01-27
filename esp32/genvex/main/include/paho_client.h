/*
 * paho_client.h
 *
 *  Created on: 24. jan. 2018
 *      Author: Benjamin
 */

#ifndef MAIN_INCLUDE_PAHO_CLIENT_H_
#define MAIN_INCLUDE_PAHO_CLIENT_H_

#include "MQTTClient.h"

void paho_wifi_connect(const char* name);
void paho_wifi_disconnect(void);

int paho_sub(const char* topicFilter, messageHandler);
int pahu_pub(const char* topic, const char* payload);
#endif /* MAIN_INCLUDE_PAHO_CLIENT_H_ */
