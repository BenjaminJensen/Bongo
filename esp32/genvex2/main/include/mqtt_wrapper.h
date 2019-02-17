/*
Wrapper for MQTT clients

*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <freertos/FreeRTOS.h>

typedef void (*mqttw_sub_handle)(const char* data);
bool mqttw_publish(const char* topic, const char* data, int qos);
bool mqttw_subscribe(const char* topic, int qos, mqttw_sub_handle handle);
void mqttw_init(void*);