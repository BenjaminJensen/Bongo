/*
Wrapper for MQTT clients

*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <freertos/FreeRTOS.h>

typedef void (*mqttw_sub_handle_t)(const char* data, int data_len);
bool mqttw_publish(const char* topic, const char* data, int qos);
bool mqttw_subscribe(const char* topic, int qos, mqttw_sub_handle_t handle);
void mqttw_init(void*);
void mqttw_handle_data(int topic_len, const char* topic, int data_len, const char* data);