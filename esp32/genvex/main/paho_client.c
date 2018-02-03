/*
 * paho_client.c
 *
 *  Created on: 24. jan. 2018
 *      Author: Benjamin
 */

#include <stdio.h>

#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "paho_client.h"

MQTTClient client;

static const char* TAG = "mqtt_paho";
static unsigned char sendBuf[1000];
static unsigned char readBuf[1000];

static TaskHandle_t paho_task_handle;

Network network;

static void task_paho(void* params);
static int paho_connect(void);
void paho_wifi_connect(const char* name)
{
	xTaskCreate(&task_paho, "task_paho", 8048, NULL, 5, &paho_task_handle);
}

void paho_wifi_disconnect(void)
{
	vTaskDelete( &paho_task_handle );
	MQTTDisconnect(&client);
}


int paho_sub(const char* topicFilter, messageHandler handler) {
	return MQTTSubscribe(&client, topicFilter, QOS0, handler);
}

int pahu_pub(const char* topic, const char* payload){
	MQTTMessage msg;

	// Setup message
	msg.qos = 0;
	msg.retained = 0;
	msg.dup = 0;
	msg.payload = (void*)payload;
	msg.payloadlen = strlen(payload);
	return MQTTPublish(&client, topic, &msg);
}

static int paho_connect() {
	int rc;
	NetworkInit(&network);
	NetworkConnect(&network, "192.168.1.252", 1883);

	MQTTClientInit(&client, &network,
		1000,            // command_timeout_ms
		sendBuf,         //sendbuf,
		sizeof(sendBuf), //sendbuf_size,
		readBuf,         //readbuf,
		sizeof(readBuf)  //readbuf_size
	);

	MQTTString clientId = MQTTString_initializer;
	clientId.cstring = "hest";

	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	data.clientID          = clientId;
	data.willFlag          = 0;
	data.MQTTVersion       = 3;
	data.keepAliveInterval = 0;
	data.cleansession      = 1;

	rc = MQTTConnect(&client, &data);
	if (rc != SUCCESS) {
		ESP_LOGE(TAG, "MQTTConnect: %d", rc);
	}
	return rc;
}

static void task_paho(void* params) {
	while(1) {
		if(client.isconnected == 1) {
			MQTTYield(&client, 1000);
		} else  {
			paho_connect();
			vTaskDelay(5000 / portTICK_PERIOD_MS);
		}
	}
}
