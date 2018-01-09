/*
 * genvex_sensors.c
 *
 *  Created on: 30. dec. 2017
 *      Author: Benjamin
 */
/************************************************
 * Includes
 ***********************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "genvex_sensors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include <esp_log.h>
#include "driver/uart.h"
#include "MQTTClient.h"

/************************************************
 * Defines
 ***********************************************/
#define ECHO_TEST_TXD  (17)
#define ECHO_TEST_RXD  (16)
#define BUF_SIZE (128)

#define NUM_SENSORS 4

// Define the MQTT Broker's hostname, port, username and passphrase. To
// configure these values, run 'make menuconfig'.
#define MQTT_HOST CONFIG_MQTT_BROKER
#define MQTT_PORT CONFIG_MQTT_PORT
#define MQTT_USER CONFIG_MQTT_USER
#define MQTT_PASS CONFIG_MQTT_PASS

/************************************************
 * Locals
 ***********************************************/
typedef struct {

	int32_t id;
	float temperature;
	float pressure;
	float humidity;

} sensor_t;
static sensor_t sensors[NUM_SENSORS] = {0};

static const char* TAG = "Sensor Module";

MQTTClient client;
static TaskHandle_t task = NULL;
Network network;

static unsigned char sendBuf[1000];
static unsigned char readBuf[1000];

enum state_t {
    IDLE = 0,
	ID,
	TEMP,
	PRES,
	HUMI
};

/************************************************
 * Local function declarations
 ***********************************************/
static void process_uart(void *p);
static void parse_string(uint8_t *buf);
static uint8_t get_int(char c, int32_t* value);
static uint8_t get_float(char c, float* value);
static uint8_t is_int(char c);

/************************************************
 * Exported functions
 ***********************************************/
void init_genvex_sensor()
{
	uart_config_t uart_config = {
		.baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};

	//Configure UART1 parameters
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	//Install UART driver (we don't need an event queue here)
	//In this example we don't even use a buffer for sending data.
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0));

	xTaskCreate(process_uart,"SENSOR_TASK", 32*1024, NULL, 10, &task);
}

void genvex_wifi_connect(void)
{
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
	clientId.cstring = "Genvex";

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
}

void genvex_wifi_disconnect(void)
{
	MQTTDisconnect(&client);
}

/************************************************
 * Local functions
 ***********************************************/

static void publish_sensors(uint32_t id)
{
	int rc;
	char topic[128];
	char payload[128];
	MQTTMessage msg;
	int payload_len;

	/** MQTT Publish - send an MQTT publish packet and wait for all acks to complete for all QoSs
	 *  @param client - the client object to use
	 *  @param topic - the topic to publish to
	 *  @param message - the message to send
	 *  @return success code
	 */
	//int MQTTPublish(MQTTClient* client, const char*, MQTTMessage*);

	if (id > (NUM_SENSORS - 1)) {
		ESP_LOGE(TAG, "Wrong sensor ID: %d", id);
		return;
	}

	// Setup message
	msg.qos = 0;
	msg.retained = 0;
	msg.dup = 0;

	// Temperature
	payload_len = sprintf(payload, "%f", sensors[id].temperature);
	msg.payloadlen = payload_len;
	msg.payload = payload;

	sprintf(topic, "genvex/sensor%d/temperature", id);
	rc = MQTTPublish(&client, (const char*)topic, &msg);
	if (rc != SUCCESS) {
		ESP_LOGE(TAG, "Publish t: %d", rc);
	}

	// Pressure
	payload_len = sprintf(payload, "%f", sensors[id].pressure);
	msg.payloadlen = payload_len;
	msg.payload = payload;

	sprintf(topic, "genvex/sensor%d/pressure", id);
	rc = MQTTPublish(&client, (const char*)topic, &msg);
	if (rc != SUCCESS) {
		ESP_LOGE(TAG, "Publish p: %d", rc);
	}

	// Humidity
	payload_len = sprintf(payload, "%f", sensors[id].humidity);
	msg.payloadlen = payload_len;
	msg.payload = payload;

	sprintf(topic, "genvex/sensor%d/humidity", id);
	rc = MQTTPublish(&client, (const char*)topic, &msg);
	if (rc != SUCCESS) {
		ESP_LOGE(TAG, "Publish h: %d", rc);
	}
}

static void process_uart(void *p)
{
	uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

	while(1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_2, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        if(len > 0)
        {
        	data[len] = 0;
			parse_string(data);
			uart_write_bytes(UART_NUM_2, (const char *) data, len);
        }
	}
}

static void parse_string(uint8_t *buf)
{
	static enum state_t state = IDLE;
	static int32_t current_id;
	int index = 0;
	char c;

	// Run through buffer
	while(buf[index] != 0)
	{
		c = buf[index];
		switch(state)
		{
			case ID:
				// get ID
				if((get_int(c, &current_id)))
				{
					state = IDLE;
				}
				else
				{
					index++;
				}
				break;
			case TEMP:
				// get Temperature
				if((get_float(c, &(sensors[current_id].temperature))))
				{
					state = IDLE;
				}
				else
				{
					index++;
				}
				break;
			case PRES:
				// get Pressure
				if((get_float(c, &(sensors[current_id].pressure))))
				{
					state = IDLE;
				}
				else
				{
					index++;
				}
				break;
			case HUMI:
				// get Humidity
				if((get_float(c, &(sensors[current_id].humidity))))
				{
					state = IDLE;
				}
				else
				{
					index++;
				}
				break;
			default:
				if(c == 'i')
				{
					state = ID;
				}
				else if(c == 't')
				{
					state = TEMP;
				}
				else if(c == 'p')
				{
					state = PRES;
				}
				else if(c == 'h')
				{
					state = HUMI;
				}
				else if(c == '\r')
				{
					// Send current values
					//ESP_LOGI(TAG, "ID: %d, t:%f h:%f p:%f", current_id, current_temp, current_humi, current_pres);

					publish_sensors(current_id);
					// reset current values
					current_id = 0;
				}
				index++;
				break;
		}
	}
}

static uint8_t is_int(char c)
{
	return (c >= '0' && c <= '9') ? 1 : 0;
}

static uint8_t get_int(char c, int32_t* value)
{
	if(is_int(c))
	{
		*value = (int32_t)(c - '0') + (*value)*10;
	}
	else
	{
		return 1;
	}
	return 0;
}

static uint8_t get_float(char c, float* value)
{
	static int32_t level = 0;
	float frac = 0;

	if(is_int(c) && level >= 0)
	{
		*value = (float)(c - '0') + (*value)*10*level;
		level++;
	}
	else if(c == '.')
	{
		level = -1;
	}
	else if(is_int(c) && level < 0)
	{
		frac = pow(10.0,-1*level);
		*value += ((float)(c - '0'))/frac;
		level--;
	}
	else
	{
		level = 0;
		return 1;
	}
	return 0;
}
