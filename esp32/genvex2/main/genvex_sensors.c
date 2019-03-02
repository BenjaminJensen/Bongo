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
	char cPayload[100];
	char mqttTopic[100];
	int mqttTopicLen;
} sensor_t;
static sensor_t sensors[NUM_SENSORS] = {0};

static const char* TAG = "Sensor Module";

static TaskHandle_t task = NULL;
/*
Network network;

static unsigned char sendBuf[1000];
static unsigned char readBuf[1000];
*/
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


	// Register Publish Handles

	
	// Start UART com task
	xTaskCreate(process_uart,"SENSOR_TASK", 32*1024, NULL, 10, &task);
}

/************************************************
 * Local functions
 ***********************************************/

static void publish_sensors(uint32_t id)
{
	int len;
	if (id > (NUM_SENSORS - 1)) {
		ESP_LOGE(TAG, "Wrong sensor ID: %d", id);
		return;
	}

	// Create JSON for payload
	len = sprintf(sensors[id].cPayload, "{\"temperature\": %f, \"pressure\": %f, \"humidity\": %f}",
			sensors[id].temperature,
			sensors[id].pressure,
			sensors[id].humidity);
	


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

	// TODO: re-write phony logic in function
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
					current_id = 0;
				}
				else if(c == 't')
				{
					state = TEMP;
					sensors[current_id].temperature = 0;
				}
				else if(c == 'p')
				{
					state = PRES;
					sensors[current_id].pressure = 0;
				}
				else if(c == 'h')
				{
					state = HUMI;
					sensors[current_id].humidity = 0;
				}
				else if(c == '\r')
				{
					// Send current values
					ESP_LOGI(TAG, "ID: %d, t:%f h:%f p:%f",
							current_id,
							sensors[current_id].temperature,
							sensors[current_id].humidity,
							sensors[current_id].pressure);

					//TODO: Move this function OUT!
					publish_sensors(current_id);
					// reset current values
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
		*value = (int32_t)(c - '0') + (*value)*10;
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
