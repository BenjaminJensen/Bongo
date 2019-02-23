/*
 * genvex_control.c
 *
 *  Created on: 19. jan. 2018
 *      Author: Benjamin
 */
/************************************************
 * Includes
 ***********************************************/
#include <stdio.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_system.h>
#include <esp_log.h>
#include <string.h>

#include "esp_log.h"

#include "genvex_control.h"
#include "mqtt_wrapper.h"


/************************************************
 * Defines
 ***********************************************/
// ###########################################################
// # GPIO 17, 18, 19 ,20 ,21, 22 Reserved for internal FLAHS #
// ###########################################################
#define INPUT_SPEED_LED0 	GPIO_NUM_15
#define INPUT_SPEED_LED1	GPIO_NUM_4
#define INPUT_SPEED_LED2	GPIO_NUM_0
#define GPIO_INPUT_PIN_SEL  ((1ULL<<INPUT_SPEED_LED0) | (1ULL<<INPUT_SPEED_LED1) | (1ULL<<INPUT_SPEED_LED2))

#define OUTPUT_SPEED		GPIO_NUM_22
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<OUTPUT_SPEED) 

// Sample interval in ms
#define SAMPLE_INTERVAL		50

#define MAX_ERRORS_SET_SPEED	3

/************************************************
 * Locals
 ***********************************************/

// handler for FreeRTOS task
static TaskHandle_t sample_handle = NULL;
static const char* TAG = "GENVEX_CONTROL";
static volatile uint8_t cur_speed;

QueueHandle_t set_speed_queue;

/************************************************
 * Local function declarations
 ***********************************************/
static void sample_task(void* parms);
static void set_speed();
static uint8_t get_speed(void);
static bool sample_speed();
static void handle_set_speed(const char* data, int data_len);

/************************************************
 * Exported functions
 ***********************************************/

void init_genvex_control() {
	gpio_config_t io_conf;

	esp_log_level_set(TAG, ESP_LOG_VERBOSE);

	// Setup INPUT
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_INPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

	// Setup OUTPUT
	gpio_set_direction(OUTPUT_SPEED, GPIO_MODE_OUTPUT);

	cur_speed = 0;

	// Create queue for speed set values
	set_speed_queue = xQueueCreate( 10, sizeof( uint8_t ) );

	// Create sampler task
	xTaskCreate(sample_task, "SAMPLER_TASK", 8*1024, NULL, 10, &sample_handle);
}

/************************************************
 * Local functions
 ***********************************************/
void handle_set_speed(const char* data, int data_len) {
	static uint8_t speed = 0;
	bool success;
	uint8_t tmp = *((char*)data) - '0'; // A little dirty

	ESP_LOGI(TAG, "New speed: %d", tmp);
	//ESP_LOGI(TAG, "handle speed: %d", speed);
	// Clamp speed
	if(speed > 3) {
		speed = 3;
	}

	if(speed != cur_speed) {
		success = xQueueSendToBack(set_speed_queue, (const void *)(&speed), 0);
		if(!success) {
			ESP_LOGE(TAG, "Unable to insert new speed into queue");
		}
	}
}
static uint8_t get_next_speed(uint8_t speed) {
	return speed == 3 ? 0 : speed + 1;
}

static void toggle_relay() {
	gpio_set_level(OUTPUT_SPEED, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	gpio_set_level(OUTPUT_SPEED, 0);
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

static void set_speed() {
	uint8_t new_speed = 0;
	uint8_t next_speed;
	//uint8_t steps = 0;
	uint8_t errors;
	uint8_t speed;

	if(set_speed_queue != 0)
	{
		// Check for new set points
		if(xQueueReceive( set_speed_queue, &(new_speed), 0 ))
		{
			/*
			if(new_speed > cur_speed) {
				steps = new_speed - cur_speed;
			} else {
				steps = 4 - cur_speed - new_speed;
			}
			*/
			// If requested speed is equal to current speed, do nothing
			if(new_speed == cur_speed) {
				return;
			}

			errors = 0;
			next_speed = get_next_speed(get_speed());
			do {
				toggle_relay();
				// Update speed
				speed = get_speed();
				// Test if new speed was achieved
				if(speed == next_speed) {
					next_speed = get_next_speed(speed);
				} else {
					errors++;
				}
			} while(speed != new_speed && errors < MAX_ERRORS_SET_SPEED);

			if(errors > MAX_ERRORS_SET_SPEED) {
				ESP_LOGE(TAG, "Errors during set speed: %d", errors);
			}
			else {
				ESP_LOGI(TAG, "New speed: %d with %d errors", speed, errors);
			}
		}
	}
}

static void sample_task(void* parms) {
	const char *speed_topic = "genvex/speed";
	bool subscribed = false;

	while (true) {
		set_speed();

		// Subscribe to set_speed if not done
		if(subscribed == false) {
			subscribed = mqttw_subscribe("genvex/set_speed", 0, &handle_set_speed);
		}

		if(sample_speed() == true) {
			char data = '0';
			bool success;
			data += cur_speed; // this can be done because  speede is between {0-3}
			success = mqttw_publish(speed_topic, (const char*)&data, 0);
			ESP_LOGI(TAG, "Sent publish: %s", success ? "true": "false");
		}
		vTaskDelay(SAMPLE_INTERVAL / portTICK_PERIOD_MS);
	}
}

static bool sample_speed() {
	uint8_t new_speed = 0;
	static uint8_t speed = 0;

	// Sample Speed
	new_speed = get_speed();

	if(speed != new_speed) {
		speed = new_speed;
	}// speed == new_speed AND != cur_speed: Update
	else if(new_speed != cur_speed) {
		// TODO: Update current speed MQTT
		cur_speed = new_speed;
		ESP_LOGI(TAG,"New speed sampled: %d", cur_speed);
		return true;
	}	
	return false;
	// else: No speed change
}

static uint8_t get_speed() {
	uint8_t speed = 0;

	if(gpio_get_level(INPUT_SPEED_LED0) == 0) {
		speed = 1;
	}

	if(gpio_get_level(INPUT_SPEED_LED1) == 0) {
		speed = 2;
	}

	if(gpio_get_level(INPUT_SPEED_LED2) == 0) {
		speed = 3;
	}

	return speed;
}

