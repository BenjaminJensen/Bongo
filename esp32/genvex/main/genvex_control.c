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
#include <esp_system.h>
#include <esp_log.h>

#include "MQTTClient.h"
#include "genvex_control.h"
#include "paho_client.h"

/************************************************
 * Defines
 ***********************************************/
#define INPUT_SPEED_LED0 	GPIO_NUM_15
#define INPUT_SPEED_LED1	GPIO_NUM_4
#define INPUT_SPEED_LED2	GPIO_NUM_18
#define INPUT_LED_ROTOR		GPIO_NUM_19
#define INPUT_LED_FILTER	GPIO_NUM_21
#define GPIO_INPUT_PIN_SEL  ((1ULL<<INPUT_SPEED_LED0) | (1ULL<<INPUT_SPEED_LED1) | (1ULL<<INPUT_SPEED_LED2) | (1ULL<<INPUT_LED_ROTOR) | (1ULL<<GPIO_NUM_21))

#define OUTPUT_SPEED		GPIO_NUM_23
#define OUTPUT_ROTOR		GPIO_NUM_22
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<OUTPUT_SPEED) | (1ULL<<OUTPUT_ROTOR))

// Sample interval in ms
#define SAMPLE_INTERVAL		100

#define MAX_ERRORS_SET_SPEED	3

/************************************************
 * Locals
 ***********************************************/

// handler for FreeRTOS task
static TaskHandle_t sample_handle = NULL;
static TaskHandle_t set_speed_handle = NULL;
static const char* TAG = "Control Module";
static volatile uint8_t cur_speed;

enum state_t {
    UNKNOWN = 0,
	MAYBE_ON,
	ON,
	MAYBE_OFF,
	OFF
};

QueueHandle_t set_speed_queue;

/************************************************
 * Local function declarations
 ***********************************************/
static void sample_task(void* parms);
static void set_speed_task(void *params);
static uint8_t sample_speed(void);
static void handle_set_speed(MessageData* msg);

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
    /*
    gpio_set_direction(INPUT_SPEED_LED0, GPIO_MODE_INPUT);
    gpio_set_direction(INPUT_SPEED_LED1, GPIO_MODE_INPUT);
    gpio_set_direction(INPUT_SPEED_LED2, GPIO_MODE_INPUT);
    gpio_set_direction(INPUT_SPEED_LED0, GPIO_MODE_INPUT);
    gpio_set_direction(INPUT_SPEED_LED0, GPIO_MODE_INPUT);
     */
    // Setup OUTPUT
	gpio_set_direction(OUTPUT_SPEED, GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT_ROTOR, GPIO_MODE_OUTPUT);

	cur_speed = 0;

	// Create queue for speed set values
	set_speed_queue = xQueueCreate( 10, sizeof( uint8_t ) );

	// Create sampler task
	xTaskCreate(sample_task, "SAMPLER_TASK", 8*1024, NULL, 10, &sample_handle);
	xTaskCreate(set_speed_task, "SET_SPEED_TASK", 8*1024, NULL, 8, &set_speed_handle);
}



/************************************************
 * Local functions
 ***********************************************/

static void handle_set_speed(MessageData* msg) {
	static uint8_t speed = 0;
	bool success;
	speed = *((char*)msg->message->payload) - '0';

	ESP_LOGI(TAG, "handle speed: %s", (char*)msg->message->payload);
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

static uint8_t get_next_speed() {
	return cur_speed == 3 ? 0 : cur_speed + 1;
}

static void toggle_relay() {
	gpio_set_level(OUTPUT_SPEED, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	gpio_set_level(OUTPUT_SPEED, 0);
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

static void set_speed_task(void *params) {
	uint8_t new_speed;
	uint8_t next_speed;
	uint8_t steps;
	uint8_t errors;
	/**
	 * speed > cur_speed
	 * 		steps = speed - cur_speed
	 * 	else
	 * 		steps = 4 - cur_speed - speed	 *
	 */

	steps = 0;
	do {
		errors = paho_sub("genvex/speed", &handle_set_speed);
		if(errors != 0) {
			steps++;
			ESP_LOGE(TAG, "Sub: %d", errors);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	} while(errors != 0);

	while(true) {

		if( set_speed_queue != 0 )
		{
			if( xQueueReceive( set_speed_queue, &( new_speed ), 200 / portTICK_PERIOD_MS ) )
			{
				if(new_speed > new_speed) {
					steps = new_speed - cur_speed;
				} else {
					steps = 4 - cur_speed - new_speed;
				}

				errors = 0;
				next_speed = get_next_speed();
				do {
					toggle_relay();
					if(cur_speed == next_speed) {
						next_speed = get_next_speed();
					} else {
						errors++;
					}
				} while(cur_speed != new_speed && errors < MAX_ERRORS_SET_SPEED);

				if(errors > MAX_ERRORS_SET_SPEED) {
					ESP_LOGE(TAG, "Errors during set speed: %d", errors);
				}
				else {
					ESP_LOGI(TAG, "New speed: %d with %d errors", cur_speed, errors);
				}
			}
		}
	}
}

static void sample_task(void* parms) {
	enum state_t state = UNKNOWN;
	uint8_t last_speed = 0;
	uint8_t speed = 0;

    while (true) {
    	// Sample Speed
		speed = sample_speed();

		switch(state) {
			case MAYBE_ON:
				if(speed == last_speed) {
					state = ON;
					cur_speed = speed;

					ESP_LOGI(TAG, "Speed updated: %d", cur_speed);

					// MQTT publish {'topic': 'genvex/speed', 'payload':cur_speed}
				}
				break;
			case ON:
				if(cur_speed != speed) {
					state = MAYBE_ON;
				}
				break;
			default:
				state = MAYBE_ON;
				break;
		}

		last_speed = speed;

        vTaskDelay(SAMPLE_INTERVAL / portTICK_PERIOD_MS);
    }
}

static uint8_t sample_speed() {
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

