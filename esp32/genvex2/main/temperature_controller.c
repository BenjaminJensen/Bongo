/*
 * temperature_controller.c
 *
 *  Created on: 3. mar. 2018
 *      Author: Benjamin
 */

/************************************************
 * Includes
 ***********************************************/
#include "temperature_controller.h"

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "mqtt_wrapper.h"

/************************************************
 * Defines
 ***********************************************/ 
#define GPIO_DS18B20_0       (5)
#define MAX_DEVICES          (1)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD        (10000)   // milliseconds

// Use this to discover devices
//#define DISCOVERY_MODE
#define NUM_KNOWN_DEVICES	1
/************************************************
 * Locals
 ***********************************************/
static TaskHandle_t temp_handle = NULL;
static const char* TAG = "TempCtrl";

uint64_t known_device[NUM_KNOWN_DEVICES] = {0x28ff983763174ad};

/************************************************
 * Local function declarations
 ***********************************************/
static void temp_task(void* parms);

/************************************************
 * Exported functions
 ***********************************************/

void init_temperature_controller(void) {
	esp_log_level_set(TAG, ESP_LOG_VERBOSE);

	xTaskCreate(temp_task, "temp_TASK", 8*1024, NULL, 10, &temp_handle);
}

/************************************************
 * Local functions
 ***********************************************/
static void temp_task(void* parms) {
	int num_devices = 0;
	OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
	
	char data[128];
	int data_len;
	const char *TOPIC = "genvex/temp";

	// Stable readings require a brief period before communication
	vTaskDelay(2000.0 / portTICK_PERIOD_MS);

	// Setup One Wire Bus
	OneWireBus * owb;
	owb_rmt_driver_info rmt_driver_info;
	owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
	owb_use_crc(owb, true);              // enable CRC check for ROM code

#ifdef DISCOVERY_MODE
    // Find all connected devices
    printf("Find devices:\n");
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    printf("Found %d devices\n", num_devices);
#else
    num_devices = NUM_KNOWN_DEVICES;
    for(int k = 0; k < NUM_KNOWN_DEVICES; k++) {
    	uint8_t* p = (uint8_t*)&known_device[k];
		for(int b = 0; b < 8; b++) {
			device_rom_codes[k].bytes[b] = p[b];
		}
    	printf("Rom code copied: %x%x%x%x%x%x%x%x\n",
    			device_rom_codes[k].bytes[0]
				,device_rom_codes[k].bytes[1]
				,device_rom_codes[k].bytes[2]
				,device_rom_codes[k].bytes[3]
				,device_rom_codes[k].bytes[4]
				,device_rom_codes[k].bytes[5]
				,device_rom_codes[k].bytes[6]
				,device_rom_codes[k].bytes[7]
										   );
    }
#endif

    // Create a DS18B20 device on the 1-Wire bus
	DS18B20_Info devices_static[MAX_DEVICES] = {0};
	DS18B20_Info * devices[MAX_DEVICES] = {0};
	for (int i = 0; i < MAX_DEVICES; ++i)
	{
		devices[i] = &(devices_static[i]);
	}

	for (int i = 0; i < num_devices; ++i)
	{
		DS18B20_Info * ds18b20_info = devices[i];

		if (num_devices == 1)
		{
			printf("Single device optimisations enabled\n");
			ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
		}
		else
		{
			ds18b20_init(ds18b20_info, owb, device_rom_codes[i]); // associate with bus and device
		}
		ds18b20_use_crc(ds18b20_info, true);           // enable CRC check for temperature readings
		ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
	}

	// Read temperatures more efficiently by starting conversions on all devices at the same time
	int errors_count[MAX_DEVICES] = {0};
	int sample_count = 0;
	if (num_devices > 0)
	{
		TickType_t last_wake_time = xTaskGetTickCount();
		float last_temp = 0;
		while (1)
		{
			last_wake_time = xTaskGetTickCount();

			ds18b20_convert_all(owb);

			// In this application all devices use the same resolution,
			// so use the first device to determine the delay
			ds18b20_wait_for_conversion(devices[0]);

			// Read the results immediately after conversion otherwise it may fail
			// (using printf before reading may take too long)
			float readings[MAX_DEVICES] = { 0 };
			DS18B20_ERROR errors[MAX_DEVICES] = { 0 };

			for (int i = 0; i < num_devices; ++i)
			{
				errors[i] = ds18b20_read_temp(devices[i], &readings[i]);
			}

			for (int i = 0; i < num_devices; ++i)
			{
				if(readings[i] < (last_temp - 0.3 ) || readings[i] > (last_temp + 0.3 ) || sample_count > 20)
				{
					sample_count = 0;
					last_temp = readings[i];
					data_len = sprintf(data, "%.1f", readings[i]);

					// MQTT publish
					ESP_LOGI(TAG, "Temp: %.*s C",data_len, data);

					mqttw_publish(TOPIC, data, 0);
				}
				else {
					sample_count++;
				}
				
			}
			/*
			// Print results in a separate loop, after all have been read
			printf("\nTemperature readings (degrees C): sample %d\n", ++sample_count);
			for (int i = 0; i < num_devices; ++i)
			{
				if (errors[i] != DS18B20_OK)
				{
					++errors_count[i];
				}

				printf("  %d: %.1f    %d errors\n", i, readings[i], errors_count[i]);
			}
			*/

			vTaskDelayUntil( &last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS );
			//vTaskDelay(SAMPLE_PERIOD / portTICK_PERIOD_MS);
		}
	}

}
