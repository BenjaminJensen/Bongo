#include "include/ds18b20_wrapper.h"
#include "include/timer.h"
#include "include/com.h"
#include <ds18b20/ds18b20.h>
#include <avr/io.h>

static uint8_t timer_handle = 255;

void setup_ds18b20() {
	timer_handle = timer_register(1000/5);
}

uint8_t ds18b20_start_conversion(void) {
	uint8_t ret = 0;
	//Start conversion (without ROM matching)
	ret = ds18b20convert( &PORTC, &DDRC, &PINC, ( 1 << 2 ), 0 );
	return ret;
}

uint8_t ds18b20_get_data(int16_t *temp) {
	uint8_t ret = 0;
	ret = ds18b20read( &PORTC, &DDRC, &PINC, ( 1 << 2 ), 0, temp );
	return ret;
}

void ds18b20_task() {
	static uint8_t state = 0;
	int16_t temp;
	uint8_t ret = 0;
	if(timer_ovf(timer_handle)) {
		switch(state)
		{
			case 0:
				// 3ms
				ret = ds18b20_start_conversion();
				if(ret != DS18B20_ERROR_OK)
					com_set_ds18b20_data(0x7F00 | ret);
				
				state++;
				break;
			case 1:
				// Get data from conversion
				// 8ms
				ret = ds18b20_get_data(&temp);
				if(ret != DS18B20_ERROR_OK)
					temp = 0x7F00 | ret;
					
				com_set_ds18b20_data(temp);
				state = 2;
				break;
			case 2:
				// Start new conversion
				ret = ds18b20_start_conversion();
				if(ret != DS18B20_ERROR_OK) 
					com_set_ds18b20_data(0x7F00 | ret);
				state = 1;
				break;
			default:
				state = 0;
				break;
		}
	}
}