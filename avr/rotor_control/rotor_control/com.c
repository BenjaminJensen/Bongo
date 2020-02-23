#include "include/com.h"
#include "include/uart.h"
#include "include/timer.h"
#include <avr/interrupt.h>
#include <util/delay.h>

#define SLAVE_ID 10

static struct  {
	int32_t temp;
	uint32_t pres;
	uint32_t humi;
	int16_t mtemp;
} status_data;

static uint8_t timer_handle = 255;
static void com_eval_cmd(uint8_t cmd, uint8_t data);

void com_init() {
	uart_init();
	timer_handle = timer_register(1000/5);
	uart_putchar(timer_handle);
}

void com_set_bme280_data(int32_t temp, uint32_t pres, uint32_t humi) {
	cli();
	status_data.temp = temp;
	status_data.pres = pres;
	status_data.humi = humi;
	sei();
}

void com_set_ds18b20_data(int16_t temp) {
	cli();
	status_data.mtemp = temp;
	sei();
}
/*
* Protocol format
* {0xAA, 0x00, 0xFF, slave_id(4) + command(4), data(8) 
*
*
*/
void com_task() {
	static uint8_t state = 0;
	static uint8_t cmd = 0xFF;
	static uint8_t slave_id = 0xFF;
	
	if(uart_char_waiting() == 1) {
		uint8_t c = uart_getchar();
		
		switch(state) {
			case 0:
				if(c == 0xAA)
					state = 1;
				break;
			case 1:
				if(c == 0x00)
					state = 2;
				else
					state = 0;
				break;
			case 2:
				if(c == 0xFF)
					state = 3;
				else
					state = 0;
				break;
			case 3:
				// Getting slave_id and cmd
				cmd = c & 0x0F;
				slave_id = (c >> 4) & 0x0F;
				state = 4;
				break;
			case 4:
				// Getting cmd data
				if(slave_id == SLAVE_ID) {
					com_eval_cmd(cmd, c);
				}
				state = 0;
				break;
			default:
				state = 0;
				break;
		}
	}
	/*
	if(timer_ovf(timer_handle)) {
		PORTD ^= (1 << DDD6);
		send_status();
	}
	*/
}

static void com_eval_cmd(uint8_t cmd, uint8_t data) {
	switch(cmd) {
		case 0:
			_delay_us(5);
			send_status();
			break;
		default:
			break;
	}
}

void send_status() {
	uint8_t id; // 0
	uint8_t speed; // 15
	uint8_t setpoint; // 16
	int32_t temp;
	uint32_t pres;
	uint32_t humi;
	int16_t mtemp;
	uint8_t i = 0; // Transmission buffer index
	uint8_t k;
	uint8_t tmp;
	uint8_t buf[21];
	
	temp = status_data.temp;
	pres = status_data.pres;
	humi = status_data.humi;
	mtemp = status_data.mtemp;
	
	// Test data
	id = 10;
	speed = 80; // 80%
	setpoint = 90; // 90%
	
	buf[0] = 0xAA;
	buf[1] = 0x00;
	buf[2] = 0xFF;
	
	buf[3] = id;
	i = 4;
	
	// send 4 bytes BME280-temp
	for(k = 0; k < 4; k++) {
		tmp = (uint8_t) ((temp >> i*8)& 0xFF);
		buf[i++] = tmp;
	}
	// send 4 bytes BME280-pres
	for(k = 0; k < 4; k++) {
		tmp = (uint8_t) ((pres >> i*8)& 0xFF);
		buf[i++] = tmp;
	}
	// send 4 bytes BME280-humi
	for(k = 0; k < 4; k++) {
		tmp = (uint8_t) ((humi >> i*8)& 0xFF);
		buf[i++] = tmp;
	}
	
	// Motor temperature
	tmp = (uint8_t)(mtemp & 0xFF);
	buf[i++] = tmp;
	tmp = (uint8_t)((mtemp >> 8) & 0xFF);
	buf[i++] = tmp;
	
	// Current speed
	buf[i++] = speed;
	
	// Current setpoint
	buf[i++] = setpoint;
	
	for(i = 0; i < 20;i++) {
		uart_putchar(buf[i]);
	}
}
