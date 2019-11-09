#include "include/com.h"
#include "include/uart.h"
#include "include/timer.h"
#include <avr/interrupt.h>

static struct  {
	int32_t temp;
	uint32_t pres;
	uint32_t humi;
	int16_t mtemp;
} status_data;

static uint8_t timer_handle = 255;

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

void com_task() {
	if(timer_ovf(timer_handle)) {
		PORTD ^= (1 << DDD6);
		send_status();
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
	uint8_t i;
	uint8_t tmp;
	
	// Copy to stack to ensure they are not modified during transmission
	cli();
	temp = status_data.temp;
	pres = status_data.pres;
	humi = status_data.humi;
	mtemp = status_data.mtemp;
	sei();
	
	// Test data
	id = 10;
	speed = 80; // 80%
	setpoint = 90; // 90%
	
	uart_putchar(0xAA);
	uart_putchar(0x00);
	uart_putchar(0xFF);
	
	uart_putchar(id);
	
	// send 8 bytes BME280-temp
	for(i = 0; i < 4; i++) {
		tmp = (uint8_t) ((temp >> i*8)& 0xFF);
		uart_putchar(tmp);
	}
	// send 8 bytes BME280-pres
	for(i = 0; i < 4; i++) {
		tmp = (uint8_t) ((pres >> i*8)& 0xFF);
		uart_putchar(tmp);
	}
	// send 8 bytes BME280-humi
	for(i = 0; i < 4; i++) {
		tmp = (uint8_t) ((humi >> i*8)& 0xFF);
		uart_putchar(tmp);
	}
	
	// Motor temperature
	tmp = (uint8_t)(mtemp & 0xFF);
	uart_putchar(tmp);
	tmp = (uint8_t)((mtemp >> 8) & 0xFF);
	uart_putchar(tmp);
	
	// Current speed
	uart_putchar(speed);
	
	// Current setpoint
	uart_putchar(setpoint);
}
