/*
 * rotor_control.c
 *
 * Created: 05/10/2019 16.35.00
 * Author : ben
 */ 

#include <avr/io.h>
#include "include/uart.h"
#include<avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#include "avr-ds18b20/include/ds18b20/ds18b20.h"

#define TIMER0_OVF_NUM 255

volatile uint8_t subtime;

ISR (TIMER0_OVF_vect)    // Timer1 ISR
{
	PORTB ^= (1 << DDB0);
	
	if(subtime <= 255)
		subtime++;
		
}

/*
	Set rotor speed 0-100%
*/
void set_speed(uint8_t speed) {
	
}

void setup_timer()
{
	/* 
		freq = F_CPU / (2 x prescaler x (1 + OCR0A) 
		1000 = 16e10 / (2 x 64 x(1 + 124) )
	*/
	
	// Max counter value
	OCR0A = TIMER0_OVF_NUM;
	// CTC mode
	TCCR0A = (1 << WGM01);
	// Set prescaler
	TCCR0B = (1<<CS00) | (1<<CS01);;  // Timer mode with 64 prescler
	TIMSK0 = (1 << TOIE0) ;   // Enable timer1 overflow interrupt(TOIE1)
	// Enable global interrupts by setting global interrupt enable bit in SREG
}


void setup_pwm()
{
	// set PWM for 50% duty cycle at 10bit
	OCR1A = 0x01FF;
	
	// set non-inverting mode
	TCCR1A |= (1 << COM1A1);
	
	// set 10bit phase corrected PWM Mode
	TCCR1A |= (1 << WGM11) | (1 << WGM10);
	
	// set prescaler to 8 and starts PWM
	TCCR1B |= (1 << CS11);
}

void send_status(int16_t mtemp) {
	uint8_t id; // 0
	int32_t temp; // 1
	uint32_t pres; // 5
	uint32_t humi; // 9
	//int16_t mtemt; // 13
	uint8_t speed; // 15
	uint8_t setpoint; // 16
	
	uint8_t i;
	uint8_t tmp;
	
	// Test data
	id = 10;
	speed = 80; // 80%
	setpoint = 90; // 90%
	//mtemt = 0x0208; // DS18b20 (32.5C)
	
	/*
		BME280
		- int32_t for temperature with the units 100 * °C
		- uint32_t for humidity with the units 1024 * % relative humidity
		- uint32_t for pressure
		If macro "BME280_64BIT_ENABLE" is enabled, which it is by default, the unit is 100 * Pascal
		If this macro is disabled, Then the unit is in Pascal
	*/
	temp = 2380; // 23.8C*100 
	pres = 1119; // pascal
	humi = 54784; // 53,5% * 1024 = 54.784
	
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

/*
* Samples DS18B20 on port D pin7
*/
int16_t sample_mtemt() {
	int16_t temp;
	//Start conversion (without ROM matching)
	ds18b20convert( &PORTD, &DDRD, &PIND, ( 1 << 7 ), 0 );

	//Delay (sensor needs time to perform conversion)
	_delay_ms( 1000 );

	//Read temperature (without ROM matching)
	ds18b20read( &PORTD, &DDRD, &PIND, ( 1 << 7 ), 0, &temp );
	return temp;
}

void test_motor() {
	
	uint8_t dir = 0;
	uint16_t ocr = 0;
	
	if(subtime >= 3) {
		cli();
		subtime = 0;
		sei();
		
		if(dir == 0) {
			if(OCR1A == 1)
			dir = 1;
			else
			{
				cli();
				ocr = (OCR1A >> 1) & 0x01FF;
				OCR1A -=  1;
				sei();
			}
		}
		
		if(dir == 1) {
			if(OCR1A == 0x03FF)
			dir = 0;
			else
			{
				cli();
				ocr = (OCR1A << 1) | 1;
				OCR1A +=  1;
				sei();
			}
		}
	}
}
int main(void)
{
	// PB0 and PB1 as output
	DDRB = (1 << DDB1) | (1 << DDB0);
	subtime = 0;
	
	setup_timer();
	setup_pwm();
	uart_init();
	   
	sei();
	int16_t t;
	
	/*
	uart_putchar((uint8_t) t);
	uart_putchar((uint8_t)(t >> 8));
	*/
    while (1) 
    {
		t = sample_mtemt();
		send_status(t);
    }
}

