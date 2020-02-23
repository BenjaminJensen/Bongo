/*
 * rotor_control.c
 *
 * Created: 05/10/2019 16.35.00
 * Author : ben
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#include "include/com.h"
#include "include/bme280_wrapper.h"
#include "include/timer.h"
#include "include/ds18b20_wrapper.h"

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

/*
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
*/

/*
* Main
*/
int main(void)
{
	// PB1 as output
	DDRB = (1 << DDB1);
	//DDRD = (1 << DDD6);
	
	setup_timer();
	setup_pwm();
	com_init();
	setup_bme280();
	setup_ds18b20();
	
	sei();
	
    while (1) 
    {
		ds18b20_task();				
		bme280_task();
		com_task();
    }
}

