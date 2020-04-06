/*
 * genvex_emulator.c
 *
 * Created: 02/04/2020 20.17.07
 * Author : ben
 */ 
#define F_CPU (8000000)
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

static void update_output();

int main(void)
{
	// PD 2,3,4 output
	// PD5 input
	DDRD = 0b00011100;
	PORTD = 0b00100000;
	
	PORTD |= 0b00011100;

	uint8_t state = 0;
	
    /* Replace with your application code */
    while (1) 
    {
		
		switch(state) {
			case 0:
				if((PIND & (1 << PIND5)) == 0) {
					state = 1;	
					_delay_ms(40);
				}
				break;
			case 1:
				if((PIND & (1 << PIND5)) ==  0) {
					state = 2;
					update_output();
					_delay_ms(40);
				}
				break;
			case 2:
				if((PIND & (1 << PIND5)) == (1 << PIND5)  ) {
					state = 3;
					_delay_ms(40);
				}
				break;
			case 3:
				if((PIND & (1 << PIND5)) == (1 << PIND5)  ) {
					state = 0;
				}
				break;
			default:
				state = 0;
				break;
		}
		
    }
}

static void update_output() {
	static uint8_t cur = 0;
	
	PORTD &= 0b11100011;
	
	if(cur == 0)
		cur = 1;
	else if(cur == 1)
		cur = 3;
	else if(cur == 3)
		cur = 7;
	else 
		cur = 0;
	PORTD |= (cur & 0x07) << 2;
}
