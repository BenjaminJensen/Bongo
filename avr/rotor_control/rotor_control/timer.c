#include "include/timer.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define TIMER0_OVF_NUM 124
#define MAX_TIMERS 3

volatile uint8_t subtime;


typedef struct {
	uint8_t ovf;
	uint8_t timer;
	uint8_t flags;
} timer_t;

timer_t timers[MAX_TIMERS];

volatile uint8_t num_timers = 255;

/*
*
* Setup the timer for 1ms
*
*/
void setup_timer() {
	
	DDRB = (1 << DDB0);
	
	/* 
		freq = F_CPU / (2 x prescaler x (1 + OCR0A) 
		1000 = 8e10 / (2 x 64 x(1 + 124) )
		
		200.321=8e10 / (2 x 256 x(1 + 77) )
	*/
	
	// Max counter value
	OCR0A = 77;
	// CTC mode
	TCCR0A = (1 << WGM01);
	// Set prescaler
	TCCR0B = (1<<CS02);  // Timer mode with 256 prescler
	TIMSK0 = (1 << OCIE0A) ;   // Enable timer compare on OCRA
}

uint8_t timer_register(uint8_t time) {
	uint8_t timer = 255;
	if(num_timers == 255)
		num_timers = 0;
		
	if(num_timers < MAX_TIMERS) {
		timer = num_timers;
		num_timers++;
		timers[timer].ovf = time;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			timers[timer].timer = 0;
		}
	}
	return timer;
}
uint8_t timer_ovf(uint8_t timer) {
	uint8_t ret = 0;
	
	if(timers[timer].timer >= timers[timer].ovf) {
		ret = 1;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			timers[timer].timer = 0;
		}		
	}
	return ret;
}

ISR (TIMER0_COMPA_vect)    // Timer1 ISR
{
	PORTB ^= (1 << DDB0);
	timers[0].timer++;
	timers[1].timer++;
	timers[2].timer++;
}

/*
uint8_t timer_ovf(uint8_t timer) {
	uint8_t ret = 0;
	
	if(timers[timer].flags == 1) {
		ret = 1;
		cli();
		timers[timer].flags = 0;
		sei();
	}
	return ret;
}

ISR (TIMER0_COMPA_vect)    // Timer1 ISR
{
	uint8_t i;
	PORTB ^= (1 << DDB0);
	if(num_timers != 255) {
		for(i = 0; i < num_timers; i++) {
			timers[i].timer++;
			if(timers[i].timer >= timers[i].ovf) {			
				timers[i].flags = 1;
				timers[i].timer = 0;
			}
		}
	}	
}

*/