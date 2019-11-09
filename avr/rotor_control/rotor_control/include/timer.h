#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

void setup_timer(void);

uint8_t timer_register(uint8_t time);
uint8_t timer_ovf(uint8_t timer);

#endif