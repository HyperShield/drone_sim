#include <stdio.h>
#include "timer.h"

void timer_init(timer *tim, float period, void(*irc)(timer *tim))
{
	tim->period = period;
	tim->interrupt = irc;
	tim->time = 0;
}
void timer_reset(timer *tim)
{
	if(tim->active){
		printf("Error, stop timer before resetting!");
	} else {
		tim->time = 0;
	}
}
void timer_start(timer *tim)
{
	tim->active = 1;
}
void timer_stop(timer *tim)
{
	tim->active = 0;
}
