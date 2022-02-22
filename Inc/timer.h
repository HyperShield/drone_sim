#ifndef TIMER_H
#define TIMER_H

typedef struct timer_st{
	float time;
	float period;
	int active;
	void (*interrupt)(struct timer_st *tim);
}timer;

void timer_init(timer *tim, float period, void(*irc)(struct timer_st *tim));
void timer_reset(timer *tim);
void timer_start(timer *tim);
void timer_stop(timer *tim);

#endif
