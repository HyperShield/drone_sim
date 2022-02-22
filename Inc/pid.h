#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct pid_st {
	float k_p;
	float k_i;
	float k_d;
	float u_max;
	float u_min;
	float i_e;
	float N;
	float d_e;
	float prev_e;
	float Ts;
	uint8_t AW;
}pid;

void pid_init(pid *c, float k_p, float k_i, float k_d, float u_max, float u_min, float N, float Ts);
float pid_update(pid *c, float y, float yd);
void pid_reset(pid *c);

#endif
