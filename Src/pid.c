#include "pid.h"

void pid_init(pid *c, float k_p, float k_i, float k_d, float u_max, float u_min, float N, float Ts)
{
	c->k_p = k_p;
	c->k_i = k_i;
	c->k_d = k_d;
	c->u_max = u_max;
	c->u_min = u_min;
	c->N = N;
	c->Ts= Ts;
	c->i_e = 0.0f;
	c->d_e = 0.0f;
	c->prev_e = 0.0f;
	c->AW = 0;
}
float pid_update(pid *c, float y, float yd)
{
	float e = yd - y;
	float out;
	if(c->AW == 0)
		c->i_e += c->Ts*e;

	c->d_e = c->N/(1+c->N*c->Ts/2.0f)*(e-c->prev_e) + (1-c->N*c->Ts/2.0f)/(1+c->N*c->Ts/2.0f)*c->d_e;

	out = c->k_p*e + c->k_i*c->i_e + c->k_d*c->d_e;

	if(out >= c->u_max){
		out = c->u_max;
		c->AW = 1;
	} else if(out <= c->u_min){
		out = c->u_min;
		c->AW = 1;
	} else {
		c->AW = 0;
	}
	c->prev_e = e;
	return out;
}
void pid_reset(pid *c)
{
	c->i_e = 0.0f;
	c->d_e = 0.0f;
	c->prev_e = 0.0f;
	c->AW = 0;
}
