#include "attitude_control.h"
#include "pid.h"
#include <math.h>
#include <stdint.h>
pid pidphi, pidtheta, pidpsi, pidwx, pidwy, pidwz;
void attitude_control_init(float t_step)
{
    pid_init(&pidphi,   5.0f, 0.0f, 0.0f, 20.0f, -20.0f, 50.0f, t_step);
    pid_init(&pidtheta, 5.0f, 0.0f, 0.0f, 20.0f, -20.0f, 50.0f, t_step);
    pid_init(&pidpsi,   1.0f, 0.0f, 0.0f, 20.0f, -20.0f, 50.0f, t_step);

    pid_init(&pidwx, 20.0f, 0.0f, 0.0f, 100.0f, -100.0f, 50.0f, t_step);
    pid_init(&pidwy, 20.0f, 0.0f, 0.0f, 100.0f, -100.0f, 50.0f, t_step);
    pid_init(&pidwz, 5.0f, 0.0f, 0.0f, 100.0f, -100.0f, 50.0f, t_step);
}
void attitude_control_update(vec3 E, vec3 Ed, vec3 w, vec3 *tau)
{
    float Jx = 0.0434f;
    float Jy = 0.0434f;
    float Jz = 0.0705f;
	vec3 wd;
	wd.x = pid_update(&pidphi, E.x, Ed.x);
	wd.y = pid_update(&pidtheta, E.y, Ed.y);
	wd.z = pid_update(&pidpsi, E.z, Ed.z);

	tau->x = Jx*pid_update(&pidwx, w.x, wd.x);
	tau->y = Jy*pid_update(&pidwy, w.y, wd.y);
	tau->z = Jz*pid_update(&pidwz, w.z, wd.z);
}

void attitude_control_mapping(vec3 f_n, vec3 E,vec3 *Ed)
{
	float g = 9.81f;
    float max_tilt = 0.5f;
	//Attitude controller
	float phi_des = 1.0f/g*(sinf(E.z)*f_n.x + cosf(E.z)*f_n.y);
	if(fabs(phi_des) > max_tilt){
		if(phi_des > 0.0f)
			phi_des = max_tilt;
		else
			phi_des = -max_tilt;
	}
	float theta_des = -1.0f/g*(cosf(E.z)*f_n.x - sinf(E.z)*f_n.y);
	if(fabs(theta_des) > max_tilt){
		if(theta_des > 0.0f)
			theta_des = max_tilt;
		else
			theta_des = -max_tilt;
	}
	Ed->x = phi_des;
	Ed->y = theta_des;
	Ed->z = 0.0f;
}
