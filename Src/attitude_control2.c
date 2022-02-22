#include "attitude_control2.h"
#include "pid.h"

#include <math.h>
#include <stdint.h>
pid pidwx, pidwy, pidwz;
void attitude_control_init2(float t_step)
{
    pid_init(&pidwx, 20.0f, 0.0f, 0.0f, 100.0f, -100.0f, 50.0f, t_step);
    pid_init(&pidwy, 20.0f, 0.0f, 0.0f, 100.0f, -100.0f, 50.0f, t_step);
    pid_init(&pidwz, 5.0f, 0.0f, 0.0f, 100.0f, -100.0f, 50.0f, t_step);
}
float sign(float x)
{
	if(x >= 0){
		return 1;
	} else {
		return -1;
	}
}
void attitude_control_update2(quat qe, vec3 w, vec3 *tau)
{
    float Jx = 0.0434f;
    float Jy = 0.0434f;
    float Jz = 0.0705f;
    float kq = 1.0f;
	vec3 wd;
	wd.x = kq*sign(qe.w)*qe.x;
	wd.y = kq*sign(qe.w)*qe.y;
	wd.z = kq*sign(qe.w)*qe.z;

	tau->x = Jx*pid_update(&pidwx, w.x, wd.x);
	tau->y = Jy*pid_update(&pidwy, w.y, wd.y);
	tau->z = Jz*pid_update(&pidwz, w.z, wd.z);
}

void attitude_control_mapping2(vec3 f_n, vec3 E,quat *qe)
{
	vec3 k;
	mat3 R = Eul_to_R(E);
	vec3 f_b = mat3_mult_vec3(mat3_transpose(R), f_n);

	vec3 ez = {0,0,-1};
	if(vec3_norm(f_b) > 0){
        k = vec3_cross(ez, vec3_mult_f(f_b,1.0f/vec3_norm(f_b)));
		if(vec3_norm(k) > 0)
			k = vec3_mult_f(k, 1/vec3_norm(k));
		else
			k = (vec3){0,0,0};
	} else {
		k = (vec3){0,0,0};
	}
    float th = 0.3*atanf(1.5*vec3_norm(f_b));
    //float th = acosf(vec3_dot(ez,vec3_mult_f(f_b,1/vec3_norm(f_b))));


	*qe = (quat){cosf(0.5f*th),k.x*sinf(0.5f*th),k.y*sinf(0.5f*th),k.z*sinf(0.5f*th)};
}
