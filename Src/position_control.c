#include "position_control.h"
#include "pid.h"

pid pidx, pidy, pidz, pidvx, pidvy, pidvz;
void position_control_init(float t_step)
{
    pid_init(&pidx, 0.3f, 0.0f, 0.1f, 20.0f, -20.0f, 50.0f, t_step);
    pid_init(&pidy, 0.3f, 0.0f, 0.1f, 20.0f, -20.0f, 50.0f, t_step);
    pid_init(&pidz, 0.2f, 0.0f, 0.0f, 20.0f, -20.0f, 50.0f, t_step);

    pid_init(&pidvx, 6.5f, 0.0f, 0.0f, 50.0f, -50.0f, 50.0f, t_step);
    pid_init(&pidvy, 6.5f, 0.0f, 0.0f, 50.0f, -50.0f, 50.0f, t_step);
    pid_init(&pidvz, 1.0f, 0.0f, 0.0f, 50.0f, -50.0f, 50.0f, t_step);

}

void position_control_init2(float t_step)
{
    pid_init(&pidx, 0.3f, 0.0f, 0.0f, 20.0f, -20.0f, 50.0f, t_step);
    pid_init(&pidy, 0.3f, 0.0f, 0.0f, 20.0f, -20.0f, 50.0f, t_step);
    pid_init(&pidz, 0.2f, 0.0f, 0.0f, 20.0f, -20.0f, 50.0f, t_step);

    pid_init(&pidvx, 6.5f, 0.0f, 20.0f, 50.0f, -50.0f, 50.0f, t_step);
    pid_init(&pidvy, 6.5f, 0.0f, 20.0f, 50.0f, -50.0f, 50.0f, t_step);
    pid_init(&pidvz, 1.0f, 0.0f, 0.0f, 50.0f, -50.0f, 50.0f, t_step);

}
void position_control_update(vec3 p, vec3 pd, vec3 v, vec3 *f_n)
{
    float m = 1.27f, g = 9.81f;
	vec3 vd;
	vd.x = pid_update(&pidx, p.x, pd.x);
	vd.y = pid_update(&pidy, p.y, pd.y);
	vd.z = pid_update(&pidz, p.z, pd.z);

	f_n->x = m*pid_update(&pidvx, v.x, vd.x);
	f_n->y = m*pid_update(&pidvy, v.y, vd.y);
	f_n->z = m*(g-pid_update(&pidvz, v.z, vd.z));
}
