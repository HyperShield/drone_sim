#ifndef ATT_CONT_H
#define ATT_CONT_H
#include "mathlib.h"
#include "quadrotor.h"
void attitude_control_init2(float t_step);
void attitude_control_update2(quat qe, vec3 w, vec3 *tau);
void attitude_control_mapping2(vec3 f_n, vec3 E,quat *qe);
#endif
