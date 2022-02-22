#ifndef ATT_CONT_H
#define ATT_CONT_H
#include "mathlib.h"

void attitude_control_init(float t_step);
void attitude_control_update(vec3 E, vec3 Ed, vec3 w, vec3 *tau);
void attitude_control_mapping(vec3 f_n, vec3 E,vec3 *Ed);
#endif
