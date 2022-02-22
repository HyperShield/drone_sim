#ifndef POS_CONT_H
#define POS_CONT_H

#include "mathlib.h"

void position_control_init(float t_step);
void position_control_update(vec3 p, vec3 pd, vec3 v, vec3 *f_n);
void position_control_init2(float t_step);
#endif
