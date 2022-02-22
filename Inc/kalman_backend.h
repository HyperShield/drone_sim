#ifndef KSYSTEM_H
#define KSYSTEM_H
#include <math.h>
#include "mathlib.h"
#include "matrix_lib.h"

#include "kalman_filter_types.h"

void kalman_backend_f(state *x, input *u, state *dot_x);
void kalman_backend_F(state *x, input *u, state_jac *F);
void kalman_backend_h(state *x, input *u, obs *y);
void kalman_backend_H(state *x, input *u, obs_jac *H);
void kalman_backend_integrate(state *x, state_cov *P, input *u, state *dot_x, state_cov *dot_P,float t_step);
#endif
