#ifndef KALMAN_H
#define KALMAN_H
#include "mathlib.h"
#include "math.h"
#include "kalman_filter_types.h"
#include "kalman_backend.h"

/*struct kalman_filter_st{
	state x;
	state_cov P;
	state dot_x;
	state_cov dot_P;
	noise_cov Q;
	obs_cov R;
	state_jac F;
	obs_jac H;
}kalman_filter;

void kalman_filter_init(kalman_filter *KF, state x0, state_cov P0, noise_cov Q_, obs_cov, R_);
void kalman_filter_reset(kalman_filter *KF, float a);
void kalman_filter_predict(kalman_filter *KF, input *u);
void kalman_filter_update(kalman_filter *KF, obs *z);
state kalman_filter_get_state(kalman_filter *KF);
*/

void kalman_filter_init(state x0, state_cov P0, noise_cov Q_, obs_cov R_);
void kalman_filter_reset(state x0, state_cov P0);
void kalman_filter_predict(input *u, float t_step);
void kalman_filter_correct(obs *z, input *u);

state kalman_filter_get_state();

#endif
