/*
 * state_estimation.h
 *
 *  Created on: 11 Nov 2021
 *      Author: Airmarshall
 */

#ifndef INC_STATE_ESTIMATION_H_
#define INC_STATE_ESTIMATION_H_
#include "mathlib.h"
typedef struct sensor_data_st{
	vec3 acc;
	vec3 gyro;
	vec3 pos;
	float yaw;
}sensor_data;

typedef struct state_vec_st{
	vec3 p;
	vec3 v;
	vec3 E;
	vec3 w;
}state_vec;

void state_estimation_init(float t_step);//(state x0, state_cov P0, noise_cov Q_, obs_cov R_);
void state_estimation_update(sensor_data sensors, state_vec *X);
#endif /* INC_STATE_ESTIMATION_H_ */
