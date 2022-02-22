/*
 * state_estimation.c
 *
 *  Created on: 11 Nov 2021
 *      Author: Airmarshall
 */

#include "state_estimation.h"
#include "kalman_filter.h"
#include "kalman_filter_types.h"

int obs_updated = 0;
float t_step;
void state_estimation_init(float t)//(state x0, state_cov P0, noise_cov Q_, obs_cov R_)
{
	kalman_filter_init((state){{0,0,0,
								0,0,0,
								0,0,0}}, (state_cov){{2,0,0, 0,0,0, 0,0,0,
													  0,2,0, 0,0,0, 0,0,0,
													  0,0,2, 0,0,0, 0,0,0,
													  0,0,0, 2,0,0, 0,0,0,
													  0,0,0, 0,2,0, 0,0,0,
													  0,0,0, 0,0,2, 0,0,0,
													  0,0,0, 0,0,0, 2,0,0,
													  0,0,0, 0,0,0, 0,2,0,
													  0,0,0, 0,0,0, 0,0,2}},
											(noise_cov){{  2,0,0, 0,0,0, 0,0,0,
														   0,2,0, 0,0,0, 0,0,0,
														   0,0,2, 0,0,0, 0,0,0,
														   0,0,0, 2,0,0, 0,0,0,
														   0,0,0, 0,2,0, 0,0,0,
														   0,0,0, 0,0,2, 0,0,0,
														   0,0,0, 0,0,0, 2,0,0,
														   0,0,0, 0,0,0, 0,2,0,
														   0,0,0, 0,0,0, 0,0,2}},
											(obs_cov){{2,0,0, 0,0,0, 0,
													   0,2,0, 0,0,0, 0,
													   0,0,2, 0,0,0, 0,
                                                       0,0,0, 5,0,0, 0,
                                                       0,0,0, 0,5,0, 0,
                                                       0,0,0, 0,0,5, 0,
													   0,0,0, 0,0,0, 2}});
	obs_updated = 0;
	t_step = t;
}
void state_estimation_update(sensor_data sensors, state_vec *X)
{
    input u = {{sensors.acc.x,sensors.acc.y,sensors.acc.z,sensors.gyro.x,sensors.gyro.y,sensors.gyro.z}};

	kalman_filter_predict(&u, t_step);
	obs_updated++;
	if(obs_updated > 10){
		float a = vec3_norm(sensors.acc);
		a = a > 0.01 ? a : 1;
		obs z = {{sensors.pos.x,sensors.pos.y,sensors.pos.z,sensors.acc.x/a,sensors.acc.y/a,sensors.acc.z/a,sensors.yaw}};
		kalman_filter_correct(&z, &u);
		obs_updated = 0;
	}
	//Control
	state x = kalman_filter_get_state();
	X->p = (vec3){x.a[0],x.a[1],x.a[2]};
	X->v = (vec3){x.a[3],x.a[4],x.a[5]};
	X->E = (vec3){x.a[6],x.a[7],x.a[8]};
}
