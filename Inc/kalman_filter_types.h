#ifndef KALMAN_TYPES_H
#define KALMAN_TYPES_H

#define DIM_STATE 9
#define DIM_OBS 7
#define DIM_INPUT 6

typedef struct state_st{
	float a[DIM_STATE];
}state;

typedef struct input_st{
	float a[DIM_INPUT];
}input;

typedef struct obs_st{
	float a[DIM_OBS];
}obs;

typedef struct state_cov_st{
	float a[DIM_STATE*DIM_STATE];
}state_cov;

typedef struct noise_cov_st{
	float a[DIM_STATE*DIM_STATE];
}noise_cov;

typedef struct obs_cov_st{
	float a[DIM_OBS*DIM_OBS];
}obs_cov;

typedef struct state_jac_st{
	float a[DIM_STATE*DIM_STATE];
}state_jac;

typedef struct obs_jac_st{
	float a[DIM_OBS*DIM_STATE];
}obs_jac;

typedef struct kalman_gain_st{
	float a[DIM_STATE*DIM_OBS];
}kalman_gain;

#endif
