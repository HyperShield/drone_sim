#include "kalman_filter.h"
#include "matrix_lib.h"
#include <stdio.h>
state x;
state_cov P;
state dot_x;
state_cov dot_P;
noise_cov Q;
obs_cov R;
state_jac F;
obs_jac H;
obs y;
kalman_gain K;
float I9[DIM_STATE*DIM_STATE];
//Initialize Q, R, P(0) and x(0)
void kalman_filter_init(state x0, state_cov P0, noise_cov Q_, obs_cov R_)
{
	x = x0;
	P = P0;
	Q = Q_;
	R = R_;
	int i,j;
	for(i = 0; i < DIM_STATE;++i){
		for(j=0;j<DIM_STATE;++j){
			I9[i*DIM_STATE + j] = i==j ? 1.0f : 0.0f;
		}
	}
}
//Set P = a*I and x = 0
void kalman_filter_reset(state x0, state_cov P0)
{
	x = x0;
	P = P0;
}

//Prediction step
//Assumes: dot_x = f(x,u) + w, where u = [acc;gyro] and w is vector of white noise
//		   dot_P = F'*P*F + Q where F is the state jacobian and Q is the noise covariance
void kalman_filter_predict(input *u, float t_step)
{

	static float FmP[DIM_STATE*DIM_STATE] = {0.0f};
	static float PmFT[DIM_STATE*DIM_STATE] = {0.0f};
	static float FT[DIM_STATE*DIM_STATE] = {0.0f};
	static float FmPpPmFT[DIM_STATE*DIM_STATE] = {0.0f};

	//dot_x = f(x,u) + w
	kalman_backend_f(&x, u, &dot_x);

	//F = d(f(x*,u)/dx where x* is the current state
	kalman_backend_F(&x, u, &F);

	//dot_P = F*P + P*F' + Q
	mat_square_mult(F.a, P.a, FmP, DIM_STATE);
	mat_square_mult(P.a, mat_square_transpose(F.a, FT, DIM_STATE), PmFT, DIM_STATE);
	mat_square_add(FmP, PmFT, FmPpPmFT, DIM_STATE);
	mat_square_add(FmPpPmFT, Q.a, dot_P.a, DIM_STATE);

	/*mat_square_mult(F.a,P.a,FmP,DIM_STATE);
	mat_square_mult(FmP,mat_square_transpose(F.a,FT,DIM_STATE),FmPmFT,DIM_STATE);
	mat_square_add(FmPmFT,Q.a,P.a,DIM_STATE);*/

	//Numeric integration
	kalman_backend_integrate(&x, &P, u, &dot_x, &dot_P, t_step);
	//printf("P: \n");
	//mat_print(P.a, DIM_STATE, DIM_STATE);
}
//Update step
void kalman_filter_correct(obs *z, input *u)
{
	kalman_backend_H(&x, u, &H);
	kalman_backend_h(&x, u, &y);

//Kalman gain
	static float L[DIM_OBS*DIM_OBS] = {0.0f};
	static float IL[DIM_OBS*DIM_OBS] = {0.0f};
	static float PmHT[DIM_STATE*DIM_OBS] = {0.0f};
	static float HmPmHT[DIM_OBS*DIM_OBS] = {0.0f};
	static float HT[DIM_STATE*DIM_OBS] = {0.0f};
	//P*H'
	mat_mult(P.a, mat_transpose(H.a,HT,DIM_OBS,DIM_STATE), PmHT, DIM_STATE, DIM_STATE, DIM_STATE, DIM_OBS);

	//L = H*P*H'+ R
	mat_square_add(mat_mult(H.a, PmHT, HmPmHT, DIM_OBS, DIM_STATE, DIM_STATE, DIM_OBS), R.a, L, DIM_OBS);
	//mat_print(L, n, m)
	static int32_t LP[DIM_OBS+1] = {0.0f};
	uint8_t res = LUPDecompose(L, DIM_OBS, 0.001f, LP);
	if(res==0)
		return;
	LUPInvert(L, LP, DIM_OBS, IL);
	//K = P*H'*L^-1
	mat_mult(PmHT, IL, K.a, DIM_STATE, DIM_OBS, DIM_OBS, DIM_OBS);

//state correction
	static float z_y[DIM_OBS] = {0.0f};
	static float x_K[DIM_STATE] = {0.0f};
	//x_K = K*(z-y)
	mat_mult(K.a, mat_sub(z->a, y.a, z_y, DIM_OBS, 1), x_K, DIM_STATE, DIM_OBS, DIM_OBS, 1);
	//x = x + x_K
	mat_add(x.a, x_K, x.a, DIM_STATE, 1);

//Covariance correction
	//P = (I - K*H)*P*(I - K*H)' + K*R*K' (Joseph form)
	static float KmH[DIM_STATE*DIM_STATE];
	static float IpKmH[DIM_STATE*DIM_STATE];
	static float IpKmHT[DIM_STATE*DIM_STATE];
	static float PmIpKmHT[DIM_STATE*DIM_STATE];
	static float IpKmHmPmIpKmHT[DIM_STATE*DIM_STATE];
	static float KmR[DIM_STATE*DIM_OBS];
	static float KT[DIM_STATE*DIM_OBS];
	static float KmRmKT[DIM_STATE*DIM_STATE];

	mat_square_sub(I9, mat_mult(K.a, H.a, KmH, DIM_STATE, DIM_OBS, DIM_OBS, DIM_STATE), IpKmH, DIM_STATE);
	mat_square_mult(IpKmH, mat_square_mult(P.a, mat_square_transpose(IpKmH, IpKmHT, DIM_STATE), PmIpKmHT, DIM_STATE), IpKmHmPmIpKmHT, DIM_STATE);
	mat_mult(mat_mult(K.a, R.a, KmR, DIM_STATE, DIM_OBS, DIM_OBS, DIM_OBS), mat_transpose(K.a, KT, DIM_STATE, DIM_OBS), KmRmKT, DIM_STATE, DIM_OBS, DIM_OBS, DIM_STATE);
	mat_square_add(IpKmHmPmIpKmHT, KmRmKT, P.a, DIM_STATE);
}

state kalman_filter_get_state()
{
	return x;
}
