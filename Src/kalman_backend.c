#include "kalman_backend.h"

void kalman_backend_f(state *x,input *u,state *dot_x)
{
	vec3 g = {0.0f,0.0f,9.81f};
	vec3 acc = (vec3){u->a[0],u->a[1],u->a[2]};
	vec3 gyro = (vec3){u->a[3],u->a[4],u->a[5]};
	vec3 E = (vec3){x->a[6],x->a[7],x->a[8]};

	mat3 R = Eul_to_R(E);
	float phi = E.x, theta = E.y;
	mat3 W = {{ {1.0f, tanf(theta)*sinf(phi), tanf(theta)*cosf(phi)},
			    {0.0f, cosf(phi)           , -sinf(phi)},
				{0.0f, sinf(phi)/cosf(theta), cosf(phi)/cosf(theta)} }};

	vec3 dot_v = vec3_add(g,mat3_mult_vec3(R, acc));
	vec3 dot_E = mat3_mult_vec3(W, gyro);

	dot_x->a[0] = x->a[3];
	dot_x->a[1] = x->a[4];
	dot_x->a[2] = x->a[5];
	dot_x->a[3] = dot_v.x;
	dot_x->a[4] = dot_v.y;
	dot_x->a[5] = dot_v.z;
	dot_x->a[6] = dot_E.x;
	dot_x->a[7] = dot_E.y;
	dot_x->a[8] = dot_E.z;
}
//x = [x y z u v w phi theta psi]
//u = [acc gyro]
//dot_p = v
//dot_v = g - R*a
//dot_E = W*gyro
/*
[0, 0, 0, 1, 0, 0,                                                                                     0,                                                                      0,                                                                                                      0]
[0, 0, 0, 0, 1, 0,                                                                                     0,                                                                      0,                                                                                                      0]
[0, 0, 0, 0, 0, 1,                                                                                     0,                                                                      0,                                                                                                      0]

[0, 0, 0, 0, 0, 0,  -ay*(s(phi)*s(psi)+c(phi)*c(psi)*s(theta))-az*(c(phi)*s(psi)-c(psi)*s(phi)*s(theta)), ax*c(psi)*s(theta)-az*c(phi)*c(psi)*c(theta)-ay*c(psi)*c(theta)*s(phi), ay*(c(phi)*c(psi)+s(phi)*s(psi)*s(theta))-az*(c(psi)*s(phi)-c(phi)*s(psi)*s(theta))+ax*c(theta)*s(psi)]
[0, 0, 0, 0, 0, 0,   ay*(c(psi)*s(phi)-c(phi)*s(psi)*s(theta))+az*(c(phi)*c(psi)+s(phi)*s(psi)*s(theta)), ax*s(psi)*s(theta)-az*c(phi)*c(theta)*s(psi)-ay*c(theta)*s(phi)*s(psi), ay*(c(phi)*s(psi)-c(psi)*s(phi)*s(theta))-az*(s(phi)*s(psi)+c(phi)*c(psi)*s(theta))-ax*c(psi)*c(theta)]
[0, 0, 0, 0, 0, 0,                                                 az*c(theta)*s(phi)-ay*c(phi)*c(theta),                    ax*c(theta) + az*c(phi)*s(theta)+ay*s(phi)*s(theta),                                                                                                      0]

[0, 0, 0, 0, 0, 0,                                                   q*c(phi)*t(theta)-r*s(phi)*t(theta),                        r*c(phi)*(t(theta)^2+1)+q*s(phi)*(t(theta)^2+1),                                                                                                      0]
[0, 0, 0, 0, 0, 0,                                                                    -r*c(phi)-q*s(phi),                                                                      0,                                                                                                      0]
[0, 0, 0, 0, 0, 0,                                               (q*c(phi))/c(theta)-(r*s(phi))/c(theta),          (r*c(phi)*s(theta))/c(theta)^2+(q*s(phi)*s(theta))/c(theta)^2,                                                                                                      0]
 */
void kalman_backend_F(state *x, input *u, state_jac *F)
{
	float phi = x->a[6], theta = x->a[7], psi = x->a[8];
	float ax = u->a[0], ay = u->a[1], az = u->a[2];
	float q = u->a[4], r = u->a[5];

	float cphi = cosf(phi);
	float sphi = sinf(phi);
	float cthe = cosf(theta);
	float sthe = sinf(theta);
	float tthe = tanf(theta);
	float spsi = sinf(psi);
	float cpsi = cosf(psi);

	float F36 = -ay*(sphi*spsi+cphi*cpsi*sthe)-az*(cphi*spsi-cpsi*sphi*sthe);
	float F37 = ax*cpsi*sthe-az*cphi*cpsi*cthe-ay*cpsi*cthe*sphi;
	float F38 = ay*(cphi*cpsi+sphi*spsi*sthe)-az*(cpsi*sphi-cphi*spsi*sthe)+ax*cthe*spsi;
	float F46 = ay*(cpsi*sphi-cphi*spsi*sthe)+az*(cphi*cpsi+sphi*spsi*sthe);
	float F47 = ax*spsi*sthe-az*cphi*cthe*spsi-ay*cthe*sphi*spsi;
	float F48 = ay*(cphi*spsi-cpsi*sphi*sthe)-az*(sphi*spsi+cphi*cpsi*sthe)-ax*cpsi*cthe;
	float F56 = az*cthe*sphi-ay*cphi*cthe;
	float F57 = ax*cthe + az*cphi*sthe+ay*sphi*sthe;
	float F66 = q*cphi*tthe-r*sphi*tthe;
	float F67 = r*cphi*(tthe*tthe+1)+q*sphi*(tthe*tthe+1);
	float F76 = -r*cphi-q*sphi;
	float F86 = (q*cphi)/cthe-(r*sphi)/cthe;
	float F87 = (r*cphi*sthe)/(cthe*cthe)+(q*sphi*sthe)/(cthe*cthe);

/*		0			  	 1					2					3			  	 4				  5					6				 7				  8			 */
/*0*/	F->a[0]  = 0.0f, F->a[1]  = 0.0f, 	F->a[2]  = 0.0f, 	F->a[3]  = 1.0f, F->a[4]  = 0.0f, F->a[5]  = 0.0f, 	F->a[6]  = 0.0f, F->a[7]  = 0.0f, F->a[8]  = 0.0f;
/*1*/	F->a[9]  = 0.0f, F->a[10] = 0.0f,	F->a[11] = 0.0f, 	F->a[12] = 0.0f, F->a[13] = 1.0f, F->a[14] = 0.0f, 	F->a[15] = 0.0f, F->a[16] = 0.0f, F->a[17] = 0.0f;
/*2*/	F->a[18] = 0.0f, F->a[19] = 0.0f,	F->a[20] = 0.0f, 	F->a[21] = 0.0f, F->a[22] = 0.0f, F->a[23] = 1.0f, 	F->a[24] = 0.0f, F->a[25] = 0.0f, F->a[26] = 0.0f;

/*3*/	F->a[27] = 0.0f, F->a[28] = 0.0f,	F->a[29] = 0.0f, 	F->a[30] = 0.0f, F->a[31] = 0.0f, F->a[32] = 0.0f, 	F->a[33] = F36,  F->a[34] = F37,  F->a[35] = F38;
/*4*/	F->a[36] = 0.0f, F->a[37] = 0.0f,	F->a[38] = 0.0f, 	F->a[39] = 0.0f, F->a[40] = 0.0f, F->a[41] = 0.0f, 	F->a[42] = F46,  F->a[43] = F47,  F->a[44] = F48;
/*5*/	F->a[45] = 0.0f, F->a[46] = 0.0f,	F->a[47] = 0.0f, 	F->a[48] = 0.0f, F->a[49] = 0.0f, F->a[50] = 0.0f, 	F->a[51] = F56,  F->a[52] = F57,  F->a[53] = 0.0f;

/*6*/	F->a[54] = 0.0f, F->a[55] = 0.0f,	F->a[56] = 0.0f, 	F->a[57] = 0.0f, F->a[58] = 0.0f, F->a[59] = 0.0f, 	F->a[60] = F66,  F->a[61] = F67,  F->a[62] = 0.0f;
/*7*/	F->a[63] = 0.0f, F->a[64] = 0.0f,	F->a[65] = 0.0f, 	F->a[66] = 0.0f, F->a[67] = 0.0f, F->a[68] = 0.0f, 	F->a[69] = F76,  F->a[70] = 0.0f, F->a[71] = 0.0f;
/*8*/	F->a[72] = 0.0f, F->a[73] = 0.0f,	F->a[74] = 0.0f, 	F->a[75] = 0.0f, F->a[76] = 0.0f, F->a[77] = 0.0f, 	F->a[78] = F86,  F->a[79] = F87,  F->a[80] = 0.0f;
}
void kalman_backend_h(state *x, input *u, obs *y)
{
	vec3 g = (vec3){0.0f,0.0f,-1.0f};
	vec3 E = (vec3){x->a[6],x->a[7],x->a[8]};
	mat3 R = Eul_to_R(E);
	vec3 g_b = mat3_mult_vec3(mat3_transpose(R), g);
	y->a[0] = x->a[0];
	y->a[1] = x->a[1];
	y->a[2] = x->a[2];
	y->a[3] = g_b.x;
	y->a[4] = g_b.y;
	y->a[5] = g_b.z;
	y->a[6] = E.z;
}
void kalman_backend_H(state *x, input *u, obs_jac *H)
{
	float phi = x->a[6], theta = x->a[7];

	float cphi = cosf(phi);
	float sphi = sinf(phi);
	float cthe = cosf(theta);
	float sthe = sinf(theta);

	float F37 = cthe;
	float F46 = -cphi*cthe;
	float F47 = sphi*sthe;
	float F56 = cthe*sphi;
	float F57 = cphi*sthe;
/*		0			  	 1					2			  		3			     4			  	  5					6			  		7					8			 */
/*0*/	H->a[0] =  1.0f, H->a[1] =  0.0f,	H->a[2] =  0.0f, 	H->a[3] =  0.0f, H->a[4] =  0.0f, H->a[5] =  0.0f, 	H->a[6] =  0.0f, 	H->a[7] =  0.0f, 	H->a[8] =  0.0f;
/*1*/	H->a[9] =  0.0f, H->a[10] = 1.0f,	H->a[11] = 0.0f, 	H->a[12] = 0.0f, H->a[13] = 0.0f, H->a[14] = 0.0f, 	H->a[15] = 0.0f,	H->a[16] = 0.0f, 	H->a[17] = 0.0f;
/*2*/	H->a[18] = 0.0f, H->a[19] = 0.0f,	H->a[20] = 1.0f, 	H->a[21] = 0.0f, H->a[22] = 0.0f, H->a[23] = 0.0f, 	H->a[24] = 0.0f, 	H->a[25] = 0.0f,	H->a[26] = 0.0f;
/*3*/	H->a[27] = 0.0f, H->a[28] = 0.0f,	H->a[29] = 0.0f, 	H->a[30] = 0.0f, H->a[31] = 0.0f, H->a[32] = 0.0f,	H->a[33] = 0.0f, 	H->a[34] = F37, 	H->a[35] = 0.0f;
/*4*/	H->a[36] = 0.0f, H->a[37] = 0.0f,	H->a[38] = 0.0f, 	H->a[39] = 0.0f, H->a[40] = 0.0f, H->a[41] = 0.0f, 	H->a[42] = F46, 	H->a[43] = F47, 	H->a[44] = 0.0f;
/*5*/	H->a[45] = 0.0f, H->a[46] = 0.0f,	H->a[47] = 0.0f, 	H->a[48] = 0.0f, H->a[49] = 0.0f, H->a[50] = 0.0f, 	H->a[51] = F56,		H->a[52] = F57, 	H->a[53] = 0.0f;
/*6*/	H->a[54] = 0.0f, H->a[55] = 0.0f,	H->a[56] = 0.0f, 	H->a[57] = 0.0f, H->a[58] = 0.0f, H->a[59] = 0.0f,	H->a[60] = 0.0f, 	H->a[61] = 0.0f, 	H->a[62] = 1.0f;
}

/*Midpoint method*/
void kalman_backend_integrate(state *x, state_cov *P, input *u, state *dot_x, state_cov *dot_P,float t_step)
{
	static state x_m = {{0.0f}};

	//P = P + t_step*dot_P
	mat_square_add(P->a,mat_square_mult_f(dot_P->a,t_step,DIM_STATE),P->a,DIM_STATE);

	//x = x + t_step*f(x+t_step/2.0f*dot_x,u) (RK2)
	mat_add(x->a, mat_mult_f(dot_x->a, t_step/2.0f, DIM_STATE, 1),x_m.a,DIM_STATE,1);
	kalman_backend_f(&x_m, u, dot_x);
	mat_add(x->a, mat_mult_f(dot_x->a, t_step, DIM_STATE, 1), x->a, DIM_STATE, 1);
	//mat_add(x->a,mat_mult_f(dot_x->a,t_step,DIM_STATE,1),x->a,DIM_STATE,1);
}
