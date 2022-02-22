#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "mathlib.h"
#include "blender_socket.h"
#include "position_control.h"
#include "state_estimation.h"
#include "matrix_lib.h"
#include "timer.h"
#include "quadrotor.h"

#define WAYPOINT 1
#define ETH 0

#if ETH == 1
    #include "attitude_control2.h"
#else
    #include "attitude_control.h"
#endif
void dynamics(quadrotor *Quad, vec3 p_n, vec3 v_n, vec3 E, vec3 w_b_nb, vec3 *dot_p_n, vec3 *dot_v_n, vec3 *dot_E, vec3 *dot_w_b_nb, vec4 pwm);
void update(quadrotor *Quad, vec4 pwm, float t_step);
mat3 Eul_to_R(vec3 E);
vec4 forces_to_pwm(quadrotor *Quad, vec4 f_u);
vec3 blade_flapping(quadrotor *Quad, float Ui, vec3 vi, vec3 w_b_nb);
void tim1_interrupt_handler(timer *tim);
void tim2_interrupt_handler(timer *tim);
void update_timers(timer timers[], int amount_timers, float t_step);
void get_sensor_measurements(quadrotor *Quad, vec3 *acc,vec3 *gyro, vec4 pwm);
float normal_distribution(float mean, float std);
typedef struct flags_st{
	int timer1_overflow;
	int timer2_overflow;
}interrupt_flags;
interrupt_flags flags;

int main(void)
{
    //Initialize simulator
    int i;
    float t_end = 50.0f;
    float t_step = 0.0001f;
    flags.timer1_overflow = 0;
    flags.timer2_overflow = 0;

    quadrotor Quad;
    quadrotor_ctor_def(&Quad);

	vec3 f_n = {0,0,0};
#if WAYPOINT == 1
    vec3 p_nd[5];
    {
        int i;
        for(i = 0;i < 5; ++i){
            p_nd[i] = (vec3){-3+rand()/(float)RAND_MAX*(9+3),
                              -6+rand()/(float)RAND_MAX*(7+6),
                               0-rand()/(float)RAND_MAX*(7+0)};
            vec3_print(p_nd[i]);
        }
    }
#else
    vec3 p_nd = {0,0,-1};
#endif
	int n = 0;
	vec3 tau = {0,0,0};
	vec4 pwm = {0};

    state_vec state = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f}};
    sensor_data sensors = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},0.0f};

	const int timer_amount = 2;
	timer timers[timer_amount];
    timer_init(&timers[0], 0.005f, tim1_interrupt_handler);
    timer_init(&timers[1], 0.04f, tim2_interrupt_handler);


#if ETH == 1
    position_control_init2(timers[0].period);
    attitude_control_init2(timers[0].period);
#else
    position_control_init(timers[0].period);
    attitude_control_init(timers[0].period);
#endif
    state_estimation_init(timers[0].period);

	if((init_socket()) > 0)
	{
		timer_start(&timers[0]);
		timer_start(&timers[1]);
		for(i = 0; i < (int)t_end/t_step;++i){
			//Calculate derivatives (Calculating control input, applying input to dynamics and kinematics)
			//Numerical integration (runge-kutta 4)
            update(&Quad, pwm, t_step);
			if(flags.timer1_overflow){
				flags.timer1_overflow = 0;
				//Estimation
                get_sensor_measurements(&Quad,&sensors.acc,&sensors.gyro,pwm);
                sensors.pos = Quad.x.p;
                sensors.yaw = Quad.x.E.z;
                state_estimation_update(sensors,&state);
				//Control
#if WAYPOINT == 1
                if((vec3_norm(vec3_sub(Quad.x.p, p_nd[n]))<0.25)){
					n++;
					n = n%5;
					printf("Waypoint reached\n");
                }
                position_control_update(state.p, p_nd[n], state.v, &f_n);
#else
                position_control_update(state.p, p_nd, state.v, &f_n);
#endif
#if ETH == 1
                quat qe;
                attitude_control_mapping2(f_n,state.E,&qe);
                attitude_control_update2(qe, sensors.gyro, &tau);
#else
                vec3 E_d;
                attitude_control_mapping(f_n,state.E,&E_d);
                attitude_control_update(state.E,E_d, sensors.gyro, &tau);
#endif
                pwm = forces_to_pwm(&Quad,(vec4){{f_n.z,tau.x,tau.y,tau.z}});
			}
			if(flags.timer2_overflow){
				flags.timer2_overflow = 0;
                send_to_blender(Quad.x.p,Quad.x.E,pwm);
			}
			update_timers(timers,timer_amount,t_step);
		}
        vec3_print(Quad.x.p);
	} else {
		printf("Connection error\n");
	}
	close_socket();
	return 0;
}
void get_sensor_measurements(quadrotor *Quad, vec3 *acc,vec3 *gyro, vec4 pwm)
{
    parameters Quadparams = Quad->params.st;
    mat3 R = Eul_to_R(Quad->x.E);

    float m_a = 0.2f, m_w = 0.1f;
	vec3 n_w = {normal_distribution(0, m_w),normal_distribution(0, m_w),normal_distribution(0, m_w)};
	vec3 n_a = {normal_distribution(0, m_a),normal_distribution(0, m_a),normal_distribution(0, m_a)};

    int i;
    vec3 T = {0,0,0};
    mat3 D = {{{0.01,0,0},{0,0.01,0},{0,0,0}}};
    vec3 pr[4];
    pr[0] = (vec3){Quadparams.l,0,-Quadparams.h};
    pr[1] = (vec3){0,Quadparams.l,-Quadparams.h};
    pr[2] = (vec3){-Quadparams.l,0,-Quadparams.h};
    pr[3] = (vec3){0,-Quadparams.l,-Quadparams.h};
    //Blade flapping
    for(i = 0; i < 4;++i){
        vec3 vi = vec3_add(mat3_mult_vec3(mat3_transpose(R),Quad->x.v),vec3_cross(Quad->x.w,pr[i]));
        float Ui = sqrtf(Quadparams.u_max*Quadparams.u_max*pwm.a[i]);
        vec3 Ti = blade_flapping(Quad,Ui,vi,Quad->x.w);
        vec3 Di = vec3_mult_f(mat3_mult_vec3(D,vi),-Quadparams.C_T*Ui*Ui);

        T = vec3_add(T,vec3_add(Di,Ti));
    }

    *gyro = vec3_add(Quad->x.w,n_w);
    *acc = vec3_add(vec3_mult_f(T,1.0f/Quadparams.m),n_a); //(F_T+F_A)/m
}
//Box-Muller transform
float normal_distribution(float mean, float std)
{
	float eps = 1.0f/RAND_MAX;
	float u1 = 0.0f;
	float u2 = 0.0f;
	while(u1 < eps){
		u1 = (float)rand()*eps;
	}
	while(u2 < eps){
			u2 = (float)rand()*eps;
	}
	return std*sqrtf(-2*logf(u1))*sinf(2*M_PI*u2)+mean;
}

vec3 blade_flapping(quadrotor *Quad, float Ui, vec3 vi, vec3 w_b_nb)
{
    if(Ui < 1){
        return (vec3){0,0,0};
    }
    float thetat = 2.0f/3.0f*Quad->params.st.theta0 + 1.0f/2.0f*Quad->params.st.theta1;
    float psi = atan2f(vi.y,vi.x);
    float mu = vec3_norm(vi)/(Ui*Quad->params.st.r);
    float cT = Quad->params.st.C_T/(Quad->params.st.rho*M_PI*pow(Quad->params.st.r,4));
    float lambda = sqrtf(cT/2.0f);
    float y = Quad->params.st.rho*Quad->params.st.a0*Quad->params.st.c0*pow(Quad->params.st.r,4)/Quad->params.st.Ib;

    float u = 2.0f*mu*(4.0f/3.0f*thetat-lambda);
    float v = 4.0f/3.0f*mu*y/8.0f*(thetat-4.0f/3.0f*lambda);

    float alpha = u*cosf(psi) - v*sinf(psi) - 16.0f/y*w_b_nb.y/Ui + w_b_nb.x/Ui;
    float beta = u*sinf(psi) + v*cosf(psi) - 16.0f/y*w_b_nb.x/Ui + w_b_nb.y/Ui;

    float Tr = Ui*Ui*Quad->params.st.C_T;
    vec3 T = {-Tr*sinf(alpha),Tr*cosf(alpha)*sinf(alpha),-Tr*cosf(alpha)*cosf(beta)};
    return T;
}

void dynamics(quadrotor *Quad, vec3 p_n, vec3 v_n, vec3 E, vec3 w_b_nb, vec3 *dot_p_n, vec3 *dot_v_n, vec3 *dot_E, vec3 *dot_w_b_nb, vec4 pwm)
{
    parameters Quadparams = Quad->params.st;

    float g = 9.81;
    mat3 J = { {{Quadparams.Jx,0,0},{0,Quadparams.Jy,0},{0,0,Quadparams.Jz}} };
	//mat3 J = { {{1,0,0},{0,1,0},{0,0,1}} };
    vec3 f_G = {0,0,Quadparams.m*g};

	float phi = E.x, theta = E.y;
	mat3 W = {{ {1, tanf(theta)*sinf(phi), tanf(theta)*cosf(phi)},
			    {0, cosf(phi)           , -sinf(phi)},
				{0, sinf(phi)/cosf(theta), cosf(phi)/cosf(theta)} }};
	mat3 R = Eul_to_R(E);
    vec4 U;
    int i;
    for(i = 0; i < 4; ++i){
        U.a[i] = sqrtf(Quadparams.u_max*Quadparams.u_max*pwm.a[i]);
    }

    vec3 T = {0,0,0};
    vec3 tau_xy = {0,0,0};
    float tau_z = 0;
    vec3 e3 = {0,0,1};
    mat3 D = {{{0.01,0,0},{0,0.01,0},{0,0,0}}};
    vec3 pr[4];
    pr[0] = (vec3){Quadparams.l,0,-Quadparams.h};
    pr[1] = (vec3){0,Quadparams.l,-Quadparams.h};
    pr[2] = (vec3){-Quadparams.l,0,-Quadparams.h};
    pr[3] = (vec3){0,-Quadparams.l,-Quadparams.h};
    //Blade flapping
    for(i = 0; i < 4;++i){
        vec3 vi = vec3_add(mat3_mult_vec3(mat3_transpose(R),v_n),vec3_cross(w_b_nb,pr[i]));

        vec3 Ti = blade_flapping(Quad,U.a[i],vi,w_b_nb);
        vec3 Di = vec3_mult_f(mat3_mult_vec3(D,vi),-Quadparams.C_T*U.a[i]*U.a[i]);

        T = vec3_add(T,vec3_add(Di,Ti));

        vec3 Gi = vec3_mult_f(vec3_cross(w_b_nb,e3),pow(-1,i)*Quadparams.Ib*U.a[i]);
        tau_xy = vec3_add(tau_xy,vec3_add(vec3_cross(pr[i],Ti),Gi));
        tau_z += pow(-1,i)*Quadparams.C_Q*U.a[i]*U.a[i];
    }
    vec3 tau = vec3_add(tau_xy,vec3_mult_f(e3,tau_z));

    *dot_p_n = v_n;
    *dot_v_n = vec3_mult_f(vec3_add(f_G,mat3_mult_vec3(R,T)),1.0f/Quadparams.m);
    *dot_E = mat3_mult_vec3(W, w_b_nb);
    *dot_w_b_nb = mat3_mult_vec3(mat3_inverse(J), (vec3_sub(tau, vec3_cross(w_b_nb, mat3_mult_vec3(J, w_b_nb)))));
}
void update(quadrotor *Quad, vec4 pwm, float t_step)
{
	//RK4
	vec3 K[4][4] = {{{0,0,0}}};
    vec3 p_n = Quad->x.p;
    vec3 v_n = Quad->x.v;
    vec3 E = Quad->x.E;
    vec3 w_b_nb = Quad->x.w;

    dynamics(Quad,p_n,v_n,E,w_b_nb,&K[0][0],&K[0][1],&K[0][2],&K[0][3],pwm);

    dynamics(Quad,vec3_add(p_n,vec3_mult_f(K[0][0], t_step/2)),
             vec3_add(v_n,vec3_mult_f(K[0][1], t_step/2)),
             vec3_add(E,vec3_mult_f(K[0][2], t_step/2)),
             vec3_add(w_b_nb,vec3_mult_f(K[0][3], t_step/2)),&K[1][0],&K[1][1],&K[1][2],&K[1][3],pwm);

    dynamics(Quad,vec3_add(p_n,vec3_mult_f(K[1][0], t_step/2)),
             vec3_add(v_n,vec3_mult_f(K[1][1], t_step/2)),
             vec3_add(E,vec3_mult_f(K[1][2], t_step/2)),
             vec3_add(w_b_nb,vec3_mult_f(K[1][3], t_step/2)),&K[2][0],&K[2][1],&K[2][2],&K[2][3],pwm);

    dynamics(Quad,vec3_add(p_n,vec3_mult_f(K[2][0], t_step)),
             vec3_add(v_n,vec3_mult_f(K[2][1], t_step)),
             vec3_add(E,vec3_mult_f(K[2][2], t_step)),
             vec3_add(w_b_nb,vec3_mult_f(K[2][3], t_step)),&K[3][0],&K[3][1],&K[3][2],&K[3][3],pwm);

	vec3 H1 = vec3_add(vec3_add(K[0][0],vec3_mult_f(K[1][0],2)),vec3_add(vec3_mult_f(K[2][0],2),K[3][0]));
	vec3 H2 = vec3_add(vec3_add(K[0][1],vec3_mult_f(K[1][1],2)),vec3_add(vec3_mult_f(K[2][1],2),K[3][1]));
	vec3 H3 = vec3_add(vec3_add(K[0][2],vec3_mult_f(K[1][2],2)),vec3_add(vec3_mult_f(K[2][2],2),K[3][2]));
	vec3 H4 = vec3_add(vec3_add(K[0][3],vec3_mult_f(K[1][3],2)),vec3_add(vec3_mult_f(K[2][3],2),K[3][3]));

    Quad->x.p = vec3_add(p_n, vec3_mult_f(H1, t_step/6));
    Quad->x.v = vec3_add(v_n, vec3_mult_f(H2, t_step/6));
    Quad->x.E = vec3_add(E, vec3_mult_f(H3, t_step/6));
    Quad->x.w = vec3_add(w_b_nb, vec3_mult_f(H4, t_step/6));

}

void update_timers(timer timers[],int amount_timers, float t_step)
{
	int i;
	for(i = 0; i < amount_timers;++i){
		if(timers[i].active){
			timers[i].time += t_step;
			if(timers[i].time > timers[i].period)
				timers[i].interrupt(&timers[i]);
		}
	}

}

void tim1_interrupt_handler(timer *tim)
{
	tim->time = 0;
	flags.timer1_overflow = 1;
}

void tim2_interrupt_handler(timer *tim)
{
	tim->time = 0;
	flags.timer2_overflow = 1;
}

vec4 forces_to_pwm(quadrotor *Quad, vec4 f_u)
{
    parameters Quadparams = Quad->params.st;

    mat4 M = {{{Quadparams.C_T,Quadparams.C_T,Quadparams.C_T,Quadparams.C_T},
               {0,-Quadparams.C_T*Quadparams.l,0,Quadparams.C_T*Quadparams.l},
               {Quadparams.C_T*Quadparams.l,0,-Quadparams.C_T*Quadparams.l,0},
               {Quadparams.C_Q,-Quadparams.C_Q,Quadparams.C_Q,-Quadparams.C_Q}}};
    vec4 U = mat4_mult_vec4(mat4_inverse(M),f_u);
    vec4 pwm;
    int i;
    float min_U = 0.0f;
    for(i = 0; i < 4; ++i){
        min_U = U.a[i] < min_U ? U.a[i] : min_U;
    }
    if(min_U < 0.0f){
        for(i = 0;i < 4;++i){
            U.a[i] -= min_U;
        }
    }
    min_U = min_U < 0.0f ? min_U : 0.0f;
    for(i = 0;i < 4;++i){
        U.a[i] -= min_U;
    }
    float max_U = 0.0f;
    for(i = 0; i < 4; ++i){
        max_U = U.a[i] > max_U ? U.a[i] : max_U;
    }
    float scale_factor = max_U < Quadparams.u_max*Quadparams.u_max ? Quadparams.u_max*Quadparams.u_max : max_U;
    for(i = 0; i < 4;++i){
        pwm.a[i] = U.a[i]/scale_factor;
    }
    return pwm;
}
