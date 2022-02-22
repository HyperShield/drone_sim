#include <stdio.h>
#include "quadrotor.h"

void quadrotor_ctor_def(quadrotor *Q)
{
    Q->params.st.m = 1.27;
    Q->params.st.Jx = 0.0434;
    Q->params.st.Jy = 0.0434;
    Q->params.st.Jz = 0.0705;
    Q->params.st.C_T = 1.5e-5;
    Q->params.st.C_Q = 1.89981e-7;
    Q->params.st.l = 0.149352;
    Q->params.st.h = 0;
    Q->params.st.Ib = 4.0675e-5;
    Q->params.st.u_max = 600;
    Q->params.st.theta0 = 0.49;
    Q->params.st.theta1 = -0.33;
    Q->params.st.a0 = 5.7;
    Q->params.st.rho = 1.225;
    Q->params.st.r = 0.128016;
    Q->params.st.c0 = 0.027432;

    Q->x.p = (vec3){0,0,0};
    Q->x.v = (vec3){0,0,0};
    Q->x.E = (vec3){0,0,0};
    Q->x.w = (vec3){0,0,0};
}

void quadrotor_ctor(quadrotor *Q, float *params)
{
    int i;
    for(i = 0; i < 14;++i){
        Q->params.list[i] = params[i];
    }
}
