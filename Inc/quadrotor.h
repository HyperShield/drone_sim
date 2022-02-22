#ifndef QUADROTOR_H
#define QUADROTOR_H

#include "state_estimation.h"

typedef struct parameters_st{
    float m;
    float Jx;
    float Jy;
    float Jz;
    float C_T;
    float C_Q;
    float l;
    float h;
    float Ib;
    float u_max;
    float theta0;
    float theta1;
    float a0;
    float rho;
    float r;
    float c0;
}parameters;

typedef struct quadrotor_st{
    union {
        float list[16];
        parameters st;
    }params;
    state_vec x;
}quadrotor;

void quadrotor_ctor_def(quadrotor *Q);

void quadrotor_ctor(quadrotor *Q, float *params);

#endif // QUADROTOR_H
