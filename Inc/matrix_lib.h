#ifndef MATLIB_H
#define MATLIB_H
#include <math.h>
#include <stdint.h>

float* mat_mult(const float *A, const float *B, float *C, const uint8_t n1, const uint8_t m1, const uint8_t n2, const uint8_t m2);
float* mat_square_mult(const float *A, const float *B, float *C, const uint8_t n);
float* mat_add(const float *A, const float *B, float *C, const uint8_t n, const uint8_t m);
float* mat_sub(const float *A, const float *B, float *C, const uint8_t n, const uint8_t m);
float* mat_square_add(const float *A, const float *B, float *C, const uint8_t n);
float* mat_square_sub(const float *A, const float *B, float *C, const uint8_t n);
float* mat_transpose(const float *A, float *B, const uint8_t n, const uint8_t m);
float* mat_square_transpose(const float *A, float *B, const uint8_t n);
float* mat_mult_f(float *A, float k,const uint8_t n, const uint8_t m);
float* mat_square_mult_f(float *A, float k, const uint8_t n);
//void mat_print(const float *A, const uint8_t n, const uint8_t m);
//Modified from wikipedia....
uint8_t LUPDecompose(float *A, int8_t N, float Tol, int32_t *P);
//Modified from wikipedia...
void LUPInvert(float *A, int32_t *P, int8_t N, float *IA);
void mat_square_inv(float *A, int32_t *P, float* IA, int8_t n);

#endif
