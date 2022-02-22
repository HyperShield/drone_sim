#include "matrix_lib.h"
#include <stdio.h>
float* mat_mult(const float *A, const float *B, float *C, const uint8_t n1, const uint8_t m1, const uint8_t n2, const uint8_t m2)
{
	uint8_t i,j,k;
	if(m1 == n2){
		float temp;
		for(i = 0; i < n1; ++i){
			for(j = 0; j < m2; ++j){
				temp = 0;
				for(k = 0; k < m1; ++k){
					temp += A[m1*i + k]*B[k*m2 + j];
				}
				C[m2*i + j] = temp;
			}
		}
	}
	return C;
}
float* mat_square_mult(const float *A, const float *B, float *C, const uint8_t n)
{
	return mat_mult(A, B, C, n, n, n, n);
}

float* mat_add(const float *A, const float *B, float *C, const uint8_t n, const uint8_t m)
{
	uint8_t i,j;
	for(i = 0; i < n; ++i){
		for(j = 0; j < m; ++j){
			C[i*m + j] = A[i*m + j]+B[i*m + j];
		}
	}
	return C;
}
float* mat_sub(const float *A, const float *B, float *C, const uint8_t n, const uint8_t m)
{
	uint8_t i,j;
	for(i = 0; i < n; ++i){
		for(j = 0; j < m; ++j){
			C[i*m + j] = A[i*m + j] - B[i*m + j];
		}
	}
	return C;
}
float* mat_square_add(const float *A, const float *B, float *C, const uint8_t n)
{
	return mat_add(A, B, C, n, n);
}
float* mat_square_sub(const float *A, const float *B, float *C, const uint8_t n)
{
	return mat_sub(A, B, C, n, n);
}
float* mat_transpose(const float *A, float *B, const uint8_t n, const uint8_t m)
{
	uint8_t i,j;
	for(i = 0; i < n; ++i){
		for(j = 0; j < m; ++j){
			B[j*n + i] = A[i*m + j];
		}
	}
	return B;
}
float* mat_square_transpose(const float *A, float *B, const uint8_t n)
{
	return mat_transpose(A, B, n, n);
}
float* mat_mult_f(float *A, float k, const uint8_t n, const uint8_t m)
{
	uint8_t i,j;
	for(i = 0; i < n; ++i){
		for(j = 0; j < m; ++j){
			A[i*m + j] *= k;
		}
	}
	return A;
}
float* mat_square_mult_f(float *A, float k, const uint8_t n)
{
	return mat_mult_f(A, k, n, n);
}
/*void mat_print(const float *A, const uint8_t n, const uint8_t m)
{
	uint8_t i,j;
	for(i = 0; i < n; ++i){
		for(j = 0; j < m; ++j){
			printf("%f,\t",A[i*m + j]);
		}
		printf("\n");
	}
}*/
//Modified from wikipedia....
uint8_t LUPDecompose(float *A, int8_t N, float Tol, int32_t *P) {

    int32_t i, j, k, imax;
    float maxA, absA;

    for (i = 0; i <= N; i++)
        P[i] = i; //Unit permutation matrix, P[N] initialized with N

    for (i = 0; i < N; i++) {
        maxA = 0.0;
        imax = i;

        for (k = i; k < N; k++)
            if ((absA = fabs(A[k*N+i])) > maxA) {
                maxA = absA;
                imax = k;
            }

        if (maxA < Tol) return 0; //failure, matrix is degenerate

        if (imax != i) {
            //pivoting P
            j = P[i];
            P[i] = P[imax];
            P[imax] = j;

            //pivoting rows of A
            float temp;
            for(k = 0; k < N; ++k){
            	temp = A[i*N + k];
            	A[i*N + k] = A[imax*N + k];
            	A[imax*N + k] = temp;
            }

            //counting pivots starting from N (for determinant)
            P[N]++;
        }

        for (j = i + 1; j < N; j++) {
            A[j*N + i] /= A[i*N + i];

            for (k = i + 1; k < N; k++)
                A[j*N + k] -= A[j*N + i] * A[i*N + k];
        }
    }

    return 1;  //decomposition done
}
//Modified from wikipedia...
void LUPInvert(float *A, int32_t *P, int8_t N, float *IA)
{
	int32_t i, j, k;
    for (j = 0; j < N; j++) {
        for (i = 0; i < N; i++) {
            IA[i*N + j] = P[i] == j ? 1.0f : 0.0f;

            for (k = 0; k < i; k++)
                IA[i*N + j] -= A[i*N + k] * IA[k*N + j];
        }

        for (i = N - 1; i >= 0; i--) {
            for (k = i + 1; k < N; k++)
                IA[i*N + j] -= A[i*N + k] * IA[k*N + j];

            IA[i*N + j] /= A[i*N + i];
        }
    }
}
void mat_square_inv(float *A, int32_t *P, float* IA, int8_t n)
{
	LUPDecompose(A, n, 0.01, P);
	LUPInvert(A, P, n, IA);
}
