#ifndef MATHLIB_H
#define MATHLIB_H
#include <stdint.h>
typedef struct vec3_st {
	float x;
	float y;
	float z;
}vec3;

typedef struct quat_st {
	float w;
	float x;
	float y;
	float z;
}quat;

typedef struct mat3_st {
	float a[3][3];
}mat3;

typedef struct mat4_st {
	float a[4][4];
}mat4;

typedef struct vec4_st {
	float a[4];
}vec4;

vec3 vec3_add(vec3 u, vec3 v);
vec3 vec3_sub(vec3 u, vec3 v);
void vec3_print(vec3 v);
vec3 vec3_mult_f(vec3 v, float a);
vec3 vec3_mult_d(vec3 v, int32_t a);
float vec3_dot(vec3 u, vec3 v);
vec3 vec3_cross(vec3 u, vec3 v);
float vec3_norm(vec3 v);
vec3 vec3_dir(vec3 v, float tol);

quat vec3_to_quat(vec3 v);
vec3 quat_to_vec3(quat q);
quat quat_prod(quat q1, quat q2);
quat quat_conj(quat q1);
vec3 quat_rot(quat q, vec3 v);
float quat_norm(quat q);
quat quat_add(quat q1, quat q2);
quat quat_mult_f(quat q1, float a);
quat quat_normalize(quat q);

vec3 mat3_mult_vec3(mat3 M,vec3 v);
mat3 mat3_mult_mat3(mat3 A, mat3 B);
mat3 mat3_inverse(mat3 M);
mat3 mat3_transpose(mat3 M);
mat3 mat3_mult_f(mat3 M, float a);
float mat3_det(mat3 M);

mat4 mat4_inverse(mat4 M);
vec4 vec4_mult_f(vec4 v, float a);
vec4 mat4_mult_vec4(mat4 M, vec4 v);

vec3 quat_to_euler(quat q);
mat3 Eul_to_R(vec3 E);
#endif
