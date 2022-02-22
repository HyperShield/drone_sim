#include "mathlib.h"
#include <stdio.h>
#include <math.h>

vec3 vec3_add(vec3 u, vec3 v)
{
	u.x += v.x;
	u.y += v.y;
	u.z += v.z;
	return u;
}

vec3 vec3_sub(vec3 u, vec3 v)
{
	u.x -= v.x;
	u.y -= v.y;
	u.z -= v.z;
	return u;
}

void vec3_print(vec3 v)
{
	printf("%f %f %f\n", v.x, v.y, v.z);
}

vec3 vec3_mult_f(vec3 v, float a)
{
	v.x *= a;
	v.y *= a;
	v.z *= a;
	return v;
}

vec3 vec3_mult_d(vec3 v, int32_t a)
{
	return vec3_mult_f(v,(float)a);
}

float vec3_dot(vec3 u, vec3 v)
{
	return u.x*v.x + u.y*v.y + u.z*v.z;
}

vec3 vec3_cross(vec3 u, vec3 v)
{
	vec3 w;
	w.x = u.y*v.z - v.y*u.z;
	w.y = -(u.x*v.z - v.x*u.z);
	w.z = u.x*v.y - v.x*u.y;
	return w;
}

float vec3_norm(vec3 v)
{
	return sqrt(vec3_dot(v,v));
}

vec3 vec3_dir(vec3 v,float tol)
{
	float mag = vec3_norm(v);
	if(mag > tol){
		return vec3_mult_f(v,1/mag);
	} else {
		return v;
	}
}

quat vec3_to_quat(vec3 v)
{
	quat q;
	q.w = 0;
	q.x = v.x;
	q.y = v.y;
	q.z = v.z;
	return q;
}

vec3 quat_to_vec3(quat q)
{
	vec3 v;
	v.x = q.x;
	v.y = q.y;
	v.z = q.z;
	return v;
}

quat quat_prod(quat q1, quat q2)
{
	quat q3;
	q3.w = q1.w*q2.w - vec3_dot(quat_to_vec3(q1), quat_to_vec3(q2));
	vec3 cr = vec3_cross(quat_to_vec3(q1), quat_to_vec3(q2));
	q3.x = q1.w*q2.x + q2.w*q1.x + cr.x;
	q3.y = q1.w*q2.y + q2.w*q1.y + cr.y;
	q3.z = q1.w*q2.z + q2.w*q1.z + cr.z;
	return q3;
}

quat quat_conj(quat q1)
{
	q1.w = q1.w;
	q1.x = -q1.x;
	q1.y = -q1.y;
	q1.z = -q1.z;
	return q1;
}

vec3 quat_rot(quat q, vec3 v)
{
	return quat_to_vec3(quat_prod(q, quat_prod(vec3_to_quat(v), quat_conj(q))));
}

float quat_norm(quat q)
{
	return sqrt(q.w*q.w + vec3_dot(quat_to_vec3(q), quat_to_vec3(q)));
}

quat quat_mult_f(quat q1, float a)
{
	q1.w *= a;
	q1.x *= a;
	q1.y *= a;
	q1.z *= a;
	return q1;
}

quat quat_add(quat q1, quat q2)
{
	q1.w += q2.w;
	q1.x += q2.x;
	q1.y += q2.y;
	q1.z += q2.z;
	return q1;
}

quat quat_normalize(quat q)
{
	float n = quat_norm(q);
	return quat_mult_f(q, 1/n);
}

vec3 mat3_mult_vec3(mat3 M,vec3 v)
{
	vec3 u;
	u.x = M.a[0][0]*v.x + M.a[0][1]*v.y + M.a[0][2]*v.z;
	u.y = M.a[1][0]*v.x + M.a[1][1]*v.y + M.a[1][2]*v.z;
	u.z = M.a[2][0]*v.x + M.a[2][1]*v.y + M.a[2][2]*v.z;
	return u;
}
mat3 mat3_mult_mat3(mat3 A, mat3 B)
{
	mat3 C = {{{0,0,0},{0,0,0},{0,0,0}}};
	int i,j,k;
	for(i = 0; i < 3; ++i){
		for(j = 0; j < 3;++j){
			for(k = 0; k < 3; ++k){
				C.a[i][j] += A.a[i][k]*B.a[k][j];
			}
		}
	}
	return C;
}
float mat3_det(mat3 M)
{
	float det = M.a[0][0]*(M.a[1][1]*M.a[2][2] - M.a[2][1]*M.a[1][2])
			  - M.a[0][1]*(M.a[1][0]*M.a[2][2] - M.a[2][0]*M.a[1][2])
			  + M.a[0][2]*(M.a[1][0]*M.a[2][1] - M.a[2][0]*M.a[1][1]);
	return det;
}
mat3 mat3_inverse(mat3 M)
{
	float det = mat3_det(M);

	float A00 =  (M.a[1][1]*M.a[2][2] - M.a[2][1]*M.a[1][2]);
	float A01 = -(M.a[1][0]*M.a[2][2] - M.a[2][0]*M.a[1][2]);
	float A02 =  (M.a[1][0]*M.a[2][1] - M.a[2][0]*M.a[1][1]);

	float A10 = -(M.a[0][1]*M.a[2][2] - M.a[2][1]*M.a[0][2]);
	float A11 =  (M.a[0][0]*M.a[2][2] - M.a[2][0]*M.a[0][2]);
	float A12 = -(M.a[0][0]*M.a[2][1] - M.a[2][0]*M.a[0][1]);

	float A20 =  (M.a[0][1]*M.a[1][2] - M.a[1][1]*M.a[0][2]);
	float A21 = -(M.a[0][0]*M.a[1][2] - M.a[1][0]*M.a[0][2]);
	float A22 =  (M.a[0][0]*M.a[1][1] - M.a[1][0]*M.a[0][1]);

	mat3 Adj = {{{A00,A01,A02},{A10,A11,A12},{A20,A21,A22}}};
	Adj = mat3_transpose(Adj);
	return mat3_mult_f(Adj, 1/det);
}
mat3 mat3_transpose(mat3 M)
{
	mat3 C;
	int i,j;
	for(i = 0; i < 3; ++i){
		for(j = 0; j < 3; ++j){
			C.a[i][j] = M.a[j][i];
		}
	}
	return C;
}
mat3 mat3_mult_f(mat3 M, float a)
{
	int i,j;
	for(i = 0; i < 3; ++i){
		for(j = 0; j < 3; ++j){
			M.a[i][j] = a*M.a[i][j];
		}
	}
	return M;
}

vec3 quat_to_euler(quat q)
{
	vec3 E;
	E.x = atan2f(2*(q.w*q.x+q.y*q.z),(q.w*q.w + q.z*q.z - q.x*q.x - q.y*q.y));
	E.y = asinf(2*(q.w*q.y - q.x*q.z));
	E.z = atan2f(2*(q.w*q.z + q.x*q.y),q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
	return E;
}

mat4 mat4_inverse(mat4 M)
{
	mat3 M00 = {{{M.a[1][1],M.a[1][2],M.a[1][3]},
				 {M.a[2][1],M.a[2][2],M.a[2][3]},
				 {M.a[3][1],M.a[3][2],M.a[3][3]}}};
	float A00 =  mat3_det(M00);
	mat3 M01 = {{{M.a[1][0],M.a[1][2],M.a[1][3]},
				 {M.a[2][0],M.a[2][2],M.a[2][3]},
				 {M.a[3][0],M.a[3][2],M.a[3][3]}}};
	float A01 = -mat3_det(M01);
	mat3 M02 = {{{M.a[1][0],M.a[1][1],M.a[1][3]},
				 {M.a[2][0],M.a[2][1],M.a[2][3]},
				 {M.a[3][0],M.a[3][1],M.a[3][3]}}};
	float A02 =  mat3_det(M02);
	mat3 M03 = {{{M.a[1][0],M.a[1][1],M.a[1][2]},
				 {M.a[2][0],M.a[2][1],M.a[2][2]},
				 {M.a[3][0],M.a[3][1],M.a[3][2]}}};
	float A03 = -mat3_det(M03);

	mat3 M10 = {{{M.a[0][1],M.a[0][2],M.a[0][3]},
				 {M.a[2][1],M.a[2][2],M.a[2][3]},
				 {M.a[3][1],M.a[3][2],M.a[3][3]}}};
	float A10 = -mat3_det(M10);
	mat3 M11 = {{{M.a[0][0],M.a[0][2],M.a[0][3]},
				 {M.a[2][0],M.a[2][2],M.a[2][3]},
				 {M.a[3][0],M.a[3][2],M.a[3][3]}}};
	float A11 =  mat3_det(M11);
	mat3 M12 = {{{M.a[0][0],M.a[0][1],M.a[0][3]},
				 {M.a[2][0],M.a[2][1],M.a[2][3]},
				 {M.a[3][0],M.a[3][1],M.a[3][3]}}};
	float A12 = -mat3_det(M12);
	mat3 M13 = {{{M.a[0][0],M.a[0][1],M.a[0][2]},
				 {M.a[2][0],M.a[2][1],M.a[2][2]},
				 {M.a[3][0],M.a[3][1],M.a[3][2]}}};
	float A13 =  mat3_det(M13);

	mat3 M20 = {{{M.a[0][1],M.a[0][2],M.a[0][3]},
				 {M.a[1][1],M.a[1][2],M.a[1][3]},
				 {M.a[3][1],M.a[3][2],M.a[3][3]}}};
	float A20 =  mat3_det(M20);
	mat3 M21 = {{{M.a[0][0],M.a[0][2],M.a[0][3]},
				 {M.a[1][0],M.a[1][2],M.a[1][3]},
				 {M.a[3][0],M.a[3][2],M.a[3][3]}}};
	float A21 = -mat3_det(M21);
	mat3 M22 = {{{M.a[0][0],M.a[0][1],M.a[0][3]},
				 {M.a[1][0],M.a[1][1],M.a[1][3]},
				 {M.a[3][0],M.a[3][1],M.a[3][3]}}};
	float A22 =  mat3_det(M22);
	mat3 M23 = {{{M.a[0][0],M.a[0][1],M.a[0][2]},
				 {M.a[1][0],M.a[1][1],M.a[1][2]},
				 {M.a[3][0],M.a[3][1],M.a[3][2]}}};
	float A23 = -mat3_det(M23);

	mat3 M30 = {{{M.a[0][1],M.a[0][2],M.a[0][3]},
				 {M.a[1][1],M.a[1][2],M.a[1][3]},
				 {M.a[2][1],M.a[2][2],M.a[2][3]}}};
	float A30 = -mat3_det(M30);
	mat3 M31 = {{{M.a[0][0],M.a[0][2],M.a[0][3]},
				 {M.a[1][0],M.a[1][2],M.a[1][3]},
				 {M.a[2][0],M.a[2][2],M.a[2][3]}}};
	float A31 =  mat3_det(M31);
	mat3 M32 = {{{M.a[0][0],M.a[0][1],M.a[0][3]},
				 {M.a[1][0],M.a[1][1],M.a[1][3]},
				 {M.a[2][0],M.a[2][1],M.a[2][3]}}};
	float A32 = -mat3_det(M32);
	mat3 M33 = {{{M.a[0][0],M.a[0][1],M.a[0][2]},
				 {M.a[1][0],M.a[1][1],M.a[1][2]},
				 {M.a[2][0],M.a[2][1],M.a[2][2]}}};
	float A33 =  mat3_det(M33);

	float d = M.a[0][0]*A00 + M.a[0][1]*A01 + M.a[0][2]*A02 + M.a[0][3]*A03;

	return (mat4){{{A00/d,A10/d,A20/d,A30/d},
				   {A01/d,A11/d,A21/d,A31/d},
				   {A02/d,A12/d,A22/d,A32/d},
				   {A03/d,A13/d,A23/d,A33/d}}};
}

vec4 vec4_mult_f(vec4 v, float a)
{
	v.a[0] *= a;
	v.a[1] *= a;
	v.a[2] *= a;
	v.a[3] *= a;
	return v;
}
vec4 mat4_mult_vec4(mat4 M, vec4 v)
{
	vec4 u = {{0}};
	int i,j;
	for(i = 0; i < 4; ++i){
		for(j = 0; j < 4; ++j){
			u.a[i] += M.a[i][j]*v.a[j];
		}
	}
	return u;
}

mat3 Eul_to_R(vec3 E)
{
	float phi = E.x, theta = E.y, psi = E.z;
	mat3 R1 = {{{1,0,0},{0,cosf(phi),-sinf(phi)},{0,sinf(phi),cosf(phi)}}};
	mat3 R2 = {{{cosf(theta),0,sinf(theta)},{0,1,0},{-sinf(theta),0,cosf(theta)}}};
	mat3 R3 = {{{cosf(psi),-sinf(psi),0},{sinf(psi),cosf(psi),0},{0,0,1}}};
	return mat3_mult_mat3(R3,mat3_mult_mat3(R2,R1));
}
