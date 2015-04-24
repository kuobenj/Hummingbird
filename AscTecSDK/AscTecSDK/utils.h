#ifndef UTILS_H_
#define UTILS_H_

//#include "mymath.h"

#define g_tolerance 0.000001 // helper variable

typedef struct _Quaternion {
	float w;
	float x;
	float y;
	float z;
} Quaternion;

typedef struct _EulerAngles {
	float phi;
	float theta;
	float psi;
} EulerAngles;

unsigned short throttle_scaling(unsigned short stick, float M, float X1);

void euler2quat(EulerAngles* angles, Quaternion* q);
void quat2euler(Quaternion* q, EulerAngles* angles);
void quaternionmult(Quaternion* q1, Quaternion* q2, Quaternion* qout);
void quaternionNormalize(Quaternion* q);

void euler2rotmat(float phi, float theta, float psi, float *R);
void S_from_euler(float phi, float theta, float psi, float *S);
void matrix_transpose(int m , int n, float* A,  float* B);
void matrix_multiply( int m, int p, int n , float *A, float *B,  float *C);
void matrix_copy(int m, int n, float* A, float* B );
void matrix_add(int n, int m, float *A, float *B); // in place!
void matrix_subtract(int n, int m, float *A, float *B); // in place!
void matrix_scalar_mult(int n, int m, float *A, float s); // in place!
void matrix_scalar_add(int n, int m, float *A, float s); // in place!
float angle_diff(float a1, float a2);
void unwedge(float* A, float* v );

#endif /* UTILS_H_ */
