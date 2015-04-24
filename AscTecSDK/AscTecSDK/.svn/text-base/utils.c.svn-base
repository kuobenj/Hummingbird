#include "utils.h"
#include "math.h"


float angle_diff(float a1, float a2)
{
    return fmodf((a1 - a2) + 3*M_PI, 2*M_PI) - M_PI;
	//return ((float)fmod((a1-a2) + 3*3.1416, 2*3.1416) - 3.1416);
	//return fmodf((a1-a2) + 3*3.1416, 2*3.1416);
}

void unwedge(float* A, float* v) {
    // A must be 3x3 matrix
    // v must be 3x1 vector
    v[0] = A[2,1];
    v[1] = -A[2,0];
    v[2] = A[1,0];
}

void euler2quat(EulerAngles* angles, Quaternion* qout) {

	float phi2 = angles->phi/2.0;
	float theta2 = angles->theta/2.0;
	float psi2 = angles->psi/2.0;

	float sinphi = sin(phi2);
	float sintheta = sin(theta2);
	float sinpsi = sin(psi2);
	float cosphi = cos(phi2);
	float costheta = cos(theta2);
	float cospsi = cos(psi2);

	qout->w = cosphi * costheta * cospsi + sinphi * sintheta * sinpsi;
	qout->x = sinphi * costheta * cospsi - cosphi * sintheta * sinpsi;
	qout->y = cosphi * sintheta * cospsi + sinphi * costheta * sinpsi;
	qout->z = cosphi * costheta * sinpsi - sinphi * sintheta * cospsi;

	//Quaternion Qx = createQuaternion(cos(phi/2.0), sin(phi/2.0), 0 ,0);
	//Quaternion Qy = createQuaternion(cos(theta/2.0), 0, sin(theta/2.0), 0);
	//Quaternion Qz = createQuaternion(cos(psi/2.0), 0, 0, sin(psi/2.0));
	//Quaternion Q1 = quaternionmult(Qy, Qx);
	//Quaternion Q2 = quaternionmult(Qz, Q1);
	//return(Q2);
}

void quat2euler(Quaternion* q, EulerAngles* angles) {
	angles->phi = atan2(2.0*(q->w*q->x + q->y*q->z), 1.0-2.0*(q->x*q->x + q->z * q->z));
	angles->theta = asin(2.0*(q->w*q->y - q->x * q->z));
	angles->psi = atan2(2.0*(q->w * q->z + q->x * q->y), 1.0-2.0*(q->y * q->y + q->z * q->z));
}


void quaternionmult(Quaternion* q1, Quaternion* q2, Quaternion* qout) {
	qout->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
	qout->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
	qout->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
	qout->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void quaternionNormalize(Quaternion* q) {
	// Don't normalize if we don't have to
	float mag2 = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
	if (fabs(mag2) > g_tolerance && fabs(mag2 - 1.0f) > g_tolerance) {
		float mag = sqrt(mag2);
		q->w /= mag;
		q->x /= mag;
		q->y /= mag;
		q->z /= mag;
	}
}

void euler2rotmat(float phi, float theta, float psi, float *R) {
    // Compute rotation matrix R01, rotation from World to Body frame
    
	float sphi = sin(phi);
	float stheta = sin(theta);
	float spsi = sin(psi);
	float cphi = cos(phi);
	float ctheta = cos(theta);
	float cpsi = cos(psi);

	// ROW ORDER?
	R[0] = ctheta * cpsi;
	R[1] = ctheta * spsi;
	R[2] = -stheta;
	R[3] = sphi*stheta*cpsi - cphi*spsi;
	R[4] = sphi*stheta*spsi + cphi*cpsi;
	R[5] = sphi*ctheta;
	R[6] = cphi*stheta*cpsi + sphi*spsi;
	R[7] = cphi*stheta*spsi - sphi*cpsi;
	R[8] = cphi*ctheta;
}
void S_from_euler(float phi, float theta, float psi, float *S) {
	float sphi = sin(phi);
	float cphi = cos(phi);
	float ctheta = cos(theta);
	float ttheta = tan(theta);

	S[0] = 1.0;
	S[1] = sphi*ttheta;
	S[2] = cphi*ttheta;
	S[3] = 0;
	S[4] = cphi;
	S[5] = -sphi;
	S[6] = 0;
	S[7] = sphi/ctheta;
	S[8] = cphi/ctheta;
}

//transpose matrix A (m x n)  to  B (n x m)
void matrix_transpose(int m , int n, float* A,  float* B) {
	int i,j;
	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
			B[j*m+i] = A[i*n+j];
}

//multiply matrix A (m x p) by  B(p x n) , put result in C (m x n)
void matrix_multiply( int m, int p, int n , float *A, float *B,  float *C) {
	int i,j,k;
	for(i=0;i<m;i++)		//each row in A
		for(j=0;j<n;j++){	//each column in B
			C[i*n+j] = 0;
			for(k=0;k<p;k++){//each element in row A & column B
				C[i*n+j] += A[i*p+k]*B[k*n+j];
				//printf("i=%d j=%d k=%d a=%f b=%f c=%f ",i,j,k,(double)(A[i*p+k]),(double)(B[k*n+j]),(double)(C[i*n+j]));
			}
			//printf("\n");
		}
};


void matrix_copy(int m, int n, float* A, float* B ) {
	int i,j;
	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
			B[i*n+j] = A[i*n+j];
}

void matrix_scalar_mult(int n, int m, float *A, float s) { // in place!
	int i, j;
	for (i=0; i<n; i++) {
		for (j=0; j<m; j++) {
			A[i*m + j] = A[i*m + j]*s;
		}
	}
}
void matrix_add(int n, int m, float *A, float *B) { // in place!  overwrites A!
	int i, j;
	for (i=0; i<n; i++) {
		for (j=0; j<m; j++) {
			A[i*m + j] = A[i*m + j] + B[i*m + j];
		}
	}
}

void matrix_subtract(int n, int m, float *A, float *B) { // in place!  overwrites A!
	int i, j;
	for (i=0; i<n; i++) {
		for (j=0; j<m; j++) {
			A[i*m + j] = A[i*m + j] - B[i*m + j];
		}
	}
}

void matrix_scalar_add(int n, int m, float *A, float s) { // in place!
	int i, j;
	for (i=0; i<n; i++) {
		for (j=0; j<m; j++) {
			A[i*m + j] = A[i*m + j] + s;
		}
	}
}

unsigned short throttle_scaling(unsigned short stick, float M, float X1) {
	float b=(1.0-M)*2047.5;
	float tmp=(float)stick;
	return((unsigned short)(tanh(3.0*tmp/(40.95*X1))*(M*tmp+b)));
}






