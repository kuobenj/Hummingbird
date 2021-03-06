#include "Utilites.h"
#include "math.h"
#include "stdio.h"

float angle_diff(float a1, float a2)
{
    return fmodf((a1 - a2) + 3*M_PI, 2*M_PI) - M_PI;
	//return ((float)fmod((a1-a2) + 3*3.1416, 2*3.1416) - 3.1416);
	//return fmodf((a1-a2) + 3*3.1416, 2*3.1416);
}

//Done!
void vector_add(int n, float *A, float *B, float *C) {
	int i;
	for (i=0; i<n; i++) {
		C[i] = A[i] + B[i];
	}
}

//Done!
// A (n x m), B (n x m), C (n x m)
void matrix_add(int n, int m, float *A, float *B, float *C) {
	int i, j;
	for (i=0; i<n; i++) {
		for (j=0; j<m; j++) {
			C[i*m+j] = A[i*m+j] + B[i*m+j];
		}
	}
}

//Done!
// A (n x m), B (n x m)
void matrix_scalar_mult(int n, int m, float *A, float s, float *B) {
	int i, j;
	for (i=0; i<n; i++) {
		for (j=0; j<m; j++) {
			B[i*m+j] = A[i*m+j]*s;
		}
	}
}

//Done!
void vector_scalar_mult(int n, float *A, float s, float *B) {
	int i;
	for (i=0; i<n; i++) {
		B[i] = A[i]*s;
	}
}

//Done!
//transpose matrix A (n x m)  to  B (m x n)
void matrix_transpose(int n , int m, float *A,  float *B) {
	int i,j;
	for(i=0;i<n;i++) {
		for(j=0;j<m;j++) {
			B[j*n+i] = A[i*m+j];
		}
	}
}

//Done!
//multiply matrix A (m x p) by  B(p x n) , put result in C (m x n)
void matrix_multiply( int m, int p, int n , float *A, float *B,  float *C) {
	int i,j,k;
	for(i=0;i<m;i++){		  //each row in A
		for(j=0;j<n;j++){	  //each column in B
			C[i*n+j] = 0;
			for(k=0;k<p;k++){ //each element in row A & column B
				C[i*n+j] += A[i*p+k]*B[k*n+j];
			}
		}
	}
}

//DONE!
//vector multiplication: Ax = b, A (n x m), x (m), b (n)
void vector_multiply( int n, int m, float *A, float *x, float *b) {
	int i, j;
	for (i=0;i<n;i++){		// Each row in A
		b[i] = 0.0;
		for (j=0;j<m;j++){	// Each column in A
			b[i] += A[i*m+j]*x[j];
		}
	}
}

//DONE!
float determinant(float *mat,int n)
{
    int i,j,i_count,j_count, count=0;
    float array[(n-1)*(n-1)], det=0.0;
    if(n==1) return mat[0*0];
    if(n==2) return (mat[0]*mat[3] - mat[1]*mat[2]);

    for(count=0; count<n; count++)
    {
        i_count=0;
        for(i=1; i<n; i++)
        {
            j_count=0;
            for(j=0; j<n; j++)
            {
                if(j == count) {continue;}
		else{
               		array[i_count*(n-1)+j_count] = mat[i*n+j];
                	j_count++;
		}
            }
            i_count++;
        }
        det += pow(-1, count) * mat[0*n+count] * determinant(array,n-1);
    }
    return det;
}


//DONE!
void vector_copy(int n, float* A, float* B ) {
	int i;
	for(i=0;i<n;i++) {
		B[i] = A[i];
	}
}

//DONE!
void make_identity(int n, float *I){
	int i, j;
	for(i=0;i<n;i++){
		for(j=0;j<n;j++){
			I[i*n+j] = 0.0;
			if (i == j) {
				I[i*n+j] = 1.0;
			}
		}
	}
}

//Done!
void cofactors(float *A,float *B, int n) {
	float b[(n-1)*(n-1)];
 	int p, q, i, j,icount,jcount;
	if (n == 1){
		B[0] = 1.0;
	}
	else {
		for (q = 0; q < n; q++) { // iterate through rows
  			for (p = 0; p < n; p++) {	//iterate through columns
				icount = 0;			
				for (i = 0; i < n; i++) { //iterate through rows of matrix
					jcount = 0;
    					for (j = 0; j < n; j++) { //iterate through columns of matrix
						if (i != q && j != p) {
							b[icount*(n-1)+jcount] = A[i*n+j];
							jcount++;
						}
					}
					if (i != q){
						icount++;
					}
				}
				B[q*n+p] = powf(-1,q+p) * determinant(b, n-1);			
			}
		}
	}
}

//Done!
void inverse(int n, float *A, float *inv) {
	float b[n*n], fac[n*n], d;
	cofactors(A,fac,n);
	matrix_transpose(n, n, fac, b);

	d = determinant(A, n);
	matrix_scalar_mult(n, n, b, 1.0/d, inv);
}
