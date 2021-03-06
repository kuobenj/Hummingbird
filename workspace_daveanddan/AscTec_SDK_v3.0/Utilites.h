#ifndef UTILITES_H_
#define UTILITES_H_



//Done
void vector_add(int n, float *A, float *B, float *C);
void vector_multiply( int n, int m, float *A, float *x, float *b);
void vector_copy(int n, float* A, float* B);
void vector_scalar_mult(int n, float *A, float s, float *B);
float angle_diff(float a1, float a2);
float determinant(float *mat,int n);
void make_identity(int n, float *I);
void matrix_transpose(int n , int m, float *A,  float *B);
void matrix_scalar_mult(int n, int m, float *A, float s, float *B);
void matrix_add(int n, int m, float *A, float *B, float *C);
void matrix_multiply( int m, int p, int n , float *A, float *B,  float *C);
void cofactors(float *A,float *B, int n);
void inverse(int n, float *A, float *inv);

#endif 
