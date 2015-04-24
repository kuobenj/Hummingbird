/*

Copyright (c) 2011, Ascending Technologies GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

 */


/*
//////////////////////////////
// miles additions
//////////////////////////////
//transpose matrix A (m x n)  to  B (n x m)
void matrix_transpose(int m , int n, float* A,  float* B) {
	int i,j;
	for(i=0;i<m;i++) {
		for(j=0;j<n;j++) {
			B[j*m+i] = A[i*n+j];
		}
	}
}

//multiply matrix A (m x p) by  B(p x n) , put result in C (m x n)
void matrix_multiply( int m, int p, int n , float *A, float *B,  float *C) {
	int i,j,k;
	for(i=0;i<m;i++) {		//each row in A
		for(j=0;j<n;j++){	//each column in B
			C[i*n+j] = 0;
			for(k=0;k<p;k++){//each element in row A & column B
				C[i*n+j] += A[i*p+k]*B[k*n+j];
			}
		}
	}
}

void matrix_copy(int m, int n, float* A, float* B ) {
	int i,j;
	for(i=0;i<m;i++) {
		for(j=0;j<n;j++) {
			B[i*n+j] = A[i*n+j];
		}
	}
}
// end additions  ////////////////////////////
*/
