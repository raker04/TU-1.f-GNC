#ifndef VECTOR_UTIL_H
#define VECTOR_UTIL_H

#include <math.h>

// function prototype
float dotProduct(float v_A[3], float v_B[3]); // res = dot(v_A, v_B)
float calcNorm(float v[3]); // res = norm(v);
void crossProduct(float v_A[3], float v_B[3], float res[3]); // res = v_A x v_B
void matrixVecMult(float T[3][3], float b[3], float res[3]); // res = T * b
void matrixTranspose(float A[3][3], float res[3][3]); // res = A^T

// function definitions
float dotProduct(float v_A[3], float v_B[3]) {
  return v_A[0]*v_B[0] + v_A[1]*v_B[1] + v_A[2]*v_B[2];
}

float calcNorm(float v[3]) {
  return sqrt(dotProduct(v, v));
}

void crossProduct(float v_A[3], float v_B[3], float res[3]) {
  res[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
  res[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
  res[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
  return;
}

void matrixVecMult(float T[3][3], float b[3], float res[3]) {
  res[0] = T[0][0]*b[0] + T[0][1]*b[1] + T[0][2]*b[2];
  res[1] = T[1][0]*b[0] + T[1][1]*b[1] + T[1][2]*b[2];
  res[2] = T[2][0]*b[0] + T[2][1]*b[1] + T[2][2]*b[2];
  return;
}

void matrixTranspose(float A[3][3], float res[3][3]) {
  res[0][0] = A[0][0];
  res[0][1] = A[1][0];
  res[0][2] = A[2][0];
  res[1][0] = A[0][1];
  res[1][1] = A[1][1];
  res[1][2] = A[2][1];
  res[2][0] = A[0][2];
  res[2][1] = A[1][2];
  res[2][2] = A[2][2];
  return;
}

#endif
