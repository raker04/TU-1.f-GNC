#ifndef VECTOR_UTIL_H
#define VECTOR_UTIL_H

#include <math.h>

// function prototype
float dotProduct(float v_A[3], float v_B[3]); // res = dot(v_A, v_B)
float calcNorm(float v[3]); // res = norm(v);
void crossProduct(float v_A[3], float v_B[3], float res[3]); // res = v_A x v_B
void matrixVecMult(float T[3][3], float b[3], float res[3]); // res = T * b
void matrixTranspose(float A[3][3], float res[3][3]); // res = A^T
void matrixInverse3(float A[3][3], float res[3][3]); // res = A^-1
void matrixInverse2(float A[2][2], float res[2][2]); // res = A^-1 but size is now 2
void matrixSub(float A[3][3], float res[2][2], int x, int y); // res = M_x,y but flipped if x==1 or y==1
float matrixDet3(float A[3][3]); //res = det(A)
float matrixDet2(float A[2][2]); //res = det(A) but size is now 2
void signalProcessFilter(float data[3],float *processed[3],float *diff[3],float *sample,int samplesize,int *samplecount,float time,float prevdata[3],float tempmatrix1[3][3],float tempmatrix2[3][3],float tempmatrix3[2][2],float tempmatrix4[2][3],float alpha); // grand design.

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

void matrixInverse3(float A[3][3], float res[3][3]) {
  float temp_norm = matrixDet3(A);
  float tempmat[2][2];
  for(int i = 0;i < 3;i++){
    for(int j = 0;j < 3;j++){
      matrixSub(A,tempmat,j,i);
      res[i][j] = matrixDet2(tempmat)/temp_norm;
    }
  }
  return;
}

void matrixInverse2(float A[2][2], float res[2][2]) {
  float temp_norm = matrixDet2(A);
  res[0][0] = A[1][1]/temp_norm;
  res[0][1] = -A[0][1]/temp_norm;
  res[1][0] = -A[1][0]/temp_norm;
  res[1][1] = A[0][0]/temp_norm;
  return;
}

void matrixSub(float A[3][3], float res[2][2], int x, int y){
  int targetx[2],targety[2];
  if(x == 0){
    targetx[0] = 1;
    targetx[1] = 2;
  }else if(x == 1){
    targetx[0] = 2;
    targetx[1] = 0;
  }else if(x == 2){
    targetx[0] = 0;
    targetx[1] = 1;
  }else{
    return;
  }
  if(y == 0){
    targety[0] = 1;
    targety[1] = 2;
  }else if(y == 1){
    targety[0] = 2;
    targety[1] = 0;
  }else if(y == 2){
    targety[0] = 0;
    targety[1] = 1;
  }else{
    return;
  }
  for(int i = 0;i < 2;i++){
    for(int j = 0;j < 2;j++){
      res[i][j] = A[targetx[i]][targety[j]];
    }
  }
  return;
}

float matrixDet3(float A[3][3]){
  float res = 0;
  float tempmat[2][2];
  for(int i = 0;i < 3;i++){
    matrixSub(A,tempmat,0,i);
    res += A[0][i]*matrixDet2(tempmat);
  }

  return res;
}

float matrixDet2(float A[2][2]){
  return A[0][0]*A[1][1] - A[1][0]*A[0][1];
}

void signalProcessFilter(float data[3],float processed[3],float diff[3],float *sample[4],int samplesize,int samplecount,float time,float prevprocessed[3],float tempmatrix1[3][3],float tempmatrix2[3][3],float tempmatrix3[2][2],float tempmatrix4[2][3],float alpha){
  int legacycount;
  float legacytime,legacydata[3];
  float resultmatrix1[3][3], resultmatrix2[2][3];
  if (samplecount == samplesize - 1){
    legacycount = 0;
  }else{
    legacycount = samplecount+1;
  }
  legacytime = sample[0][legacycount];
  legacydata[0] = sample[1][legacycount];
  legacydata[1] = sample[2][legacycount];
  legacydata[2] = sample[3][legacycount];
  sample[0][samplecount] = time;
  sample[1][samplecount] = data[0];
  sample[2][samplecount] = data[1];
  sample[3][samplecount] = data[2];
  for(int i = 0;i < 3;i++){
    for(int j = 0;j < 3;j++){
      tempmatrix1[i][j] += pow(time,i+j)-pow(legacytime,i+j);
      tempmatrix2[i][j] += pow(time,i)*data[j]-pow(legacytime,i)*legacydata[j];
    }
  }
  for(int i = 0;i < 2;i++){
    for(int j = 0;j < 2;j++){
      tempmatrix3[i][j] += pow(time,i+j)-pow(legacytime,i+j);
    }
  }
  for(int i = 0;i < 2;i++){
    for(int j = 0;j < 3;j++){
      tempmatrix4[i][j] += pow(time,i)*data[j]-pow(legacytime,i)*legacydata[j];
    }
  }
  for(int i = 0;i < 3;i++){
    for(int j = 0;j < 3;j++){
      resultmatrix1[i][j] = 0;
      for(int k = 0;k < 3;k++){
        resultmatrix1[i][j] += tempmatrix1[i][k]*tempmatrix2[k][j];
      }
    }
  }
  for(int i = 0;i < 2;i++){
    for(int j = 0;j < 3;j++){
      resultmatrix1[i][j] = 0;
      for(int k = 0;k < 2;k++){
        resultmatrix2[i][j] += tempmatrix3[i][k]*tempmatrix4[k][j];
      }
    }
  }
  for(int i = 0;i < 3;i++){
    processed[i] = 0;
    for(int j = 0;j < 3;j++){
      processed[i] += (1-alpha)*resultmatrix1[j][i]*pow(time,j);
    }
    processed[i] += alpha*prevprocessed[i];
  }
  for(int i = 0;i < 3;i++){
    diff[i] = resultmatrix2[1][i];
  }
}
#endif
