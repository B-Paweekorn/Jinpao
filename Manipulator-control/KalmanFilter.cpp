#include <stdint.h>
#include "Arduino.h"
/*
 * kalmanfilter.cpp
 *
 *  Created on: 15 เม.ย. 2565
 *      Author: weera
 */

#include <KalmanFilter.h>

// KalmanFilter::KalmanFilter(float *_A_data, float *_B_data, float *_C_data, float *_Q_data, float *_R_data) {

//   matrix buf1(4, 1);
//   predictX_old = buf1;
//   matrix buf2(4, 4);
//   P_old = buf2;
//   float buf_val[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
//   matrix buf3(4, 4, buf_val);
//   I33 = buf3;
// }

KalmanFilter::KalmanFilter(float *_A_data, float *_B_data, float *_C_data, float *_Q_data, float *_R_data) {

  //Init Kalman Matrix
  matrix A_init(4, 4, _A_data);
  A = A_init;
  matrix B_init(4, 1, _B_data);
  B = B_init;
  matrix C_init(1, 4, _C_data);
  C = C_init;
  matrix D_init(4, 1);
  D = D_init;
  matrix R_init(1, 1, _R_data);
  R = R_init;
  matrix G_init(4, 1, G_data);
  G = G_init;
  matrix Q_init(1, 1, _Q_data);
  Q = Q_init;

  matrix buf1(4, 1);
  predictX_old = buf1;
  matrix buf2(4, 4);
  P_old = buf2;
  float buf_val[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  matrix buf3(4, 4, buf_val);
  I33 = buf3;
}

void KalmanFilter::init() {

  float P_val[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  matrix P_init(4, 4, P_val);
  P = P_init;

  float U_val[1] = { 0 };
  matrix U_init(1, 1, U_val);
  U = U_init;

  float Y_val[1] = { 0 };
  matrix Y_init(1, 1, Y_val);
  Y = Y_init;

  float gainK_val[4] = { 0, 0, 0, 0 };
  matrix gainK_init(4, 1, gainK_val);
  gainK = gainK_init;
}

void KalmanFilter::run() {
  doKalman_gain();
  doPredict_y();
  doCorrect_p();
  doCorrect();
  // Correct
  doPredict_x();
  doPredict_p();
  doResult();
  // Predict
  predictX_old = predictX_new;
  P_old = P_new;
  //update
}

void KalmanFilter::doKalman_gain() {
  matrix C_tra = C.transpose();
  matrix buf2 = ((C * P_old) * C_tra);
  matrix buf2_5 = buf2 + R;
  gainK = (P_old * C_tra).gain(1 / (buf2_5.data[0][0]));
}

void KalmanFilter::doPredict_y() {
  matrix buf = C * predictX_old;
  matrix buf2 = D * U;
  errorY = (Y - buf) + buf2;
}

void KalmanFilter::doCorrect_p() {
  matrix buf = gainK * C;
  P = (I33 - buf) * P_old;
  //P_old = (I33-(gainK*C))*P_old;
}

void KalmanFilter::doCorrect() {
  matrix buf1 = (gainK * errorY);
  predictX = buf1 + predictX_old;
}

void KalmanFilter::doPredict_x() {
  matrix buf = B * U;
  predictX_new = (A * predictX) + buf;
  //predictX_new = (A*predictX)+(B*U);
}

void KalmanFilter::doPredict_p() {
  matrix A_tran = A.transpose();
  matrix G_tran = G.transpose();
  matrix buf = (G * Q);
  buf = buf * G_tran;
  matrix buf1 = (A * P) * A_tran;
  P_new = buf1 + buf;
  //P_new = ((A*P)*A.transpose())+((G*Q)*G.transpose());
}

void KalmanFilter::doResult() {
  matrix buf = D * U;
  resultY = (C * predictX) + buf;
  //resultY = (C*predictX)+(D*U);
  resultX = predictX;
}

// float KalmanFilter::EstimateSpeed(double _measureRad, float Vin) {

//   //Update Observer and Command
//   measureRad = _measureRad;
//   updated_U[0] = Vin;
//   updated_Ymeas[0] = measureRad;
//   U.read(updated_U);
//   Y.read(updated_Ymeas);

//   //Kalmanfilter
//   run();



//   //Estimate Velocity
//   // estimateVel = resultX.data[1][0];

//   //Estimate State
//   estimateState[0] = resultX.data[0][0];
//   estimateState[1] = resultX.data[1][0];
//   estimateState[2] = resultX.data[2][0];
//   estimateState[3] = resultX.data[3][0];

//   return estimateState;
// }

float *KalmanFilter::EstimateSpeed(double _measureRad, float Vin) {

  //Update Observer and Command
  measureRad = _measureRad;
  updated_U[0] = Vin;
  updated_Ymeas[0] = measureRad;
  U.read(updated_U);
  Y.read(updated_Ymeas);

  //Kalmanfilter
  run();



  //Estimate Velocity
  // estimateVel = resultX.data[1][0];

  //Estimate State
  estimateState[0] = resultX.data[0][0];
  estimateState[1] = resultX.data[1][0];
  estimateState[2] = resultX.data[2][0];
  estimateState[3] = resultX.data[3][0];

  return estimateState;
}
