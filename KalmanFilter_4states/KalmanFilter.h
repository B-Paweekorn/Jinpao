#include <stdint.h>
/*
 * kalmanfilter.h
 *
 *  Created on: 15 เม.ย. 2565
 *      Author: weera
 */

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include "Matrix.h"

class kalman_filter {
public:
  matrix A, B, C, D, R, G, Q;
  matrix gainK, errorY, U, Y, I33;
  matrix P, P_old, P_new;
  matrix predictX, predictX_old, predictX_new;
  matrix resultX, resultY;

  float updated_U[1] = { 0 };
  float updated_Ymeas[1] = { 0 };

  // state trnsition matrix
  float A_data[16] = { 1.0, 9.945211490311812E-4, -3.727956981163958E-5, 4.075838177204351E-6,
                       0.0, 0.98685183868228, -0.074386312650263, 0.005453127455664,
                       0.0, 0.0, 1.0, 0.0,
                       0.0, -0.79388648772862, 0.044382187370876, 0.027207395024798 };
  
  //input matrix
  float B_data[4] = { 9.012107076660543E-06,
                      0.022191093685438,
                      0.0,
                      1.509516246505134 };
  
  // observation matrix
  float C_data[4] = { 1.0, 0.0, 0.0, 0.0 };
  
  // measurement covariance matrix
  float R_data[1] = { 10000 };  //0.09

  // Process noise covariance matrix
  float G_data[4] = { 0.0, 1.0, 0.0, 0.0 };
  
  //process model variance vaule
  float Q_data[1] = { 0.01 };  //assign Q here<------------- 0.01

  float estimateVel;
  float measureRad;
  

  kalman_filter();
  void init();
  void doKalman_gain();
  void doCorrect_p();
  void doPredict_y();
  void doCorrect();
  void doPredict_x();
  void doPredict_p();
  void doResult();
  void run();
  float EstimateSpeed(int32_t measurePulse, float Vin);
};

#endif /* INC_KALMANFILTER_H_ */
