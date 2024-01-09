#ifndef PID_H
#define PID_H

#include "setMotor.h"
#include "QEI.h"
#include <math.h>
#include "KalmanFilter.h"

class PID {
private:


public:

  setMotor* motor;  // Pointer to a setMotor object
  QEI* enc;
  KalmanFilter* kf;
  float dt = 1 / 1000.0;
  float kp, ki, kd;
  float eintegral = 0;
  float e = 0;
  float v = 0;
  float q = 0;
  float Vin = 0;
  double u = 0;
  double u_out = 0;
  double u_prev = 0;
  float e_prev = 0;
  float e_prev2 = 0;
  int32_t pos_prev = 0;
  float targetRads = 0;  // Renamed from setRads
  float counts_per_rev = 2048 * 4.0;
  float a = 13; //134.2335958;
  float PWM_feedforward = 0;
  float i = 0;

  PID(setMotor* _motor, QEI* _enc, KalmanFilter* _kf, float _kp, float _ki, float _kd);  // Constructor that takes a setMotor object
  void compute();
  void setRads(float _setRads);  // This remains the same
  void setK(float _kp, float _ki, float _kd);
  float scaleToPWM(float v_in);
  float calculateV_in(float omega, float i);
  float *estimateState;
};

#endif
