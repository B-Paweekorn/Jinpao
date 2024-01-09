#ifndef __CYTRON_MOTOR_680RPM_250W_H__
#define __CYTRON_MOTOR_680RPM_250W_H__

#include <stdint.h>
#include "Motor_Model.h"

MotorConstant_Structure CYTRON_MOTOR_680RPM_250W_Constant = {
  .Ke = 0.201460170000000,
  .Kt = 0.100730085000000,
  .L = 1.624060150375940E-04,
  .R = 0.604511278195489,
  .J = 0.002131517773002,
  .B = 0.001114942610956
};

float CYTRON_MOTOR_680RPM_250W_MatrixA[16] = { 1.0, 9.949973044407977E-04, -2.339023679053055E-04, 9.332203891763732E-06,
                     0.0, 0.987903201446773, -0.466802255671299, 0.012284494183730,
                     0.0, 0.0, 1.0, 0.0,
                     0.0, -0.322458717787010, 0.114924362741164, 0.020453711839646 };

//input matrix
float CYTRON_MOTOR_680RPM_250W_MatrixB[4] = { 2.353769403858433E-05,
                    0.057462181370582,
                    0.0,
                    1.601243818349142 };

// observation matrix
float CYTRON_MOTOR_680RPM_250W_MatrixC[4] = { 1.0, 0.0, 0.0, 0.0 };

//process model variance vaule
float CYTRON_MOTOR_680RPM_250W_MatrixQ[1] = { 1 };

// measurement covariance matrix
float CYTRON_MOTOR_680RPM_250W_MatrixR[1] = { 0.0001 }; //0.0001



#endif