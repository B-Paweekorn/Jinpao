#ifndef __MOTOR_MODEL_H__
#define __MOTOR_MODEL_H__

#include <stdint.h>

typedef struct {
  float Ke;
  float Kt;
  float L;
  float R;
  float J;
  float B;
} MotorConstant_Structure;

#endif