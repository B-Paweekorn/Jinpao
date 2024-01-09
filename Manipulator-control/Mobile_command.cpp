#include "HardwareSerial.h"
#include "Mobile_command.h"


Mobile_command::Mobile_command(setMotor* _motors[], QEI* _encoders[], PID* _pids[], Kinematics* _kinematics)
  : kinematics(_kinematics) {  // Initialize kinematics using the initializer list
  // Initialize motors, encoders, and pids using a loop in the constructor body
  for (int i = 0; i < NUM_MOTORS; ++i) {
    motors[i] = _motors[i];
    encoders[i] = _encoders[i];
    pids[i] = _pids[i];
  }
}


void Mobile_command::control(float _vx, float _vy, float _wz) {
  Kinematics::RadPS wheel_radps = kinematics->Inverse_Kinematics(_vx, _vy, _wz);
  
  // Set the target rotational speed for each wheel's PID controller
  pids[0]->setRads(wheel_radps.radps_fl);  // Front Left Wheel
  pids[1]->setRads(wheel_radps.radps_fr);  // Front Right Wheel
  pids[2]->setRads(wheel_radps.radps_bl);  // Back Left Wheel
  pids[3]->setRads(wheel_radps.radps_br);  // Back Right Wheel

  // Compute PID for each wheel
  for (int i = 0; i < NUM_MOTORS; ++i) {
    pids[i]->compute();
  }
}
