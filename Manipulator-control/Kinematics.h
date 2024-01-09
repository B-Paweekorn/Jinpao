#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
class Kinematics {
private:


public:

  //Robot
  struct Robot {
    float wheel_diameter;
    float lx;
    float ly;
    float wheel_circumference;
    float angular_to_rpm;
  };

  Robot robot;  // Adding Robot object as a member

  //RadPS of each wheel for inverse kinematic
  struct RadPS {
    float radps_fl;
    float radps_fr;
    float radps_bl;
    float radps_br;
  };

  //Velocity for forward kinematic
  struct Velocity {
    float vx;
    float vy;
    float wz;
  };

  //Position
  struct Position {
    double x;
    double y;
    double theta;  //in degree
  };

  Position current_position;

  Kinematics(float wheel_diameter, float lx, float ly);
  RadPS Inverse_Kinematics(float vx, float vy, float wz);
  Velocity Forward_Kinematics_Velocity(float radps_fl, float radps_fr, float radps_bl, float radps_br);
  Position Forward_Kinematics_Position(float radps_fl, float radps_fr, float radps_bl, float radps_br, Position current_position);
};

#endif  // KINEMATICS_H
