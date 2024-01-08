#include "Kinematics.h"

Kinematics::Kinematics(float wheel_diameter, float lx, float ly) {
  robot.wheel_diameter = wheel_diameter;
  robot.lx = lx;
  robot.ly = ly;
  robot.wheel_circumference = robot.wheel_diameter * M_PI;
  robot.angular_to_rpm = 60 / robot.wheel_circumference;
}

Kinematics::Velocity Kinematics::Forward_Kinematics_Velocity(float radps_fl, float radps_fr, float radps_bl, float radps_br) {
  Velocity basev;
  basev.vx = (robot.wheel_diameter / 2) / 4 * (radps_fl + radps_fr + radps_bl + radps_br);
  basev.vy = (robot.wheel_diameter / 2) / 4 * (-radps_fl + radps_fr + radps_bl - radps_br);
  basev.wz = (robot.wheel_diameter / 2) / (4 * (robot.lx + robot.ly)) * (radps_fl - radps_fr + radps_bl - radps_br);
  return basev;
}

Kinematics::RadPS Kinematics::Inverse_Kinematics(float vx, float vy, float wz) {
  RadPS wheel_rads;
  wheel_rads.radps_fl = (vx + vy + (robot.lx + robot.ly) * wz)/(robot.wheel_diameter / 2.0);
  wheel_rads.radps_fr = (vx - vy - (robot.lx + robot.ly) * wz)/(robot.wheel_diameter / 2.0);
  wheel_rads.radps_bl = (vx - vy + (robot.lx + robot.ly) * wz)/(robot.wheel_diameter / 2.0);
  wheel_rads.radps_br = (vx + vy - (robot.lx + robot.ly) * wz)/(robot.wheel_diameter / 2.0);


  return wheel_rads;
}

Kinematics::Position Kinematics::Forward_Kinematics_Position(float radps_fl, float radps_fr, float radps_bl, float radps_br, Position current_position) {
  Velocity basev = Forward_Kinematics_Velocity(radps_fl, radps_fr, radps_bl, radps_br);
  float dt = 1 / 1000.0;
  current_position.x += basev.vx * dt;
  current_position.y += basev.vy * dt;
  current_position.theta += basev.wz * dt * (180.0 / M_PI);
  current_position.theta = fmod(current_position.theta, 360.0);

  if (current_position.theta < 0) {
    current_position.theta += 360.0;
  }

  return current_position;
}