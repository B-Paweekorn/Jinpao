#ifndef MOTOR_H
#define MOTOR_H

#include <arduino.h>

class Motor{
  public:
    
    // Time
    float prevT = 0.0; // previous time
    float curT = 0.0; // current time
    
    // PID and low pass filter
    float eprev_position = 0; // previous error
    float eintegral_position = 0; // sum of error 
    float eprev_velocity = 0; // previous error
    float eintegral_velocity = 0; // sum of error 
    
    float s2prev = 0; // previous position with filter
    float s2filt = 0; // current position with filter
    
    // Trajectory
    float t1 = 0.0; //time 1 to 2 
    float t2 = 0.0; // time 2 to 3
    float t3 = 0.0; // end time
    
    float initial_position = 0.0; // initial position
    float target_position = 0.0; // target position
    float max_velocity = 0.0; // max velocity of motor
    float max_acceleration = 0.0; // max acceleration of motor
    float distance = 0.0; // between initial position and target position
    
    float tempS = 0.0; // position when change state
    float tempT = 0.0; // time when change state
    float tempV = 0.0; // velocity when change state
    
    float tim = 0.0; // current time in each trajectory
    float v = 0.0; // current velocity in each state of trajectory
    float s = 0.0; // current position in each state of trajectory


    // Position
    int cnt = 0; // current position of motor
    int cnt2 = 0; // previous position of motor
    float pos = 0.00;

    // Trajectory
    float posit = 0.0; //current position in each trajectory
    // Temporary
    double temp = 0; //state 1 = change target / 0 = not change target  
    Motor();
    void init();
    void cascade(float estimateVel);
    void timing(float ip, float tp, float mv,float ma);
    void trapezoidal_trajectory(float t);
    void setmotor(int dir, int pwmVal, int pwm, int in1, int in2); 
    
    float u_velocity;
  
};

#endif