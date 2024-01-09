#ifndef MOBILE_COMMAND_H
#define MOBILE_COMMAND_H

#include "QEI.h"
#include "setMotor.h"
#include "Kinematics.h"
#include "PID.h"

class Mobile_command {
private:
    static const int NUM_MOTORS = 4; // Number of motors, encoders, and PIDs

    setMotor* motors[NUM_MOTORS];
    QEI* encoders[NUM_MOTORS];
    PID* pids[NUM_MOTORS];
    Kinematics* kinematics;

    

public:
    Mobile_command(setMotor* _motors[], QEI* _encoders[], PID* _pids[], Kinematics* _kinematics);
    void control(float _vx, float _vy, float _wz);

};

#endif
