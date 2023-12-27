#include "AS5600.h"
#include "Wire.h"
AS5600 as5600;  //  use default Wire

// #include <Kalman.h>
// using namespace BLA;

/* USER CODE BEGIN Includes */
#include "Matrix.h"
#include "KalmanFilter.h"
/* USER CODE END Includes */

/* PREPARE kalman filter BEGIN*/

// declare kalman filter instance
kalman_filter KF;

/* PREPARE kalman filter END*/

//------------------------------------
/****  MICROCONTROLLER SETUP  ****/
//------------------------------------

#define dirPin 40
#define pwmPin 39
#define PWM1_Ch 0
#define PWM1_Res 14
#define PWM1_Freq 1000

//------------------------------------
/****  MODELIZATION PARAMETERS  ****/
//------------------------------------

#define Nstate 3  // Position, Velocity, Current
#define Nobs 1    // Position
#define Ncom 1    // Vin

// measurement std of the noise
#define n_p 0.3  // position measurement noise

// model std (1/inertia)
#define m_p 0.1  // position model noise
#define m_v 0.1  // velocity model noise
#define m_i 0.8  // current model noise

// BLA::Matrix<Nobs> obs;         // observation vector
// BLA::Matrix<Ncom> com;         // command vector (input)
// KALMAN<Nstate, Nobs, Ncom> K;  // your Kalman filter
// Note: I made 'obs' a global variable so memory is allocated before the loop.
//       This might provide slightly better speed efficiency in loop.

//Check loop time
unsigned long T;  // current time
float DT;         // delay between two updates of the filter

//Print loop Time (100 Hz)
unsigned long prev_timestep_print;
unsigned long current_timestep_print;
unsigned long timestamp_print = 0;
int timestep_print = 10000;

//Control loop Time (1000 Hz)
unsigned long prev_timestep;
unsigned long current_timestep;
unsigned long timestamp = 0;
int timestep = 1000;

//Sinwave Time (1000 Hz)
unsigned long tcur;

//AS5600 Unwrap
int rawAngle_prev = 0;
int count = 0;
int32_t Angle = 0;
int32_t Angle_rad = 0;
float velocity_angle = 0;
//Input Command
float Vin;

//Output of Kalmanfilter
float estimateVel;

//------------------------------------
/****    SIMULATOR PARAMETERS   ****/
//------------------------------------

// These variables simulate a physical process to be measured
// In real life, the SIMULATOR is replaced by your operational system

// BLA::Matrix<Nstate> state;  // true state vector

//Generate Sinwave
float dummy_sinwave = 0;
#define SIMUL_PERIOD 0.3  // oscillating period [s]
#define SIMUL_AMP 4900.0  // oscillation amplitude


//------------------------------------
/****        SETUP & LOOP       ****/
//------------------------------------

void setup() {

  Serial.begin(115200);

  Wire.begin(13, 14);
  Wire.setClock(400000);

  //AS5600 Setup
  delay(1000);
  as5600.begin(4);                         //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  delay(1000);
  
  //PWM Pin
  ledcAttach(pwmPin, PWM1_Freq, PWM1_Res);
  ledcWrite(pwmPin, 0);

  //Direction Pin
  pinMode(dirPin, OUTPUT);
  
  KF.init();
}

void loop() {

  // TIME COMPUTATION
  // DT = (micros() - T);
  // T = micros();

  // Serial.println(DT);
  //Serial.print(" ");

  //Print loop
  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    timestamp_print = micros();

    Serial.print(velocity_angle);
    Serial.print(" ");
    Serial.println(estimateVel);
  }

  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    //Serial.println(current_timestep - timestamp);
    timestamp = micros();

    // Generate Sinwave to motor
    setmotor_sinwave();

    // Result of the measurement is written into 'obs'
    Update_Measurement();

    estimateVel = KF.EstimateSpeed(Angle, Vin);

  }
}

//------------------------------------
/****     SIMULATOR FUNCTIONS   ****/
//------------------------------------

void setmotor_sinwave() {
  tcur++;
  dummy_sinwave = SIMUL_AMP * sin(tcur / 1000.0 / SIMUL_PERIOD);  // position
  //dummy_sinwave = SIMUL_AMP/SIMUL_PERIOD*cos(tcur/1000.0/SIMUL_PERIOD); // speed

  // Update Input into com
  Update_Command(dummy_sinwave);

  if (dummy_sinwave >= 0) {
    setmotor(dummy_sinwave, 1);
  } else if (dummy_sinwave < 0) {
    dummy_sinwave = dummy_sinwave * (-1);
    setmotor(dummy_sinwave, 0);
  }
}

void setmotor(int dutyCycle, int In_dir) {
  ledcWrite(pwmPin, dutyCycle);
  digitalWrite(dirPin, In_dir);
}

int AS5600_Unwrap(int rawAngle) {
  if (rawAngle - rawAngle_prev <= -2500) {
    count++;
  } else if (rawAngle - rawAngle_prev >= 2500) {
    count--;
  }

  rawAngle_prev = rawAngle;
  return count * 4096 + rawAngle;
}

void Update_Command(int Raw_dutyCycle) {
  Vin = Raw_dutyCycle * 18 / 16383.0;  // 14 Bits (16383)
}

void Update_Measurement() {
  Angle = AS5600_Unwrap(as5600.rawAngle());                      //Unwrap Angle
  Angle_rad = Angle * AS5600_RAW_TO_RADIANS;                   //Position
  velocity_angle = as5600.getAngularSpeed(AS5600_MODE_RADIANS);  //Velocity
}
