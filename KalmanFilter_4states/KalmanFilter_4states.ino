#include "AS5600.h"
#include "Wire.h"
AS5600 as5600;  //  use default Wire

// #include <Kalman.h>
// using namespace BLA;

/* USER CODE BEGIN Includes */
#include "Matrix.h"
#include "KalmanFilter.h"
/* USER CODE END Includes */

float frequency;
float dutyCycle;

/* PREPARE kalman filter BEGIN*/

// declare kalman filter instance
kalman_filter KF;
//known input
float U_data[1] = {0.0};
matrix U_MATRIX(1, 1, U_data);
// Y measurement
float Ymeas_data[1] = {0.0};
matrix Ymeas_MATRIX(1, 1, Ymeas_data);
//output
matrix KF_output(3, 1);
/* PREPARE kalman filter END*/

//------------------------------------
/****  MICROCONTROLLER SETUP  ****/
//------------------------------------

#define dirPin 4
#define pwmPin 2
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
#define n_p 0.3         // position measurement noise

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

  Serial.begin(921600);

  Wire.begin(8, 9);
  Wire.setClock(400000);

  //AS5600 Setup
  delay(1000);
  as5600.begin(4);                         //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  delay(1000);

  // // time evolution matrix (whatever... it will be updated inloop)
  // K.F = { 1.0, 9.945211490311812E-4, 4.075838177204351E-6,
  //         0.0, 0.98685183868228, 0.005453127455664,
  //         0.0, -0.79388648772862, 0.027207395024798 };

  // // Input matrix
  // K.B = { 9.012107076660543e-06,
  //         0.022191093685438, 
  //         1.509516246505134};

  // // measurement matrix n the position and velocity
  // K.H = { 1.0, 0.0, 0.0};

  // // measurement covariance matrix
  // K.R = { n_p * n_p};

  // // model covariance matrix
  // K.Q = { m_p * m_p, 0.0, 0.0,
  //       0.0, m_v * m_v, 0.0,
  //       0.0, 0.0, m_i * m_i };
  /*kalman filter INITIAL BEGIN*/
  // state trnsition matrix
  float A_data[9]  = { 1.0, 9.945211490311812E-4, 4.075838177204351E-6,
                        0.0, 0.98685183868228, 0.005453127455664,
                        0.0, -0.79388648772862, 0.027207395024798 };
  matrix A_MATRIX(3, 3, A_data);

  //input matrix
  float B_data[3]  = { 9.012107076660543E-06,
                           0.022191093685438, 
                           1.509516246505134};
  matrix B_MATRIX(3, 1, B_data);

  // observation matrix
  float C_data[3] = { 1.0, 0.0, 0.0};
  matrix C_MATRIX(1, 3, C_data);

  //D matrix
  matrix D_MATRIX(3, 1);

  // measurement covariance matrix
  float R_data[1] = { n_p * n_p};
  matrix R_MATRIX(1, 1, R_data);

  // Process noise covariance matrix
  float G_data[3] = {0.0, 1.0, 0.0};
  matrix G_MATRIX(3, 1, G_data);
  // float Q_data[9] = { m_p * m_p, 0.0, 0.0,
  //                         0.0, m_v * m_v, 0.0,
  //                         0.0, 0.0, m_i * m_i };
  // matrix Q_MATRIX(3, 3, &Q_data);

  //process model variance vaule
  float Q_data[1] = {0.01}; //assign Q here<-------------
  matrix Q_MATRIX(1, 1, Q_data);

  //kalman filter matrix set up
  KF.setAtoD(A_MATRIX, B_MATRIX, C_MATRIX, D_MATRIX);
  KF.setQGR(Q_MATRIX, G_MATRIX, R_MATRIX);
  /*kalman filter INITIAL END*/

  //Direction Pin
  pinMode(dirPin, OUTPUT);

  //PWM Pin
  // ledcAttachPin(pwmPin, PWM1_Ch);
  // ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
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

    float newU[1] = {0};
    float newYmeas[1] = {0};
    U_MATRIX.read(newU);
    Ymeas_MATRIX.read(newYmeas);
    KF.run(U_MATRIX,Ymeas_MATRIX);
    KF_output = KF.resultX;

    // Serial.print(15000);
    // Serial.print(" ");
    // Serial.print(-15000);
    // Serial.print(" ");
    // Serial.println(dummy_sinwave);

    // Serial.print(obs(0));
    // Serial.print(" ");
    // Serial.print(obs(1));
    // Serial.print(" ");
    // Serial.print(K.x(0));
    // Serial.print(" ");
    // Serial.print(K.x(1));
    // Serial.print(" ");
    // Serial.print(35);
    // Serial.print(" ");
    // Serial.println(-35);
  }

  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    //Serial.println(current_timestep - timestamp);
    timestamp = micros();

    // Generate Sinwave to motor
    //setmotor_sinwave();

    // Update Input into com
    //Update_Command(dummy_sinwave);

    // // Result of the measurement is written into 'obs'
    // Update_Measurement();

    // // APPLY KALMAN FILTER
    // K.update(obs, com);
  }
}

//------------------------------------
/****     SIMULATOR FUNCTIONS   ****/
//------------------------------------

void setmotor_sinwave() {
  tcur++;
  dummy_sinwave = SIMUL_AMP * sin(tcur / 1000.0 / SIMUL_PERIOD);  // position
  //dummy_sinwave = SIMUL_AMP/SIMUL_PERIOD*cos(tcur/1000.0/SIMUL_PERIOD); // speed

  if (dummy_sinwave >= 0) {
    setmotor(dummy_sinwave, 1);
  } else if (dummy_sinwave < 0) {
    dummy_sinwave = dummy_sinwave * (-1);
    setmotor(dummy_sinwave, 0);
  }
}

void setmotor(int dutyCycle, int In_dir) {
  ledcWrite(PWM1_Ch, dutyCycle);
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

// void Update_Command(int Raw_dutyCycle) {
//   float Vin = Raw_dutyCycle * 18 / 100.0;  // 14 Bits (16383)
//   com(0) = Vin;
// }

// void Update_Measurement() {
//   Angle = AS5600_Unwrap(as5600.rawAngle());              //Unwrap Angle
//   obs(0) = Angle * AS5600_RAW_TO_RADIANS;                //Position
//   obs(1) = as5600.getAngularSpeed(AS5600_MODE_RADIANS);  //Velocity
// }
