/*
 * Run an example of Kalman filter.
 * This example simulates a sinusoidal position of an object.
 * The 'SIMULATOR_' functions below simulate the physical process and its measurement with sensors. 
 * Results are printed on Serial port. You can use 'kalman_full.py' to analyse them with Python.
 * 
 * Author:
 *  R.JL. Fétick
 *  
 * Revision:
 *  31 Aug 2019 - Creation
 * 
 */

#include "AS5600.h"
#include "Wire.h"
AS5600 as5600;  //  use default Wire

#include <Kalman.h>
using namespace BLA;

#define dirPin 19
#define pwmPin 18

//------------------------------------
/****  RP2040 PWM SETTING  ****/
//------------------------------------
//#include "RP2040_PWM.h"
//creates pwm instance
//RP2040_PWM* PWM_Instance;

float frequency;
float dutyCycle;

int rawAngle_prev = 0;
int count = 0;
int32_t Angle = 0;

//------------------------------------
/****  MODELIZATION PARAMETERS  ****/
//------------------------------------

#define Nstate 4  // position, velocity, external load, current
#define Nobs 2    // Position, velocity
#define Ncom 2    // Vin, external force

// measurement std of the noise
#define n_p 1.0         // position measurement noise 0.3
#define n_v 10000000.0  // velocity measurement noise 5.0

// model std (1/inertia)
#define m_p 0.1
#define m_v 0.1  //0.1
#define m_l 0.8
#define m_i 0.8

BLA::Matrix<Nobs> obs;         // observation vector
BLA::Matrix<Ncom> com;         // command vector
KALMAN<Nstate, Nobs, Ncom> K;  // your Kalman filter
unsigned long T;               // current time
float DT;                      // delay between two updates of the filter

// Note: I made 'obs' a global variable so memory is allocated before the loop.
//       This might provide slightly better speed efficiency in loop.

unsigned long timestamp;
unsigned long check_time;

unsigned long step_prev;
unsigned long step_current;

//------------------------------------
/****    SIMULATOR PARAMETERS   ****/
//------------------------------------

// These variables simulate a physical process to be measured
// In real life, the SIMULATOR is replaced by your operational system

BLA::Matrix<Nstate> state;  // true state vector

float dummy_sinwave = 0;
#define SIMUL_PERIOD 0.3  // oscillating period [s]
#define SIMUL_AMP 30.0    // oscillation amplitude
#define LOOP_DELAY 10     // add delay in the measurement loop [ms]

//------------------------------------
/****        SETUP & LOOP       ****/
//------------------------------------

void setup() {

  Serial.begin(921600);

  //Wire.setSDA(20);
  //Wire.setSCL(21);
  Wire.begin();

  delay(1000);
  as5600.begin(4);                         //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  delay(1000);

  pinMode(dirPin, OUTPUT);
  //PWM_Instance = new RP2040_PWM(pwmPin, 5000, 0);

  // time evolution matrix (whatever... it will be updated inloop)
  K.F = { 1.0, 4.990144645001977E-4, -9.339164534756672E-6, 1.489079905462724E-6,
          0.0, 0.994996613765791, -0.037324340472272, 0.004663672724017,
          0.0, 0.0, 1.0, 0.0,
          0.0, -0.678954744573234, 0.016214731915530, 0.174280858811019 };

  K.B = { 1.522624718567567E-6, 0.0,
          0.008107365957765, 0.0,
          0.0, 0.0,
          1.290169016824592, 0.0 };

  // measurement matrix n the position (e.g. GPS) and acceleration (e.g. accelerometer)
  K.H = { 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0 };

  // measurement covariance matrix
  K.R = { n_p * n_p, 0.0,
          0.0, n_v * n_v };
  // model covariance matrix
  K.Q = { m_p * m_p, 0.0, 0.0, 0.0,
          0.0, m_v * m_v, 0.0, 0.0,
          0.0, 0.0, m_l * m_l, 0.0,
          0.0, 0.0, 0.0, m_i * m_i };

  T = millis();

}

void loop() {



  // TIME COMPUTATION
  DT = (micros() - T);
  T = micros();

  Serial.println(DT);
  //Serial.print(" ");


  // UPDATE THE SIMULATED PHYSICAL PROCESS
  //setmotor_sinwave();

  // Update Input into com
  //Update_Command();

  // // SIMULATE A NOISY MEASUREMENT WITH A SENSOR
  // // Result of the measurement is written into 'obs'
  //Update_Measurement();

  // // APPLY KALMAN FILTER
  K.update(obs, com);

  // PRINT RESULTS: true state, measurements, estimated state, posterior covariance
  // The most important variable for you might be the estimated state 'K.x'
  //Serial << state << ' ' << obs << ' ' << K.x << ' ' << K.P << '\n';

  //Plot State
  // Serial.print(observation);
  // Serial.print(" ");
  // Serial.print(X[0]);
  // Serial.print(" ");
  // Serial.println(P[0]);


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
  // Serial.print(K.x(2));
  // Serial.print(" ");
  // Serial.println(K.x(3));
}

//------------------------------------
/****     SIMULATOR FUNCTIONS   ****/
//------------------------------------

void setmotor_sinwave() {
  unsigned long tcur = millis();
  dummy_sinwave = SIMUL_AMP * sin(tcur / 1000.0 / SIMUL_PERIOD);  // position
  //dummy_sinwave = SIMUL_AMP/SIMUL_PERIOD*cos(tcur/1000.0/SIMUL_PERIOD); // speed
  //dummy_sinwave = 0;
  dutyCycle = dummy_sinwave;

  if (dummy_sinwave >= 0) {
    setmotor(dummy_sinwave, 1);
  } else if (dummy_sinwave < 0) {
    dummy_sinwave = dummy_sinwave * (-1);
    setmotor(dummy_sinwave, 0);
  }
}

void setmotor(int dutyCycle, int In_dir) {
  frequency = 5000;
  //PWM_Instance->setPWM(pwmPin, frequency, dutyCycle);
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

void SIMULATOR_UPDATE() {
  // Simulate a physical process
  // Here we simulate a sinusoidal position of an object
  unsigned long tcur = millis();
  state(0) = SIMUL_AMP * sin(tcur / 1000.0 / SIMUL_PERIOD);                 // position
  state(1) = SIMUL_AMP / SIMUL_PERIOD * cos(tcur / 1000.0 / SIMUL_PERIOD);  // speed
  //state(2) = -SIMUL_AMP/SIMUL_PERIOD/SIMUL_PERIOD*sin(tcur/1000.0/SIMUL_PERIOD); // acceleration
}

void Update_Command() {
  float Vin = dutyCycle * 18 / 100.0;
  com(0) = Vin;
}

void Update_Measurement() {
  Angle = AS5600_Unwrap(as5600.rawAngle());
  obs(0) = Angle * AS5600_RAW_TO_RADIANS;
  obs(1) = as5600.getAngularSpeed(AS5600_MODE_RADIANS);
}
