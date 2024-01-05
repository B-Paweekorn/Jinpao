#include "motor.h"
#include "RPi_Pico_TimerInterrupt.h"
#include "pio_encoder.h"
#include <cmath>

//Timer Interrupt Setup
#define dt_us 1000
float deltaT = dt_us / 1.0e6;
RPI_PICO_Timer Timer(0);
bool TimerStatus = false;

//Motor Setup
Motor FL(0, 4, 5, 30000, 100, 0);
Motor FR(14, 2, 3, 30000,100, 0);
Motor BL(18, 6, 7, 30000, 100, 0);
Motor BR(16, 8, 9, 30000, 100, 0);

//Transform
struct TransformStep {
  float vx;
  float vy;
  float wz;
  unsigned long duration;
};

//RPM of each wheel for inverse kinematic
struct RPM {
  float rpm_FL;
  float rpm_FR;
  float rpm_BL;
  float rpm_BR;
};

//Velocity for forward kinematic
struct Velocity {
  float vx;
  float vy;
  float wz;
};

//Postion
struct Position {
  float x;
  float y;
  float theta;
};

Position currentPosition = { 0.0, 0.0, 0.0 };

//Kinematic Setup
const float wheelDiameter = 0.058;
const float l = 0.585;
const float lx = 0.15;
const float ly = 0.15;
const float wheelCircumference = wheelDiameter * PI;
float angular_to_rpm = 60 / wheelCircumference;

//variable
long start_time, T;
int currentStep = 0;
int stepsCount = 0;
bool newStepsAvailable = false;
TransformStep steps[10];

int commaIndex1, commaIndex2;
String speedX, speedY, speedZ;

void setup() {
  Serial.begin(250000);
  Serial.setTimeout(1);
  analogWriteFreq(2400);
  analogWriteRange(62500);

  FR.encoder.begin();
  FL.encoder.begin();
  BR.encoder.begin();
  BL.encoder.begin();

  //Timer Interrupt Setup
  TimerStatus = Timer.attachInterruptInterval(dt_us, TimerHandler);
  start_time = millis();
}

void loop() {
}

bool TimerHandler(struct repeating_timer* t) {
  (void)t;
  long T = millis();
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    // Serial.print("Received data: ");
    // Serial.println(data);

    int firstSemiColon = data.indexOf(';');
    int secondSemiColon = data.indexOf(';', firstSemiColon + 1);

    speedX = data.substring(0, firstSemiColon);
    speedY = data.substring(firstSemiColon + 1, secondSemiColon);
    speedZ = data.substring(secondSemiColon + 1);

    Serial.print("Parsed X: ");
    Serial.print(strToFloat(speedX));
    Serial.print(" Y: ");
    Serial.print(strToFloat(speedY));
    Serial.print(" Z: ");
    Serial.println(strToFloat(speedZ));
  }
  RPM wheel_rpms = Inverse_Kinematic(strToFloat(speedX), strToFloat(speedY), strToFloat(speedZ));
  FL.compute(wheel_rpms.rpm_FL, deltaT);
  FR.compute(wheel_rpms.rpm_FR, deltaT);
  BL.compute(wheel_rpms.rpm_BL, deltaT);
  BR.compute(wheel_rpms.rpm_BR, deltaT);
  // Serial.println(" ");

  currentPosition = FindPosition(wheel_rpms.rpm_FL, wheel_rpms.rpm_FR, wheel_rpms.rpm_BL, wheel_rpms.rpm_BR, deltaT, currentPosition);
  // TestKinematic(0,0,3.14);

  // FL.compute(20,deltaT);
  return true;
}