#include "QEI.h"
#include "setMotor.h"
#include "Kinematics.h"
#include "PID.h"
#include "Mobile_command.h"
#include "AS5600.h"
#include "Wire.h"

AS5600 as5600;  // Assuming AS5600 is compatible with TwoWire

#define I2C_SDA 13
#define I2C_SCL 14
/*-----QEI(encA, encB)------*/

QEI enc1(17, 18);
QEI enc2(33, 34);
QEI enc3(35, 36);
QEI enc4(37, 38);

long counter0 = 0;
long counter1 = 0;
long counter2 = 0;
long counter3 = 0;

/*-----Config Motor -----*/

#define MOTOR_TIMER_14_BIT 14
#define MOTOR_BASE_FREQ 1000

#define MOTOR_1_PWM_PIN 39
#define MOTOR_2_PWM_PIN 42
#define MOTOR_3_PWM_PIN 1
#define MOTOR_4_PWM_PIN 4

#define MOTOR_1_PWM_CHANNEL 0
#define MOTOR_2_PWM_CHANNEL 1
#define MOTOR_3_PWM_CHANNEL 2
#define MOTOR_4_PWM_CHANNEL 3

#define MOTOR_1_DIR_PIN 40
#define MOTOR_2_DIR_PIN 41
#define MOTOR_3_DIR_PIN 2
#define MOTOR_4_DIR_PIN 5

setMotor MOTOR_1(MOTOR_1_PWM_PIN, MOTOR_1_DIR_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);
setMotor MOTOR_2(MOTOR_2_PWM_PIN, MOTOR_2_DIR_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);
setMotor MOTOR_3(MOTOR_3_PWM_PIN, MOTOR_3_DIR_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);
setMotor MOTOR_4(MOTOR_4_PWM_PIN, MOTOR_4_DIR_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);

/*-----Config ADC-----*/
#define ADC_SDA 13
#define ADC_SCL 14

/*-----Config BNO-----*/
#define BNO_SDA 15
#define BNO_SCL 16

/*-----Config PID -----*/
PID pid1(&MOTOR_1, &enc1, 1, 0.0f, 0.0f);
PID pid2(&MOTOR_2, &enc2, 0.0f, 0.0f, 0.0f);
PID pid3(&MOTOR_3, &enc3, 0.0f, 0.0f, 0.0f);
PID pid4(&MOTOR_4, &enc4, 0.0f, 0.0f, 0.0f);

/*-----Config Robot Base-----*/
#define WHEEL_DIAMETER 0.25  //meter
#define LX 0.5               //meter
#define LY 0.5               //meter

Kinematics kinematics(WHEEL_DIAMETER, LX, LY);


/*-----Pass value-----*/
setMotor* motors[4] = { &MOTOR_1, &MOTOR_2, &MOTOR_3, &MOTOR_4 };
QEI* encoders[4] = { &enc1, &enc2, &enc3, &enc4 };
PID* pids[4] = { &pid1, &pid2, &pid3, &pid4 };

Mobile_command Mobile(motors, encoders, pids, &kinematics);

int commaIndex1, commaIndex2;
String speedX, speedY, speedZ;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  /*-----Setup Hardware Start-----*/

  MOTOR_1.begin();
  MOTOR_2.begin();
  MOTOR_3.begin();
  MOTOR_4.begin();

  enc1.begin();
  enc2.begin();
  enc3.begin();
  enc4.begin();
  Wire.begin(I2C_SDA, I2C_SCL);  // Initialize I2C communication

  as5600.begin();                          // Initialize AS5600 sensor
  as5600.setDirection(AS5600_CLOCK_WISE);  // Set direction

  // pid1.setK(1, 1, 1);

  /*-----Setup Hardware End-------*/
}

void loop() {
  float v = as5600.getAngularSpeed(AS5600_MODE_RADIANS);
  Mobile.control(10, 0, 0, v);
  Serial.println(pid1.u);
  /*-----Test PWM Start (0 - 16383)-----*/
  // %duty 0 - 16382
  // MOTOR_1.setPWM(4096);
  // MOTOR_2.setPWM(8192);
  // MOTOR_3.setPWM(12288);
  // MOTOR_4.setPWM(16383);

  /*-----Test PWM End-------*/

  uint32_t time = micros();

  counter0 += enc1.get_diff_count();
  counter1 += enc2.get_diff_count();
  counter2 += enc3.get_diff_count();
  counter3 += enc4.get_diff_count();

  uint32_t dt = micros() - time;

  Serial.print(dt);
  Serial.print(' ');
  Serial.print(counter0);
  Serial.print(' ');
  Serial.print(counter1);
  Serial.print(' ');
  Serial.print(counter2);
  Serial.print(' ');
  Serial.println(counter3);

  //Receive Data
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
}

void test() {
  Serial.println("Success");
}