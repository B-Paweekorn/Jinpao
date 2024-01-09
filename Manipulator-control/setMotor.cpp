#include "esp32-hal-ledc.h"
#include "esp32-hal-gpio.h"
#include "setMotor.h"
#include "math.h"

setMotor::setMotor(int16_t _PWM_PIN, uint8_t _DIR_PIN, uint32_t _MOTOR_BASE_FREQ, uint8_t _MOTOR_TIMER_14_BIT)
{
    PWM_PIN = _PWM_PIN;
    DIR_PIN = _DIR_PIN;
    MOTOR_BASE_FREQ = _MOTOR_BASE_FREQ;
    MOTOR_TIMER_14_BIT = _MOTOR_TIMER_14_BIT;

}

void setMotor::begin()
{
    pinMode(DIR_PIN, OUTPUT);
    ledcAttach(PWM_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);
}

void setMotor::setPWM(int16_t PWM)
{   
    if(PWM >= 0)
    {
      digitalWrite(DIR_PIN, 1);
    }
    else
    {
      digitalWrite(DIR_PIN, 0);
      PWM = -1 * PWM;
    }
    ledcWrite(PWM_PIN, PWM);
}

