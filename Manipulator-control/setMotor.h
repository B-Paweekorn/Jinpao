#ifndef SETMOTOR_H
#define SETMOTOR_H

#include <inttypes.h>
#include <Arduino.h>
#include <esp32-hal-ledc.h>

class setMotor
{
    private:
      uint8_t PWM_PIN;  
      uint8_t DIR_PIN;
      uint32_t MOTOR_BASE_FREQ; //PWM frequency
      uint8_t MOTOR_TIMER_14_BIT;  //PWM resolution
    public:
      setMotor(int16_t _PWM_PIN, uint8_t _DIR_PIN, uint32_t _MOTOR_BASE_FREQ, uint8_t _MOTOR_TIMER_14_BIT);  //constructor
      void begin(); //set pin mode
      void setPWM(int16_t PWM); //set motor's PWM 
};
#endif