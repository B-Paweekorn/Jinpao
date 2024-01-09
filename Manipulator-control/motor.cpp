#include "motor.h"
#include "QEI.h"

Motor::Motor(){

}

void Motor::init(){

}
void Motor::cascade(float estimateVel){
  curT = micros();
  if (temp == 1){
    prevT = curT;
    tim = 0;
    temp = 0;
  }
  if (curT - prevT >= 1000){
    tim = tim + ((curT-prevT));
    if (tim/1000000.0 <= t3){
      trapezoidal_trajectory(tim/1000000.0);
    } 
    // // feed forward + velocity control
    // pos = 0;
    // noInterrupts();
    // pos = (cnt*(360.0/3072.0));
    // interrupts();
    
    float deltaT = (1000)*0.000001;
    prevT = curT;


    // kf1_estimateStateValue = kf1.EstimateSpeed(pos, u_velocity);
    
    float pwm_velocity = (0.0039*v*v) - (0.1566*v);
    // Compute the control signal u
    float kp_velocity = 1000;
    float ki_velocity = 0;
    float e_velocity = v - estimateVel; 
  
    eintegral_velocity = eintegral_velocity + e_velocity * deltaT;
    u_velocity = kp_velocity * e_velocity + ki_velocity * eintegral_velocity;    // Compute the control signal u

    if(u_velocity > 16383){
      u_velocity = 16383;
    }
    else if(u_velocity < -16383) {
      u_velocity = -16383;
    }
    //setmotor(dir_velocity,pwr_velocity,pwm,in2,in1);
    
    //position control
    // float pos_position = 0;
    // noInterrupts();
    // pos_position = (cnt*(360.0/3072.0));
    // interrupts();
    
    float kp_position = 0.; // 6 0.1 0.09
    float kd_position = 0;
    float ki_position = 0;
  
    float e_position = posit - 0;
    float dedt_position = (e_position-eprev_position)/deltaT;
    eintegral_position = eintegral_position + e_position * deltaT;
    
    float u_position = kp_position * e_position + ki_position * eintegral_position + kd_position * dedt_position; //u = velocity
  }
}

void Motor::timing(float ip, float tp, float mv,float ma){
  initial_position = ip;
  target_position = tp;
  max_velocity = mv;
  max_acceleration = ma;
  distance = abs(target_position - initial_position);
  t1 = max_velocity/max_acceleration;
  if (max_acceleration*t1*t1 >= distance){
    t1 = sqrt(distance/max_acceleration);
    t2 = t1;
    t3 = 2*t1;    
  }
  else{
    t3 = 2*t1 + ((distance-(max_acceleration*t1*t1))/max_velocity);
    t2 = t3 - t1; 
  }
}

void Motor::trapezoidal_trajectory(float t){  
  if (t <= t1 && t+0.001 >= t1){
      if (target_position > initial_position){
          tempS = initial_position+(0.5*max_acceleration*t*t);        
      }
      else{
          tempS = initial_position-(0.5*max_acceleration*t*t);        
      } 
      tempV = max_acceleration*t;
      tempT = t;    
  }

  if (t <= t2 && t+0.001 >= t2){
      if (target_position > initial_position){
          tempS = tempS + tempV*(t-tempT);        
      }
      else{
          tempS = tempS - tempV*(t-tempT);        
      }
      tempT = t;    
  }

  if (t<t1){
      v = max_acceleration*t;
      s = (0.5*max_acceleration*t*t);
      if (target_position > initial_position){
          posit = initial_position+s;        
      }  
      else{
          posit = initial_position-s;         
      }   
  }

  else if (t >= t1 && t <= t2){
      t = t - tempT;
      v = tempV;
      s = (tempV*t);
      if (target_position > initial_position){
          posit = tempS+s;        
      }  
      else{
          posit = tempS-s;        
      }   
  }
  
  else if (t > t2){
      t = t - tempT;
      v = tempV - (max_acceleration*t);
      s = (tempV*t)-(0.5*max_acceleration*t*t);
      if (target_position > initial_position){
          posit = tempS+s;        
      }  
      else{
          posit = tempS-s;         
      }  
  }
}

// void Motor::setmotor(int dir, int pwmVal, int pwm, int in1, int in2){
//   analogWrite(pwm,pwmVal);
//   if (dir == 1){
//     digitalWrite(in1,HIGH);
//     digitalWrite(in2,LOW);
//   }
//   else if(dir == -1){
//     digitalWrite(in2,HIGH);
//     digitalWrite(in1,LOW);
//   }
//   else{
//     digitalWrite(in1,LOW);
//     digitalWrite(in2,LOW);    
//   }
// }