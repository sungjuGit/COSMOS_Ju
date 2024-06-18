#ifndef _MOTOR_H
#define _MOTOR_H

#include "Pins.h"

class Motor
{
  public:
          void Pin_init();
          /*Measuring_speed*/
          void Encoder_init();
           
          void Control(int AIN1_value,int BIN1_value,int PWM_pin,int speed);
          
          void Stop();

  public:
          static unsigned long encoder_count_right_a;
          static unsigned long encoder_count_left_a;

};



#endif
