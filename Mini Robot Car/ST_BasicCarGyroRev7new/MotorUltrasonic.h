#ifndef _MOTORULTRASONIC_H
#define _MOTORULTRASONIC_H

#include <Arduino.h>
#include "Balanced.h"
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

static void EncoderCountRightA();
static void EncoderCountLeftA();


class Ultrasonic
{
  public:
    void Pin_init();
    void Get_Distance();
    
    static void Distance_Measure();

  public:
    static char measure_flag;
    static unsigned long measure_prev_time;
    unsigned long get_distance_prev_time;
    
    static double distance_value;

};

void TurnByAngle(float turn_angle);

#endif
