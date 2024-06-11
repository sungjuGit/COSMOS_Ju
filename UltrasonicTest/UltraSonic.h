#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_

#include <Arduino.h>

#define ECHO_PIN 17
#define TRIG_PIN 11

#define Distance_MIN 3
#define Distance_MAX 100

#define DISTANCE_JUDGEMENT (distance_value > Distance_MIN && distance_value < Distance_MAX)


class Ultrasonic
{
  public:
    void Pin_init();
    void Get_Distance();
    void Check();
    static void Distance_Measure();

  public:
    static char measure_flag;
    static unsigned long measure_prev_time;
    unsigned long get_distance_prev_time;
    static double distance_value;

};


#endif
