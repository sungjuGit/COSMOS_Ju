#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include <Arduino.h>  // necessary as we have .cpp files as part of the code

#define ECHO_PIN 17
#define TRIG_PIN 11

class Ultrasonic
{
  public:
    void Pin_init();
    void Get_Distance();
    void Check();
    static void Distance_Measure();  // ISR needs to be a static function when defined for a class

  public:
    static char measure_flag;  // variable in static functions needs to be static
    static unsigned long measure_prev_time;
    static double distance_value;
    unsigned long get_distance_prev_time;

};

#endif
