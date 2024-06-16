#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_

#include <Arduino.h>
#include "Balanced.h"

#define Distance_MIN 3
#define Distance_MAX 100

#define DISTANCE_JUDGEMENT (distance_value > Distance_MIN && distance_value < Distance_MAX)

extern int obstacle_min;
extern int turning_time;

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


class Function
{
  public:
    void Obstacle_Mode();

  public:
    unsigned long obstacle_prev_time;
    unsigned long Turning_time_counter;
    unsigned long Obstacle_time;
    unsigned long Back_time;
    int turn_flag = 1;
};





#endif
