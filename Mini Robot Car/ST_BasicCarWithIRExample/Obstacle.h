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


class IR
{
  public:
    void Pin_init();
    void Check();
    void Send();
    void Filter();
    static void Left_Receive();
    static void Right_Receive();


  public:
    static unsigned char left_receive_flag;
    static unsigned int left_count;

    static unsigned char right_receive_flag;
    static unsigned int right_count;

    unsigned long left_count_time = 0;
    static int left_is_obstacle;
    static int right_is_obstacle;

  private:
#define RECV_PIN 9
#define IR_SEND_PIN 9
#define LEFT_RECEIVE_PIN A0
#define RIGHT_RECEIVE_PIN A1
#define If_IR_TRIGGERED (IR.left_is_obstacle || IR.right_is_obstacle)

};

enum FOLLW_MOTION
{
  Meet_OBSTACLE,
  FOLLOW_RIGHT,
  FOLLOW_LEFT,
  FOLLOW_BACK,
};



#endif
