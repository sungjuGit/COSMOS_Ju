/*
 * @Description: Obstacle.cpp
 * @Author: 
 * @Date: 
 * @LastEditTime: 
 * @LastEditors: 
 */
 
#include "Obstacle.h"
#include "Pins.h"

extern Ultrasonic myultrasonic;
extern Balanced myBalanced;
extern MyMpu6050 mympu6050;
extern Function myfunction;

char Ultrasonic ::measure_flag = 0;
unsigned long Ultrasonic ::measure_prev_time = 0;
double Ultrasonic ::distance_value;

float targetAngle = 0.0;


void Ultrasonic::Pin_init()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}


void Function::Left90Turn()
{
      targetAngle = -90.0;
      myBalanced.Motion_Control(LEFT);  

      while (abs(targetAngle - mympu6050.z_angle) > 1.0) {
        delay(1);
      }
      
      mympu6050.z_angle = 0;
      targetAngle = 0;
      myBalanced.Motion_Control(STOP); 
  
}


void Function::Right90Turn()
{
      targetAngle = 90.0;
      myBalanced.Motion_Control(RIGHT);  

      while (abs(targetAngle - mympu6050.z_angle) > 1.0) {
        delay(1);
      }
      
      mympu6050.z_angle = 0;
      targetAngle = 0;
      myBalanced.Motion_Control(STOP); 
        
}


void Function::Maze_Mode()
{
  
  myultrasonic.Get_Distance();
  
  if (millis() - obstacle_prev_time >= 100)
  {
        
    if ( myultrasonic.distance_value < obstacle_min )
    {

      if (turn_flag == 0)
      {
        myBalanced.Motion_Control(STOP);
        turn_flag = 1;     
      }
      else if (turn_flag == 1)
      {
        myfunction.Left90Turn();
        turn_flag = 2;
      }
      else if (turn_flag == 2)
      {
        myfunction.Right90Turn();
        myfunction.Right90Turn();
        turn_flag = 0;
      }
      
    }
    else
    {
      myBalanced.Motion_Control(FORWARD);
      turn_flag = 0;
    }

    obstacle_prev_time = millis();
 
 }
  
}
