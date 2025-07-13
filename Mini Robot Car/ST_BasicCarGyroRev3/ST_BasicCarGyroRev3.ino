/*
 * @Description: ST_BasicCarExample.ino
 * @Author: 
 * @Date: 
 * @LastEditTime: 
 * @LastEditors: 
 */
 
#include "Motor.h"
#include "Balanced.h"
#include "Obstacle.h"
#include "Pins.h"

MyMpu6050 mympu6050;
Motor mymotor;
Ultrasonic myultrasonic;
Balanced myBalanced;
Function myfunction;
Timer2 myTimer2;

KalmanFilter kalmanfilter;

int f_speed = 30, b_speed = 30, t_speed = 30;
int obstacle_min = 30;  // in cm

unsigned long start_prev_time = 0;


void setup() 
{
  
  mymotor.Pin_init(); 
  mymotor.Encoder_init();
   
  mympu6050.init();
  mympu6050.SetOffset(-5321, 874, 920, -19, 13, 23);  // these offset values should come from your MPU6050 calibration
  
  myultrasonic.Pin_init();

  Serial.begin(9600);

  start_prev_time = millis();
  
  myBalanced.Motion_Control(STOP);
  
  myTimer2.init(TIMER);

  delay(1000); 
  mympu6050.z_angle = 0;  // reset z_angle

  
  myBalanced.Motion_Control(FORWARD);
  delay(5000);
  
  myfunction.Left90Turn();
  delay(1000);

  myfunction.Left90Turn();
  delay(1000);
  
  myBalanced.Motion_Control(FORWARD);
  delay(5000);

  myBalanced.Motion_Control(STOP);
  
}



void loop() 
{ 
     
   //myfunction.Maze_Mode();


  if (Serial.available())
  {
    
    Serial.print("received");
    char data = Serial.read();
    Serial.print("data:"); Serial.println(data);
    
    if (data != '\0' && data != '\n')
    {
      switch (data)
      {
        case '1': myBalanced.Motion_Control(FORWARD); delay(1000); myBalanced.Motion_Control(STOP); break;

        case '2': myBalanced.Motion_Control(BACK); delay(1000); myBalanced.Motion_Control(STOP); break;

        case '3': myfunction.Left90Turn(); break;

        case '4': myfunction.Right90Turn(); break;

        default: myBalanced.Motion_Control(STOP);
      }
    }
    
  }

   
}
