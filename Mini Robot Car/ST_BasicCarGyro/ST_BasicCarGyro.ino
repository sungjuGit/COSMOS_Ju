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
#include "Rgb.h"


MyMpu6050 mympu6050;
Motor mymotor;
Ultrasonic myultrasonic;
Balanced myBalanced;
Function myfunction;
Timer2 myTimer2;

KalmanFilter kalmanfilter;

int f_speed = 70, b_speed = 70, t_speed = 30;
int obstacle_min = 40;  // in cm
int turning_time = 1000; // in ms

float targetAngle = 0.0;


unsigned long start_prev_time = 0;


void setup() 
{
  
  mymotor.Pin_init(); 
  mymotor.Encoder_init();
   
  mympu6050.init();
  mympu6050.SetOffset(-5321, 874, 920, -19, 13, 23);  // these offset values should come from your MPU6050 calibration
  
  myultrasonic.Pin_init();

  rgb.initialize();

  Serial.begin(1000000);

  delay(1000);

  
  start_prev_time = millis();
  
  myBalanced.Motion_Control(STOP);
  
  myTimer2.init(TIMER);


  delay(2000);
  
  mympu6050.z_angle = 0;  // reset z_angle



// Example motion sequence  

  f_speed = 70;
  targetAngle = 0.0;
  myBalanced.Motion_Control(FORWARD);
  delay(5000);

  myBalanced.Motion_Control(STOP);
  delay(1000);

  b_speed = 70;
  targetAngle = 0.0;
  myBalanced.Motion_Control(BACK);
  delay(5000);

  myBalanced.Motion_Control(STOP);
  delay(1000);

  t_speed = 30;
  targetAngle = -90.0;
  myBalanced.Motion_Control(LEFT);
  while (abs(targetAngle - mympu6050.z_angle) > 1.0) {
    delay(1); // to give time for interrupts
  }

  myBalanced.Motion_Control(STOP); 
  delay(1000);

  t_speed = 30;
  targetAngle = 90.0;
  myBalanced.Motion_Control(LEFT);
  while (abs(targetAngle - mympu6050.z_angle) > 1.0) {
    delay(1); // to give time for interrupts
  }

  myBalanced.Motion_Control(STOP);
 
}



void loop() 
{ 
     
   //myfunction.Obstacle_Mode();
   //rgb.blink(100);

   
}
