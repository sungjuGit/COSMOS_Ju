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

int f_speed = 40, b_speed = 40, t_speed = 30;
int obstacle_min = 40;  // in cm
int turning_time = 1000; // in ms


unsigned long start_prev_time = 0;


void setup() 
{
  
  mymotor.Pin_init();
  mymotor.Encoder_init();
   
  mympu6050.init();
  mympu6050.SetOffset(-5321, 874, 920, -19, 13, 23);  // these offset values should come from your MPU6050 calibration
  
  myultrasonic.Pin_init();

  rgb.initialize();

  Serial.begin(9600);

  delay(1000);

//  rgb.flashYellowColorFront();
//  rgb.simple_blink(250, 4);


  start_prev_time = millis();
  
  myBalanced.Motion_Control(STOP);
  
  myTimer2.init(TIMER);

  

//  rgb.flashGreenColorFront();
//  rgb.simple_blink(250, 4);
  delay(2000);

  myBalanced.Motion_Control(FORWARD);
//  rgb.flashBlueColorFront();
//  rgb.simple_blink(250, 4); 
  delay(1000);

  myBalanced.Motion_Control(BACK);
//  rgb.flashBlueColorback();
//  rgb.simple_blink(250, 4);
  delay(1000);

//  myBalanced.Motion_Control(STOP);
//  delay(2000);
//  
//  myBalanced.Motion_Control(LEFT);
//  rgb.flashBlueColorLeft();
//  rgb.simple_blink(250, 4);
//  delay(1000);
//
//  myBalanced.Motion_Control(STOP);
//  delay(2000);
//  
//  myBalanced.Motion_Control(FORWARD);
//  rgb.flashBlueColorFront();
//  rgb.simple_blink(250, 4);
//  delay(1000);
//
//  myBalanced.Motion_Control(STOP);
//  delay(2000);
//  
//  myBalanced.Motion_Control(RIGHT);
//  rgb.flashBlueColorRight();
//  rgb.simple_blink(250, 4);
//  delay(1000);
//
//  myBalanced.Motion_Control(STOP);
//  delay(2000);
//  
//  myBalanced.Motion_Control(FORWARD);
//  rgb.flashBlueColorFront();
//  rgb.simple_blink(250, 4);
//  delay(1000);
  
  myBalanced.Motion_Control(STOP);
//  rgb.lightOff();

  
}

void loop() 
{ 
   
   
   //myfunction.Obstacle_Mode();
   //rgb.blink(100);

   
}
