/*
 * @Description: ST_BasicCarExample.ino
 * @Author: 
 * @Date: 
 * @LastEditTime: 
 * @LastEditors: 
 */
 
#include "MotorUltrasonic.h"
#include "Balanced.h"
#include "Pins.h"


MyMpu6050 mympu6050;
Motor mymotor;
Ultrasonic myultrasonic;
Balanced myBalanced;
Timer2 myTimer2;

KalmanFilter kalmanfilter;


int f_speed = 40, b_speed = 40, t_speed = 30;
int obstacle_min = 30;  // in cm

// for Maze
unsigned long obstacle_prev_time;
int turn_flag = 0;


void setup() 
{
  
  mymotor.Pin_init(); 
  mymotor.Encoder_init();
   
  mympu6050.init();
  mympu6050.SetOffset(-5386, 894, 902, -21, 9, 16);  // these offset values should come from your MPU6050 calibration

  myultrasonic.Pin_init();

  Serial.begin(9600);
  
  myBalanced.Motion_Control(STOP);
  
  myTimer2.init(TIMER);

  delay(1000); 
  mympu6050.z_angle = 0;  // reset z_angle


  
  //myBalanced.Motion_Control(FORWARD);
  //delay(3000);
  //myBalanced.Motion_Control(STOP);
  delay(2000);
  
  for (int i=0; i < 12; i++) {
    TurnByAngle(90.0);
    delay(1000);  
  }
  

//  TurnByAngle(-90.0);
//  delay(1000);
//  
//  
//  myBalanced.Motion_Control(FORWARD);
//  delay(5000);
//
//  myBalanced.Motion_Control(STOP);


}



void loop() 
{ 
     
  //Maze_Mode();
  BluetoothMode();      

}


void Maze_Mode()
{

  myultrasonic.Get_Distance();
  
  if (millis() - obstacle_prev_time > 100)
  {
        
    if ( (myultrasonic.distance_value < obstacle_min) && (myultrasonic.distance_value > 5) )
    {

      if (turn_flag == 0)
      {
        myBalanced.Motion_Control(STOP);
        //delay(1000);
        turn_flag = 1;     
      }
      else if (turn_flag == 1)
      {
        TurnByAngle(-90.0);
        //delay(1000);
        turn_flag = 2;
      }
      else if (turn_flag == 2)
      {
        TurnByAngle(90.0);
        TurnByAngle(90.0);
        //delay(1000);        
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


void BluetoothMode() {

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

        case '3': TurnByAngle(-90.0); break;

        case '4': TurnByAngle(90.0); break;

        case '5': GoStop(); break;
          
        default: myBalanced.Motion_Control(STOP);
      
      }
    }
    
  }
  
}


void GoStop()
{
  int my_flag = 1;
          
  while (my_flag) {

    myultrasonic.Get_Distance();
  
    if (millis() - obstacle_prev_time > 100)
    {
        
      if ( (myultrasonic.distance_value < obstacle_min) && (myultrasonic.distance_value > 5) )
      {
        myBalanced.Motion_Control(STOP);
        delay(1000);
        my_flag = 0;    
      }
      else
      {
        myBalanced.Motion_Control(FORWARD);
      }

    obstacle_prev_time = millis();
 
    }
  }
        
}
