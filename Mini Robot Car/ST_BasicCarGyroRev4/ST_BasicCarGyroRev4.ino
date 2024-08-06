/*
 * @Description:
 * @Author: 
 * @Date: 
 * @LastEditTime: 
 * @LastEditors: 
 */
 
#include "Balanced.h"

void setup() 
{

  f_speed = 30; b_speed = 30; t_speed = 30;
  obstacle_min = 30;  // in cm

  kp_balance = 55; kd_balance = 0.75; kp_speed = 10; 
  ki_speed = 0.26; kp_turn = 0.5; ki_turn = 5.0;
   

  Motor_Pin_init(); 
  Encoder_init();
   
  Mpu6050_init();
  Mpu6050_SetOffset(-5321, 874, 920, -19, 13, 23);  // these offset values should come from your MPU6050 calibration
  
  Ultrasonic_Pin_init();

  Serial.begin(9600);
  
  Motion_Control(STOP);
  
  Timer2_init(TIMER);
  
  
  delay(1000); 

  
  Motion_Control(FORWARD);
  delay(1000);
  
  TurnByAngle(-90.0);
  delay(1000);
  
}



void loop() 
{ 
     
   Maze_Mode();
   //Bluetooth_mode();
   
}


// Custom motion modes

void Bluetooth_mode()
{
  
 if (Serial.available())
  {
    
    Serial.print("received");
    char data = Serial.read();
    Serial.print("data:"); Serial.println(data);
    
    if (data != '\0' && data != '\n')
    {
      switch (data)
      {
        case '1': Motion_Control(FORWARD); delay(1000); Motion_Control(STOP); break;

        case '2': Motion_Control(BACK); delay(1000); Motion_Control(STOP); break;

        case '3': TurnByAngle(-90.0); break;

        case '4': TurnByAngle(90.0); break;

        default: Motion_Control(STOP);
      }
    }
    
  }
  
}


void Maze_Mode()
{
  
  Get_Distance();
  
  if (millis() - obstacle_prev_time >= 100)
  {
        
    if ( distance_value < obstacle_min )
    {

      if (turn_flag == 0)
      {
        Motion_Control(STOP);
        turn_flag = 1;     
      }
      else if (turn_flag == 1)
      {
        TurnByAngle(-90.0);
        turn_flag = 2;
      }
      else if (turn_flag == 2)
      {
        TurnByAngle(180.0);
        turn_flag = 0;
      }
      
    }
    else
    {
      Motion_Control(FORWARD);
      turn_flag = 0;
    }

    obstacle_prev_time = millis();
 
 }
  
}
