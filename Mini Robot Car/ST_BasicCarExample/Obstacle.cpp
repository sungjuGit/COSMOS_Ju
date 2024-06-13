#include "Obstacle.h"
#include "Pins.h"

extern Ultrasonic myultrasonic;
extern Balanced myBalanced;

char Ultrasonic ::measure_flag = 0;
unsigned long Ultrasonic ::measure_prev_time = 0;
double Ultrasonic ::distance_value;



void Ultrasonic::Pin_init()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}


void Function::Obstacle_Mode()
{
  
  myultrasonic.Get_Distance();
  
  if (millis() - obstacle_prev_time >= 100)
  {
    obstacle_prev_time = millis();
    myBalanced.Motion_Control(FORWARD);

    myultrasonic.Get_Distance();
    //Serial.println(myultrasonic.distance_value);
      
    while (myultrasonic.distance_value < obstacle_min)
    {
      myBalanced.Motion_Control(STOP);

      if (millis() - Obstacle_time > 5000)
      { 
        Obstacle_time = millis();
        Back_time = millis();
        while (millis() - Back_time < 500)
        {
          myBalanced.Motion_Control(BACK);
        }
      }

      Turning_time_counter = millis();
      while (millis() - Turning_time_counter < turning_time)
      {
        if (turn_flag)
        { 
          turn_flag = 0;
          myBalanced.Motion_Control(LEFT);
        }
      }
      turn_flag = 1;


      myultrasonic.Get_Distance();
      //Serial.println(myultrasonic.distance_value);
      
    }

  }
    Obstacle_time = millis();
}
