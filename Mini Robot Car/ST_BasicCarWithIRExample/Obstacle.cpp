#include "Obstacle.h"
#include "Pins.h"

extern Ultrasonic myultrasonic;
extern IR myIR;
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

  myIR.Send();
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
          myIR.Check();
          //myBalanced.Motion_Control(LEFT);
        }
      }
      turn_flag = 1;


      myultrasonic.Get_Distance();
      //Serial.println(myultrasonic.distance_value);
      
    }

  }
    Obstacle_time = millis();
}


void IR::Check()
{
  int motion = left_is_obstacle + right_is_obstacle;
  switch (motion)
  {
    case Meet_OBSTACLE: myBalanced.Motion_Control(LEFT);
      left_is_obstacle = 0; break;

    case FOLLOW_RIGHT:
      myBalanced.Motion_Control(RIGHT);
      left_is_obstacle = 0; break;

    case FOLLOW_LEFT:
      myBalanced.Motion_Control(LEFT);
      right_is_obstacle = 0; break;

    case FOLLOW_BACK:
      myBalanced.Motion_Control(LEFT);
      right_is_obstacle = 0;
      left_is_obstacle = 0; break;
  }

}

void IR::Send()
{
  static unsigned long ir_send_time;

  if (millis() - ir_send_time > 15)
  {
    for (int i = 0; i < 39; i++)
    {
      digitalWrite(IR_SEND_PIN, LOW);
      delayMicroseconds(9);
      digitalWrite(IR_SEND_PIN, HIGH);
      delayMicroseconds(9);
    }
    ir_send_time = millis();
  }
}
