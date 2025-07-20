/*
 * @Description: Motor.cpp
 * @Author: 
 * @Date: 
 * @LastEditTime: 
 * @LastEditors: 
 */
 
#include <Arduino.h>
#include "MotorUltrasonic.h"
#include "EnableInterrupt.h"


// Motor Related

void Motor::Encoder_init()
{
  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, EncoderCountLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, CHANGE);
}


//Getting Right Wheel Speed

unsigned long Motor::encoder_count_right_a;

static void EncoderCountRightA()
{
  Motor::encoder_count_right_a++;
}


//Getting Left Wheel Speed

unsigned long Motor::encoder_count_left_a;

static void EncoderCountLeftA()
{
  Motor::encoder_count_left_a++;
}


void Motor::Pin_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
}

void Motor::Stop()
{
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}


void Motor::Control(int PIN, int PIN_value, int PWM_pin, int speed)
{
  digitalWrite(PIN, PIN_value);
  analogWrite(PWM_pin, speed);
}



// Ultrasonic Sensor Related

char Ultrasonic::measure_flag = 0;
unsigned long Ultrasonic::measure_prev_time = 0;
double Ultrasonic::distance_value = 0;
unsigned long get_distance_prev_time = 0;


void Ultrasonic::Pin_init()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}

void Ultrasonic::Get_Distance()
{

  if (millis() - get_distance_prev_time > 80)
  {
    delayMicroseconds(1);
    get_distance_prev_time = millis();
     
    measure_flag = 0;
    
    enableInterrupt(ECHO_PIN, Distance_Measure, RISING);
    
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(TRIG_PIN, LOW);
  }
  
}


void Ultrasonic::Distance_Measure()
{
  if (measure_flag == 0)
  {
    measure_prev_time = micros();
    enableInterrupt(ECHO_PIN, Distance_Measure, FALLING);
    measure_flag = 1;
  }
  else if (measure_flag == 1)
  {
    distance_value = (micros() - measure_prev_time) * 0.017; //340.29 m/s / 2 -> (340.29*100 cm) /(1000*1000 us) / 2 = 0.0170145
    measure_flag = 2;
  }
  
}


// Turn by Angle

extern Balanced myBalanced;
extern MyMpu6050 mympu6050;

float targetAngle = 0.0;

void TurnByAngle(float turn_angle)
{
      targetAngle = turn_angle;
      (turn_angle < 0.0) ? myBalanced.Motion_Control(LEFT) : myBalanced.Motion_Control(RIGHT);

      while (abs(targetAngle - mympu6050.z_angle) > 0.2) {
        delay(1);
      }
      
      mympu6050.z_angle = 0;
      targetAngle = 0;
      myBalanced.Motion_Control(STOP);  
}
