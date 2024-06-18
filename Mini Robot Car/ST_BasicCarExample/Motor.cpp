/*
 * @Description: Motor.cpp
 * @Author: 
 * @Date: 
 * @LastEditTime: 
 * @LastEditors: 
 */
 
#include <Arduino.h>
#include "Motor.h"

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
