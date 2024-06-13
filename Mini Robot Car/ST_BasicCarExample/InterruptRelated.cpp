#include "Motor.h"
#include "Obstacle.h"
#include "EnableInterrupt.h"
#include "Pins.h"

static void EncoderCountRightA();
static void EncoderCountLeftA();


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
