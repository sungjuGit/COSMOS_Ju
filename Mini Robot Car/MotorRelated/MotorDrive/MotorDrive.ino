#include "EnableInterrupt.h"
#include <util/atomic.h>
#define ENCODER_LEFT_A_PIN 2

volatile unsigned long encoder_count_left_a;

/*Motor pin*/
#define AIN1 7
#define PWMA_LEFT 5
#define BIN1 12
#define PWMB_RIGHT 6
#define STBY_PIN 8

void setup() {

  // Encoder initialization
  pinMode(ENCODER_LEFT_A_PIN, INPUT);
  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, encoderCountLeftA, CHANGE);
 
  // Motor drive initialization
  pinMode(AIN1, OUTPUT);
  //pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  //pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  digitalWrite(STBY_PIN, HIGH);

  // Serial initialization
  Serial.begin(9600);
}

void loop() {

  int pos = 0;
  int pwm = 200;  // change this value for different rotation speeds

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = encoder_count_left_a;
  }
  Serial.println(pos);

  // Motor drive
  digitalWrite(AIN1, 0);  // change this value to switch rotation direction
  //digitalWrite(BIN1, 0);  // change this value to switch rotation direction
  analogWrite(PWMA_LEFT, pwm);
  //analogWrite(PWMB_RIGHT, pwm);
}

// Interrupt callback function
static void encoderCountLeftA()
{
    encoder_count_left_a++;
}
