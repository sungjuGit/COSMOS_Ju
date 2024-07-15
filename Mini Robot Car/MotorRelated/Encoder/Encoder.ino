#include "EnableInterrupt.h"
#define ENCODER_LEFT_A_PIN 2

volatile unsigned long encoder_count_left_a;

void setup() {
  pinMode(ENCODER_LEFT_A_PIN, INPUT);
  Serial.begin(9600);
  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, encoderCountLeftA, CHANGE);
}

void loop() {
  Serial.println(encoder_count_left_a);
}

static void encoderCountLeftA()
{
    encoder_count_left_a++;
}
