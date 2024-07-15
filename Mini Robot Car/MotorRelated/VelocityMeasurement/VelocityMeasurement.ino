#include "EnableInterrupt.h"
#include <util/atomic.h>
#define ENCODER_LEFT_A_PIN 2

/*Motor pin*/
#define AIN1 7
#define PWMA_LEFT 5
#define BIN1 12
#define PWMB_RIGHT 6
#define STBY_PIN 8

volatile unsigned long encoder_count_left_a = 0;


volatile int pos_i = 0;
int posPrev = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;


void setup() {

  // Encoder initialization
  pinMode(ENCODER_LEFT_A_PIN, INPUT);
  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, encoderCountLeftA, CHANGE);
 
  // Motor initialization
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
  float velocity = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = encoder_count_left_a;
    velocity = velocity_i/26/30*60;
  }

  // Motor power
  float pwm = 200;

  // Motor drive
  digitalWrite(AIN1, 0);
  //digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, pwm);
  //analogWrite(PWMB_RIGHT, pwm);

  Serial.println(velocity);
}

// Interrupt callback function
static void encoderCountLeftA()
{
  encoder_count_left_a++;
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = 1/deltaT;
  prevT_i = currT;
}
