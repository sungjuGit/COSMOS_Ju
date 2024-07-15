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
long prevT = 0;
float eprev = 0;
float eintegral = 0;

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
 
  int target = 2000;

  // initial guesses for the PID constants
  float kp = 0.5;
  float kd = 0.025;
  float ki = 0.01;

  // Time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = encoder_count_left_a;
  }
 
  // Error
  int e = pos - target;

  // Derivative
  float dedt = (e-eprev)/(deltaT);

  // Integral
  eintegral = eintegral + e*deltaT;

  // Control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // Motor power
  float pwm = fabs(u);
  if( pwm > 255 ){
    pwm = 255;
  }

  // Motor drive
  digitalWrite(AIN1, 0);
  //digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, pwm);
  //analogWrite(PWMB_RIGHT, pwm);

  // store previous error
  eprev = e;
  
  // display
  Serial.print(pos);
  Serial.print("    ");
  Serial.print(pwm);
  Serial.print("    ");
  Serial.println(deltaT);
}

// Interrupt callback function
static void encoderCountLeftA()
{
    encoder_count_left_a++;
}
