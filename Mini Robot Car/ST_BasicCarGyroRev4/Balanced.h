/*
 * @Description: Balanced.h
 * @Author: 
 * @Date: 
 * @LastEditTime: 
 * @LastEditors: 
 */
 
#ifndef _BALANCED_h
#define _BALANCED_h

#include "MsTimer2.h"
#include "EnableInterrupt.h"
#include "KalmanFilter.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Pins.h"

#define MaxPWM 255
#define Balance_angle_MIN -35 //-22;
#define Balance_angle_MAX 35 //22;
#define TIMER 5


enum Direction
{
  FORWARD,
  BACK,
  LEFT,
  RIGHT,
  STOP,
};


int f_speed, b_speed, t_speed;
int obstacle_min;


// For MPU6050
MPU6050 MPU6050;
KalmanFilter kalmanfilter;

int ax, ay, az, gx, gy, gz;
float dt, Q_angle, Q_gyro, R_angle, C_0, K1;
volatile float z_angle;

// For ultrasonic sensor
static char measure_flag = 0;
static unsigned long measure_prev_time = 0;
unsigned long get_distance_prev_time;
static double distance_value;

  
/*Speed value*/
double pwm_left;
double pwm_right;
int encoder_left_pulse_num_speed;
int encoder_right_pulse_num_speed;

/*Cnt*/
int interrupt_cnt;

/*PID parameter*/
         /*PD_VerticalRing*/
double kp_balance, kd_balance;
         /*PI_SpeedRing*/
double kp_speed, ki_speed;
         /*PI_SteeringRing*/
double kp_turn, ki_turn;

double speed_filter;
double speed_filter_old;            
double car_speed_integral;
   
double balance_control_output;
double speed_control_output;
double rotation_control_output;

int setting_turn_speed;
int setting_car_speed;


// Turning related

float targetAngle;

unsigned long obstacle_prev_time;
int turn_flag;



void Timer2_init(int time);
void Timer2_interrupt();


void Motor_Pin_init();
void Motor_Stop();
void Motor_Control(int PIN, int PIN_value, int PWM_pin, int speed);

void Encoder_init();
static unsigned long encoder_count_right_a;
static void EncoderCountRightA();
static unsigned long encoder_count_left_a;
static void EncoderCountLeftA();

void Set_PID_Gains();
void PI_SpeedRing();
void PD_VerticalRing();
void PI_SteeringRing();
void Total_Control();
void Get_EncoderSpeed();

void Motion_Control(Direction direction);
void Stop();
void Forward(int speed);
void Back(int speed);
void Left(int speed);
void Right(int speed);

void TurnByAngle(float turn_angle);

void Mpu6050_init();
void Mpu6050_DataProcessing();
void Mpu6050_SetOffset(int16_t ax_offset, int16_t ay_offset, int16_t az_offset, int16_t gx_offset, int16_t gy_offset, int16_t gz_offset);

void Ultrasonic_Pin_init();
void Get_Distance();
static void Distance_Measure();


// Timer Interrupt

void Timer2_init(int time)
{
  MsTimer2::set(time,Timer2_interrupt);
  MsTimer2::start();
}

void Timer2_interrupt()
{ 
  sei();
  Get_EncoderSpeed();
  
  Mpu6050_DataProcessing();
  
  PD_VerticalRing();
  
  interrupt_cnt++;
  if(interrupt_cnt > 8)
  {
    interrupt_cnt=0;
    PI_SpeedRing();
    PI_SteeringRing();
   }
  Total_Control();
}


// Motor related
void Motor_Pin_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
}

void Motor_Stop()
{
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void Motor_Control(int PIN, int PIN_value, int PWM_pin, int speed)
{
  digitalWrite(PIN, PIN_value);
  analogWrite(PWM_pin, speed);
}


// Encoder related
void Encoder_init()
{
  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, EncoderCountLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, CHANGE);
}

static void EncoderCountRightA()  //For right Wheel Speed
{
  encoder_count_right_a++;
}

static void EncoderCountLeftA()  //For Left Wheel Speed
{
  encoder_count_left_a++;
}

void Get_EncoderSpeed()
{
  encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : 
                                                  encoder_count_left_a;  
  encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : 
                                                    encoder_count_right_a; 
  encoder_count_left_a = 0;  
  encoder_count_right_a = 0; 
}


// PID control related

void PI_SpeedRing()
{
   double car_speed=(encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
   
   encoder_left_pulse_num_speed = 0;
   encoder_right_pulse_num_speed = 0;
   
   speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
   speed_filter_old = speed_filter;

   car_speed_integral += speed_filter;
   car_speed_integral += -setting_car_speed; 
   car_speed_integral = constrain(car_speed_integral, -3000, 3000);

   speed_control_output = kp_speed * speed_filter + ki_speed * car_speed_integral;
   
}

void PD_VerticalRing()
{
  balance_control_output= kp_balance * (kalmanfilter.angle - 0) + kd_balance * (kalmanfilter.Gyro_x - 0);
}

void PI_SteeringRing()
{  
  
  rotation_control_output = setting_turn_speed + kp_turn * (setting_turn_speed - kalmanfilter.Gyro_z);
  
  if ( (setting_turn_speed == 0) && (targetAngle == 0) ) {  // going straight or stopping
    rotation_control_output += ki_turn * (0.0 - z_angle);
  }
  
}

void Total_Control()
{

  pwm_left = balance_control_output + speed_control_output + rotation_control_output;
  pwm_right = balance_control_output + speed_control_output - rotation_control_output;

  pwm_left = constrain(pwm_left, -MaxPWM, MaxPWM);
  pwm_right = constrain(pwm_right, -MaxPWM, MaxPWM);

  while(kalmanfilter.angle < Balance_angle_MIN || Balance_angle_MAX < kalmanfilter.angle) 
  { 
    Motor_Stop();
  }
  
  (pwm_left < 0) ?  (Motor_Control(AIN1,1,PWMA_LEFT,-pwm_left)):
                    (Motor_Control(AIN1,0,PWMA_LEFT,pwm_left));
  
  (pwm_right < 0) ? (Motor_Control(BIN1,1,PWMB_RIGHT,-pwm_right)): 
                    (Motor_Control(BIN1,0,PWMB_RIGHT,pwm_right));

}


// Motion control related

void Motion_Control(Direction direction)
{
  switch(direction)
  {
    case STOP:
                  Stop(); break;
    case FORWARD:
                  Forward(f_speed); break;
    case BACK:
                  Back(b_speed); break;
    case LEFT:
                  Left(t_speed); break;
    case RIGHT:
                  Right(t_speed); break;
    default:      
                  Stop(); break;
  }
}

void Stop()
{
  setting_car_speed = 0;
  setting_turn_speed = 0;
}

void Forward(int speed)
{
  setting_car_speed = speed;
  setting_turn_speed = 0;
}

void Back(int speed)
{
  setting_car_speed = -speed;
  setting_turn_speed = 0;
}

void Left(int speed)
{
  setting_car_speed = 0; 
  setting_turn_speed = -speed;
}

void Right(int speed)
{
  setting_car_speed = 0; 
  setting_turn_speed = speed;
}



// Special turns

void TurnByAngle(float turn_angle)
{
      targetAngle = turn_angle;
      (turn_angle < 0.0) ? Motion_Control(LEFT) : Motion_Control(RIGHT);

      while (abs(targetAngle - z_angle) > 1.0) {
        delay(1);
      }
      
      z_angle = 0;
      targetAngle = 0;
      Motion_Control(STOP);  
}



/// MPU6050 related

void Mpu6050_init()
{
   Wire.begin();         
   MPU6050.initialize();

   dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
}


void Mpu6050_DataProcessing()
{  
  MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter

  z_angle += kalmanfilter.Gyro_z * dt;
  if (z_angle <= -180.0) {
    z_angle += 360.0; 
  }
  else if (z_angle > 180.0) {
    z_angle -= 360.0;
  }
}

void Mpu6050_SetOffset(int16_t ax_offset, int16_t ay_offset, int16_t az_offset, int16_t gx_offset, int16_t gy_offset, int16_t gz_offset)
{  
    MPU6050.setXAccelOffset(ax_offset);
    MPU6050.setYAccelOffset(ay_offset);
    MPU6050.setZAccelOffset(az_offset);

    MPU6050.setXGyroOffset(gx_offset);
    MPU6050.setYGyroOffset(gy_offset);
    MPU6050.setZGyroOffset(gz_offset);
}


// Ultrasonic sensor related

void Ultrasonic_Pin_init()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}

void Get_Distance()
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

static void Distance_Measure()
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
#endif
