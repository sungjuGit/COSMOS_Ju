/*
 * @Description: Balanced.cpp
 * @Author: 
 * @Date: 
 * @LastEditTime: 
 * @LastEditors: 
 */
 
#include "Balanced.h"
#include "Motor.h"
#include "Obstacle.h"

extern MyMpu6050 mympu6050;
extern Motor mymotor;
extern Ultrasonic myultrasonic;
extern Balanced myBalanced;

MPU6050 MPU6050;
extern KalmanFilter kalmanfilter;


//****************************************//
extern float targetAngle;
//****************************************//


void Timer2::init(int time)
{
  MsTimer2::set(time,interrupt);
  MsTimer2::start();
}


void Timer2::interrupt()
{ 
  sei();
  myBalanced.Get_EncoderSpeed();
  
  mympu6050.DataProcessing();
  
  myBalanced.PD_VerticalRing();
  
  myBalanced.interrupt_cnt++;
  if(myBalanced.interrupt_cnt > 8)
  {
    myBalanced.interrupt_cnt=0;
    myBalanced.PI_SpeedRing();
    myBalanced.PI_SteeringRing();
   }
  myBalanced.Total_Control();
  
  myultrasonic.Get_Distance();
}


Balanced::Balanced()
{
  kp_balance = 55, kd_balance = 0.75;
  kp_speed = 10, ki_speed = 0.26;
  kp_turn = 0.5;
}


void Balanced::PI_SpeedRing()
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

void Balanced::PD_VerticalRing()
{
  balance_control_output= kp_balance * (kalmanfilter.angle - 0) + kd_balance * (kalmanfilter.Gyro_x - 0);
}

void Balanced::PI_SteeringRing()
{  
   rotation_control_output = setting_turn_speed + kp_turn * (setting_turn_speed - kalmanfilter.Gyro_z);
}


void Balanced::Total_Control()
{

  pwm_left = balance_control_output + speed_control_output + rotation_control_output;
  pwm_right = balance_control_output + speed_control_output - rotation_control_output;


//****************************************//

  if ( (setting_turn_speed == 0) ) {    // when not turning
    if (mympu6050.z_angle > targetAngle + 1.0) {
      pwm_left -= 1;
      pwm_right += 1;
    }
    else if (mympu6050.z_angle < targetAngle - 1.0) {
      pwm_left += 1;
      pwm_right -= 1;
    }
  }

//****************************************//

  
  pwm_left = constrain(pwm_left, -MaxPWM, MaxPWM);
  pwm_right = constrain(pwm_right, -MaxPWM, MaxPWM);

  while(kalmanfilter.angle < Balance_angle_MIN || Balance_angle_MAX < kalmanfilter.angle) 
  { 
    mympu6050.DataProcessing();
    mymotor.Stop();
  }
  
  (pwm_left < 0) ?  (mymotor.Control(AIN1,1,PWMA_LEFT,-pwm_left)):
                    (mymotor.Control(AIN1,0,PWMA_LEFT,pwm_left));
  
  (pwm_right < 0) ? (mymotor.Control(BIN1,1,PWMB_RIGHT,-pwm_right)): 
                    (mymotor.Control(BIN1,0,PWMB_RIGHT,pwm_right));

}

void Balanced::Get_EncoderSpeed()
{
  encoder_left_pulse_num_speed += pwm_left < 0 ? -mymotor.encoder_count_left_a : 
                                                  mymotor.encoder_count_left_a;  
  encoder_right_pulse_num_speed += pwm_right < 0 ? -mymotor.encoder_count_right_a : 
                                                    mymotor.encoder_count_right_a; 
  mymotor.encoder_count_left_a = 0;  
  mymotor.encoder_count_right_a = 0; 
}

void Balanced::Motion_Control(Direction direction)
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

void Balanced::Stop()
{
  setting_car_speed = 0;
  setting_turn_speed = 0;
}

void Balanced::Forward(int speed)
{
  setting_car_speed = speed;
  setting_turn_speed = 0;
}

void Balanced::Back(int speed)
{
  setting_car_speed = -speed;
  setting_turn_speed = 0;
}

void Balanced::Left(int speed)
{
  setting_car_speed = 0; 
  setting_turn_speed = -speed;
}

void Balanced::Right(int speed)
{
  setting_car_speed = 0; 
  setting_turn_speed = speed;
}



void MyMpu6050::init()
{
   Wire.begin();         
   MPU6050.initialize();    
}

MyMpu6050::MyMpu6050()
{
    dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
}

void MyMpu6050::DataProcessing()
{  
  MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter

//****************************************//
  z_angle += kalmanfilter.Gyro_z * dt;

  if (z_angle < -180.0) {
    z_angle += 360.0; 
  }
  else if (z_angle > 180.0) {
    z_angle -= 360.0;
  }
//****************************************//
  
}

void MyMpu6050::SetOffset(int16_t ax_offset, int16_t ay_offset, int16_t az_offset, int16_t gx_offset, int16_t gy_offset, int16_t gz_offset)
{  
    MPU6050.setXAccelOffset(ax_offset);
    MPU6050.setYAccelOffset(ay_offset);
    MPU6050.setZAccelOffset(az_offset);

    MPU6050.setXGyroOffset(gx_offset);
    MPU6050.setYGyroOffset(gy_offset);
    MPU6050.setZGyroOffset(gz_offset);
}
