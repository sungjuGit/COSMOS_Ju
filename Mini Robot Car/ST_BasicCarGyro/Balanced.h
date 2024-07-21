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
#include "KalmanFilter.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define MaxPWM 255
#define Balance_angle_MIN -35 //-22;
#define Balance_angle_MAX 35 //22;


extern int f_speed, b_speed, t_speed;
extern char balance_angle_min;
extern char balance_angle_max;

enum Direction
{
  FORWARD,
  BACK,
  LEFT,
  RIGHT,
  STOP,
};

class Balanced
{
  public:
          Balanced();
          void Get_EncoderSpeed();
          void PD_VerticalRing();
          void PI_SpeedRing();
          void PI_SteeringRing();
          void Total_Control();

          void Motion_Control(Direction direction);
          void Stop();
          void Forward(int speed);
          void Back(int speed);
          void Left(int speed);
          void Right(int speed);
  
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
          double kp_turn;

          double speed_filter;
          double speed_filter_old;            
          double car_speed_integral;
   
          double balance_control_output;
          double speed_control_output;
          double rotation_control_output;
          int setting_turn_speed;
          int setting_car_speed;
          
};

class Timer2
{
  public:
          void init(int time);
          static void interrupt();
  private:       
          #define TIMER 5
};


class MyMpu6050
{
  public:
          void init();
          void DataProcessing();
          void SetOffset(int16_t ax_offset, int16_t ay_offset, int16_t az_offset, int16_t gx_offset, int16_t gy_offset, int16_t gz_offset);
          MyMpu6050();

  public:
         int ax, ay, az, gx, gy, gz;
         float dt, Q_angle, Q_gyro, R_angle, C_0, K1;
         
//****************************************//
         float z_angle;
//****************************************//

};



#endif
