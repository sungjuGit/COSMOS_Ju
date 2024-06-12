#include "I2Cdev.h" 
#include "MPU6050.h"
#include "KalmanFilter.h"

int accel_or_angle = 1;

MPU6050 MPU6050;
KalmanFilter kalmanfilter;

class MyMpu6050
{
  public:
          void init();
          void DataProcessing();
          MyMpu6050();

  public:
         int ax, ay, az, gx, gy, gz;
         float dt, Q_angle, Q_gyro, R_angle, C_0, K1;
};


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
  MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);// Data acquisition of MPU6050 gyroscope and accelerometer
  
  if (accel_or_angle == 0)
  {
    Serial.print(ax);  Serial.print(" ");
    Serial.print(ay);  Serial.print(" ");
    Serial.print(az);  Serial.print(" ");
    Serial.print(gx);  Serial.print(" ");
    Serial.print(gy);  Serial.print(" ");
    Serial.println(gz);
  }
  else
  {
    double Angle = atan2(ay , az) * 57.3;
    Serial.print(Angle);
    Serial.print(" ");
  
    kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
    Serial.println(kalmanfilter.angle);
  }
}



MyMpu6050 mympu6050;

void setup() {
  mympu6050.init();
  Serial.begin(9600);
}


void loop() {
  mympu6050.DataProcessing();
  delay(100);
}
