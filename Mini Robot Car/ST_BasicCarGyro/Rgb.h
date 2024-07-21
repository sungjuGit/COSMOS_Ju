/*
 * @Description: RGB.h
 * @Author: 
 * @Date: 
 * @LastEditTime: 
 * @LastEditors: 
 */
 
#include "Adafruit_NeoPixel.h"

class RGB : public Adafruit_NeoPixel
{
public:
  RGB() : Adafruit_NeoPixel(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800) {}
  uint8_t gamma[256] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
      2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
      5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
      10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
      17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
      25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
      37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
      51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
      69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
      90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
      115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
      144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
      177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
      215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255};
  uint32_t led_rgb_new[NUMPIXELS];
  uint32_t led_rgb_old[NUMPIXELS];
  int brightness = 50;
  unsigned long rgb_delay_time = 0;
  unsigned long dispaly_timeout = 0;
  char flag = 0;

  bool rgbDelay(unsigned long wait)
  {
    rgb_delay_time = millis();
    while (millis() - rgb_delay_time < wait)
    {
//      if (getBluetoothData() || getKeyValue())
//      {
//        return true;
//      }
//      if (motion_mode == STOP && balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
//      {
//        motion_mode = STANDBY;
//        lightOff();
//        function_mode = IDLE;
//        return true;
//      }
    }
    return false;
  }



  void initialize()
  {
    begin();
    setBrightness(brightness);
    show();
  }

  
  void setColorNew(unsigned char r0, unsigned char g0, unsigned char b0,
                   unsigned char r1, unsigned char g1, unsigned char b1,
                   unsigned char r2, unsigned char g2, unsigned char b2,
                   unsigned char r3, unsigned char g3, unsigned char b3)
  {
    led_rgb_new[0] = Color(r0, g0, b0);
    led_rgb_new[1] = Color(r1, g1, b1);
    led_rgb_new[2] = Color(r2, g2, b2);
    led_rgb_new[3] = Color(r3, g3, b3);
  }
  
  void setColorOld(unsigned char r0, unsigned char g0, unsigned char b0,
                   unsigned char r1, unsigned char g1, unsigned char b1,
                   unsigned char r2, unsigned char g2, unsigned char b2,
                   unsigned char r3, unsigned char g3, unsigned char b3)
  {
    led_rgb_old[0] = Color(r0, g0, b0);
    led_rgb_old[1] = Color(r1, g1, b1);
    led_rgb_old[2] = Color(r2, g2, b2);
    led_rgb_old[3] = Color(r3, g3, b3);
  }

  
  void blink(unsigned long delay_time)
  {
    if ((millis() - previous_millis < delay_time) && (delay_flag == 0))
    {
      delay_flag = 1;
      for (size_t i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, led_rgb_new[i]);
      }
      show();
    }
    else if ((millis() - previous_millis < delay_time * 2) && (millis() - previous_millis > delay_time) && (delay_flag == 1))
    {
      delay_flag = 2;
      for (size_t i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, led_rgb_old[i]);
      }
      show();
    }
    else if (millis() - previous_millis >= delay_time * 2)
    {
      delay_flag = 0;
      previous_millis = millis();
    }
  }


  void simple_blink(unsigned long delay_time, int times)
  {

    for (int count = 0; count < times; count++) {
      
      for (size_t i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, led_rgb_new[i]);
      }
      show();

      delay(delay_time);
      
      for (size_t i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, led_rgb_old[i]);
      }
      show();

      delay(delay_time);
      
    }
     
  }


  
  void lightOff()
  {
    setColorNew(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  
  void brightRedColor()
  {
    setColorNew(255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0);
    setColorOld(255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0);
  }

  void flashRedColor()
  {
    setColorNew(255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  void brightBlueColor()
  {
    setColorNew(0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255);
    setColorOld(0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255);
  }
  
  void flashBlueColorFront()
  {
    setColorNew(0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 255);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  void flashBlueColorback()
  {
    setColorNew(0, 0, 255, 0, 0, 255, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  void flashBlueColorLeft()
  {
    setColorNew(0, 0, 0, 0, 0, 255, 0, 0, 255, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  void flashBlueColorRight()
  {
    setColorNew(0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 255);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  void brightYellowColor()
  {
    setColorNew(255, 255, 0, 255, 255, 0, 255, 255, 0, 255, 255, 0);
    setColorOld(255, 255, 0, 255, 255, 0, 255, 255, 0, 255, 255, 0);
  }

  void flashYellowColorFront()
  {
    setColorNew(0, 0, 0, 0, 0, 0, 255, 255, 0, 255, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  void flashYellowColorback()
  {
    setColorNew(255, 255, 0, 255, 255, 0, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  void flashYellowColorLeft()
  {
    setColorNew(0, 0, 0, 255, 255, 0, 255, 255, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  void flashYellowColorRight()
  {
    setColorNew(255, 255, 0, 0, 0, 0, 0, 0, 0, 255, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  void brightGreenColor() 
  {
    setColorNew(0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0);
    setColorOld(0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0);
  }
  
  void flashGreenColorFront()
  {
    setColorNew(0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  void flashGreenColorback()
  {
    setColorNew(0, 255, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  
  void flashGreenColorLeft()
  {
    setColorNew(0, 0, 0, 0, 255, 0, 0, 255, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  
  void flashGreenColorRight()
  {
    setColorNew(0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  

private:
  unsigned char delay_flag = 0;
  unsigned long previous_millis = 0;
} rgb;
