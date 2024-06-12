
#include "UltraSonic.h"


Ultrasonic myultrasonic;


void setup()
{

  myultrasonic.Pin_init();

  Serial.begin(9600);
  
}


void loop()
{

  myultrasonic.Get_Distance();
  
  Serial.println(myultrasonic.distance_value);

  delay(250);
  
}
