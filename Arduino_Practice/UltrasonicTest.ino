// connect pin 3 to the trigger pin
// connect pin 4 to the echo Pin
 
int trigPin = 3;
int echoPin = 4;

// variable to store the distance (cm)
int distance_in_cm = 0 ;
 
// function for ultrasonic 
int ultrasonic()
{
  long duration; 
  int distance;
   
  // send out trigger signal
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
 
    // wait for a pulse on the echo pin
  
  duration = pulseIn(echoPin, HIGH);
  
    // take the pulse and scale it to cm
  distance = duration / 59;
   
  // if distance is less than two or greater than 300, something is wrong.  Return an error.
  
  if ((distance < 2) || (distance> 300)) return false;
 
  return distance;
}
 
 
void setup()
{
  // setup trigger and echo pins
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
 
  digitalWrite(trigPin, LOW);
}


void loop()
{
  distance_in_cm = ultrasonic();
}
