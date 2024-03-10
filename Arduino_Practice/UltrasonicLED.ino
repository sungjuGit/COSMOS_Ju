// connect pin 3 to the trigger pin
// connect pin 4 to the echo Pin
 
int trigPin = 3;
int echoPin = 4;

// variable to store the distance (cm)
int distance_in_cm = 0;
 
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

  pinMode(7, OUTPUT);
}
 
void loop()
{
  distance_in_cm = ultrasonic();

  // test if object is more than 100 cm away
  if (distance_in_cm > 100){
    digitalWrite(7,HIGH);
    delay(2000);
    digitalWrite(7,LOW);
    delay(2000);
  }
  // test if object is between 50 cm and 100 cm
  else if (distance_in_cm > 50) {
    digitalWrite(7,HIGH);
    delay(1000);
    digitalWrite(7,LOW);
    delay(1000);
  }  
  // test if object is between 25 cm and 50 cm
  else if (distance_in_cm > 25) {
    digitalWrite(7,HIGH);
    delay(500);
    digitalWrite(7,LOW);
    delay(500);
  } 
  // test if object is between 10 cm and 25 cm
  else if (distance_in_cm > 10) {
    digitalWrite(7,HIGH);
    delay(200);
    digitalWrite(7,LOW);
    delay(200);
  } 
  // test if object is between 5 cm and 10 cm
  else if (distance_in_cm > 5) {
    digitalWrite(7,HIGH);
    delay(100);
    digitalWrite(7,LOW);
    delay(100);
  } 
  // if we get to here, the object is less than 5 cm away
  else {
    digitalWrite(7,HIGH);
    delay(50);
    digitalWrite(7,LOW);
    delay(50);
  } 
  
  delay(10);

}
