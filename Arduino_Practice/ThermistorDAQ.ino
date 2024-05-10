/*
  ThermistorDAQ

  Reads an analog input on pin 2, converts it to temperature, and prints the result to the Serial Monitor.

*/

int interval = 10;           // interval in milliseconds at which to take analog reading
int num_samples = 2000;        // number of samples

// Define Fixed Variables
const double V_ref = 3.3;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for a variable of type int to store   
unsigned long currentMicros;



// For Thermistor Only ***********

// Steinhart and Hart Coefficients
const double SH_COEFF_1 = 0.00116741481796827;
const double SH_COEFF_2 = 0.000227371846448138;
const double SH_COEFF_3 = 0.000000119663032602201;

const double R_ref = 10000.0; //Reference resistor used in the voltage divider (ohms)
double Temperature;



// the setup routine runs once when you press reset:
void setup() {

  delay(1000);
  
  // initialize serial communication at XXX bits per second:
  Serial.begin(250000);
  
  analogReference(EXTERNAL);  // When using 3.3V or another external reference voltage connected to Aref

  int i=0;

 
  while (i <= num_samples) {
    
    currentMicros = micros();   
    Temperature = Reading_To_Temp(analogRead(A2));

    Serial.print(currentMicros/1000000.0,4);
    Serial.print("\t");
    Serial.println(Temperature);
  
    i++;

    delay(interval);
    
    
  }
 
}




// the loop routine runs over and over again forever:
void loop() {

// I often don't put anything here (I have a better control by my own for-loop in setup() )

}



// Converts analog reading into temperature

double Reading_To_Temp(int Reading) {
  
  // Convert Reading to a Voltage
  double VA2 = Reading * (V_ref/1023.0);

  // Find Resistance Based on Voltage Divider Eq
  double log_Resistance = log( (VA2 / (V_ref - VA2)) * R_ref );

  // Use Steinhart_Hart Coeffs to Find Temp
  double Tempera = 1.0/(SH_COEFF_1 + (SH_COEFF_2 * log_Resistance) + (SH_COEFF_3 * pow(log_Resistance,3)));
      
  // Convert K to C
  Tempera = Tempera - 273.15;

  return Tempera;
        
}
