/*
  Arduino control of a motor
 
 Reads motor position from analog input, compares this to a desired value and computes a control law
 Outputs the control signal through PWM along with a separate digital channel that gives the direction of movement
 
 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * "P" channel on motor driver connected to digital pin 9
 * "D" channel on motor driver connected to digital pin 7
 
 created 26 April 2013
 by UCI
 
 */

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the "P" channel is attached to (PWM Signal)
const int digitalOutPin = 7; // Digital output pin that the "D" channel is attached to (controls motor Direction)

// PID Gains
const int PGain = 2;
const int DGain = 50;
const int IGain = 0.05;

// Input and Output Variables
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

// Variables needed for Control Law
double potAngle = 0;			// value in degrees
double prevPotAngle = 0;		// previous value read from pot
double desiredPosition = 90;		// desired position (in degrees) we want the motor to go to
double error = 0;				// current error
double motorVelocity = 0;		// first derivative of motorPosition
double errorIntegral = 0;		// used in PID control law
double controlSignal = 0;		// control signal sent to the motor

// Time keeping variables
int count = 0;				// keeps track of time (for switching desired position)
int dt = 1;					// time it takes to execute one loop (in ms)


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200); 		// This allows us to write values to the Serial Monitor
  pinMode(digitalOutPin, OUTPUT);	// sets the "D" channel pin to an output
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);  // Read as an integer b/w 0 and 1023 (10-bit resolution)
  
  // First save the previous value
  prevPotAngle = potAngle;
  // then save the new value, mapping it to a value in degrees:
  potAngle = map(sensorValue, 0, 1023, 0, 360); 
  
  // use it to compute error and motor speed
  error = potAngle - desiredPosition;
  motorVelocity = (potAngle - prevPotAngle)/(dt); // (in deg/ms)
  errorIntegral = errorIntegral + error*(dt/1000); 
  
  // Calculate the motor control signal using a PID control law
  controlSignal = -1*PGain*error - DGain*motorVelocity - IGain*errorIntegral;	
  
   // tell the motor which way to spin
  if (controlSignal <= 0) {
	digitalWrite(digitalOutPin, LOW);
  } else {
	digitalWrite(digitalOutPin, HIGH);
  }
    
  // map it to the range of the analog out:
  controlSignal = abs(controlSignal);  // can only ouptut positive values
  outputValue = constrain(controlSignal, 0, 255);  // 8-bit resolution
  
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);         

 

  //print the results to the serial monitor: (UNCOMMENT TO DISPLAY)
  //Serial.print("Desired Position = " );                       
  Serial.print(desiredPosition);      
  //Serial.print("\t Actual Position = ");     
  Serial.print("\t       "); 
  Serial.println(potAngle);   

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(dt);
  count = count + dt;		// increment the counter
  if (count >= 500) {		// every 500 ms, change the desired position of the motor (simulates a square wave input)
	count = 0;
	if (desiredPosition == 90) {
		desiredPosition = 270;
	} else {
		desiredPosition = 90;
	}
  }
}
