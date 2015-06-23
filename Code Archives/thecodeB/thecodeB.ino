#include <avr/interrupt.h>
#include <avr/io.h>


int pulses,high=pulses,low=pulses;
int count,hcount,lcount;
const int countMax=4,diff=1;
const int waittime = 500;
boolean A_SIG = HIGH, B_SIG = HIGH;
boolean finished = false;

int valve = 12;

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A4;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 6; // Analog output pin that the "P" channel is attached to (PWM Signal)
const int digitalOutPin = 7; // Digital output pin that the "D" channel is attached to (controls motor Direction)

// PID Gains
int PGain = 35;
int DGain = 0;
int IGain = 0.025;

// Input and Output Variables
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

// Variables needed for Control Law
double potAngle;			// value in degrees
double prevPotAngle = 0;		// previous value read from pot
double desiredPosition = 146
;		// desired position (in degrees) we want the motor to go to
double error = 0;				// current error
double motorVelocity = 0;		// first derivative of motorPosition
double errorIntegral = 0;		// used in PID control law
double controlSignal = 0;		// control signal sent to the motor

// Time keeping variables
//int pulsesr = pulses;
int minutecount = 0;                            //one minute limit
int counter = 0;				// keeps track of time (for switching desired position)
int dt = 4;					// time it takes to execute one loop (in ms)

void setup(){
  Serial.begin(115200);
  //  Serial.println("booted");
  pinMode(digitalOutPin, OUTPUT);	// sets the "D" channel pin to an output
  pinMode(valve, OUTPUT);
  attachInterrupt(0,A_CHANGE,A_SIG);
  attachInterrupt(1,B_CHANGE,B_SIG);
  //  InitializeIO();
  //  InitializeInterrupt();
  potAngle = map(sensorValue, 0, 1023, 0, 300); 
}

void loop(){
  // read the analog in value:
  if(minutecount < waittime)
  {
    desiredPosition = 0;
    PGain = 250;
    DGain = 0;
    IGain = .35;
  }
  else if (!finished)
  {
    desiredPosition = 152;
    PGain = 50;
    DGain = 0;
    IGain = .15; 
  }

  sensorValue = analogRead(analogInPin);  // Read as an integer b/w 0 and 1023 (10-bit resolution)

  // First save the previous value
  prevPotAngle = potAngle;
  // then save the new value, mapping it to a value in degrees:
  potAngle = map(sensorValue, 0, 1023, 0, 300); 

  // use it to compute error and motor speed
  error = potAngle - desiredPosition;
  motorVelocity = (potAngle - prevPotAngle)/(dt); // (in deg/ms)
  errorIntegral = errorIntegral + error*(dt/1000); 

  // Calculate the motor control signal using a PID control law
  controlSignal = -1*PGain*error - DGain*motorVelocity - IGain*errorIntegral;	

  // tell the motor which way to spin
  if (controlSignal <= 0) {
    digitalWrite(digitalOutPin, LOW);
  } 
  else {
    digitalWrite(digitalOutPin, HIGH);
  }

  // map it to the range of the analog out:
  controlSignal = abs(controlSignal);  // can only ouptut positive values
  outputValue = constrain(controlSignal, 0, 255);  // 8-bit resolution

  // change the analog out value:
  analogWrite(analogOutPin, outputValue);         



  //print the results to the serial monitor: (UNCOMMENT TO DISPLAY)
  //Serial.print("Desired Position = " );                       
//  Serial.print(desiredPosition);      
//  //Serial.print("\t Actual Position = ");     
//  Serial.print("\t       "); 
//  Serial.println(potAngle);   

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(dt);
  counter = counter + dt;		// increment the counter

  if (pulses >=750)
  {
    counter = 201;
    desiredPosition = 300;
    PGain = 250;
    DGain = 0;
    IGain = .35;
    finished = true;
  }

  minutecount = minutecount + 1;

  if (minutecount >= 60000){
    counter = 201;
    desiredPosition = 300;
    PGain = 250;
    DGain = 0;
    IGain = .35;
    finished = true;
  }

  if (counter >= 200){
    digitalWrite(valve, LOW); 
  }

  if ((minutecount >= waittime) && (counter >= 400)) {
    Serial.println("hello");		// every 500 ms, change the desired position of the motor (simulates a square wave input)
    counter = 0;
    digitalWrite(valve, HIGH);
    //       delay(1500);
  }

  //  delay(5000);

  //  if (pulses >= 120 && pulses <=200){
  //	desiredPosition = 173;  
  //  }
  //  else {
  //          desiredPosition = 133;
  //        }

  Serial.print(pulses);
  Serial.print(",  ");
  Serial.print(desiredPosition);      
  Serial.print(",  ");
  Serial.println(potAngle);  
}


void A_CHANGE(){
  //  Serial.print(A_SIG);
  //  Serial.print(B_SIG);
  //  Serial.print(" ");

  if(XOR(A_SIG,B_SIG)){
    pulses--;
    reMinMax(&high,&max(high,pulses),&hcount);
  }
  else{
    pulses++;
    reMinMax(&low,&max(low,pulses),&lcount);
  }
  checkCount();

  A_SIG=!A_SIG;
  //  Serial.print("\t       ");
  //  Serial.println(pulses);
  attachInterrupt(0,A_CHANGE,A_SIG);
}

void B_CHANGE(){
  //  Serial.print(A_SIG);
  //  Serial.print(B_SIG);
  //  Serial.print(" ");

  if(!XOR(A_SIG,B_SIG)){
    pulses--;
    reMinMax(&high,&max(high,pulses),&hcount); 
  }
  else{
    pulses++; 
    reMinMax(&low,&min(low,pulses),&lcount);
  }
  checkCount();

  B_SIG=!B_SIG;
  //  Serial.println(pulses);
  attachInterrupt(1,B_CHANGE,B_SIG);
}

boolean XOR(boolean A, boolean B){
  return((A&&(!B))||(B&&(!A))); 
}

void reMinMax(int*hl,int*minmax,int*mcount){
  if(hl!=minmax){
    count = 0;
    mcount = 0;
    hl = minmax;
  }
  if(mcount>=&countMax){
    hl = minmax; 
  }
}

void checkCount(){
  count++;
  hcount++;
  lcount++;
  if(count>=countMax){
    count = 0;
    if((high-low)<diff){
      A_SIG = 0;
      B_SIG = 0;
      hcount=0;
      lcount=0;
    }
  }
}

//void InitializeIO(){
//  pinMode(A0,INPUT);
//  pinMode(A1,INPUT); 
////  Serial.println("initialized inputs");
//}
//
//void InitializeInterrupt(){
//  cli();
//  PCICR =0x02;
////  Serial.print("PCICR:");
////  Serial.println(PCICR);
//  PCMSK1 = 0b00000001;
////  Serial.print("PCMSK:");
////  Serial.println(PCMSK1);
//  MCUCR = (1<<ISC10)|(1<<ISC11);
////  Serial.print("MCUCR:");
////  Serial.println(MCUCR);
//  sei();
////  Serial.println("arduino initialized");
//}
//
//ISR(PCINT1_vect){
////  Serial.print("sig_");
//  if(digitalRead(A0)!=A_STATE){
////    Serial.println("A");
////    Serial.print("A_SIG: ");
////    Serial.println(A_SIG);
//    A_STATE=digitalRead(A0);
////    Serial.print("A_STATE: ");
////    Serial.println(A_STATE);
//    if(A_STATE==A_SIG){
//      A_CHANGE();
//    }
//  }
//  if(digitalRead(A1)!=B_STATE){
////    Serial.println("B");
////    Serial.print("B_SIG: ");
////    Serial.println(B_SIG);
//    B_STATE=digitalRead(A1);
////    Serial.print("B_STATE: ");
////    Serial.println(B_STATE);
//    if(B_STATE==B_SIG){
//      B_CHANGE();
//    }
//  }
////  Serial.print(" ");
////  Serial.print(A_SIG);
////  Serial.println(B_SIG);
//}
//
//
//


