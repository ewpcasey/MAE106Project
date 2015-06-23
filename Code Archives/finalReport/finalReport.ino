#include <avr/interrupt.h>
#include <avr/io.h>


//Preface:
//For our control, we modified the desired posisition of the motor
//and the final desired pulse count to control our robot's
//destination. We found the ideal values we used through testing
//and a feed-forward model.



//Variables and constants, by row:
//  Encoder pulse tracking
//  Encoder failsafe trigger counters
//  Encoder failsafe trigger boundaries
//  Delay for robot to start firing
//  Encoder signals
//  Robot finished moving
//  Valve status for debug messages

int pulses,high=pulses,low=pulses;
int count,hcount,lcount;
const int countMax=4,diff=1;
const int waittime = 500;
boolean A_SIG = HIGH, B_SIG = HIGH;
boolean finished = false;
boolean state = LOW;

//Vale pin
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
double desiredPosition = 146;		// desired position (in degrees) we want the motor to go to
double error = 0;				// current error
double motorVelocity = 0;		// first derivative of motorPosition
double errorIntegral = 0;		// used in PID control law
double controlSignal = 0;		// control signal sent to the motor

// Time keeping variables
//int pulsesr = pulses;
int minutecount = 0;                            //one minute limit
int counter = 0;				// keeps track of time (for switching desired position)
int dt = 1;					// time it takes to execute one loop (in ms)

void setup(){
  Serial.begin(115200);
  pinMode(digitalOutPin, OUTPUT);	// sets the "D" channel pin to an output
  pinMode(valve, OUTPUT);
  attachInterrupt(0,A_CHANGE,A_SIG);
  attachInterrupt(1,B_CHANGE,B_SIG);
  //Enable alternate interrupt pins for second encoder.
  //  InitializeIO();
  //  InitializeInterrupt();
  potAngle = map(sensorValue, 0, 1023, 0, 300); 
}

void loop(){
  
  //Correct wheel trajectory during the wait time that takes
  //place before the robot runs.
  if(minutecount < waittime)
  {
    desiredPosition = 0;
    PGain = 250;
    DGain = 0;
    IGain = .35;
  }
  
  //If the robot hasn't reached its final position, these
  //are the angles and gains the robot uses.
  else if (!finished)
  {
    desiredPosition = 145;
    PGain = 50;
    DGain = 0;
    IGain = .15; 
  }

  // read the analog in value:
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


  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(dt);
  counter = counter + dt;		// increment the counter


  //Robot has reached the desired position and should stop firing
  //the piston.  Wheels should turn sharply and lock right to
  //brake
  if (pulses >=850)
  {
    counter = 201;
    desiredPosition = 300;
    PGain = 250;
    DGain = 0;
    IGain = .35;
    finished = true;
  }


  minutecount = minutecount + dt;


  //Robot has run for 1 minute and should stop firing the piston.
  if (minutecount >= 60000){
    counter = 201;
    desiredPosition = 300;
    PGain = 250;
    DGain = 0;
    IGain = .35;
    finished = true;
  }


  //Handle timing of the valve. 200ms extension and 50ms
  //retraction are ideal for speed.
  if (counter >= 200){
    digitalWrite(valve, LOW); 
  }

  if ((minutecount >= waittime) && (counter >= 250)) {
    counter = 0;
    digitalWrite(valve, HIGH);
  }
}


//Handles the events for the interrupts the encoders
//send on the  A-channel.
void A_CHANGE(){
  //Remove the interrupt.
  detachInterrupt(0);
  
  //If A and B are the same, the robot is moving in reverse.
  if(XOR(A_SIG,B_SIG)){
    pulses--;
    reMinMax(&high,&max(high,pulses),&hcount);
  }
  
  //Otherwise, the robot is moving forward.
  else{
    pulses++;
    reMinMax(&low,&max(low,pulses),&lcount);
  }
  
  //Check that the robot hasn't been stuck between the same two
  //pulse values. This is our failsafe for correcting erroneous
  //encoder data.
  checkCount();

  //Since A has changed, logged A_SIG is now !A_SIG.
  A_SIG=!A_SIG;
  //Reattach pin interrupt.
  attachInterrupt(0,A_CHANGE,A_SIG);
}


//Handles the events for the interrupts the encoders
//send on the B channel.
void B_CHANGE(){
  detachInterrupt(1);
  //In this case, if A and B are different, the robot is moving in
  //reverse.
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
  attachInterrupt(1,B_CHANGE,B_SIG);
}


//XOR function for handling interrupt and direction logic.
boolean XOR(boolean A, boolean B){
  return((A&&(!B))||(B&&(!A))); 
}


//Resets the counters on the failsafe tracker when the encoder
//starts giving erroneous pulse data.
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


//Checks the failsafe conditions are met. Called before the
//reMinMax function, which "fires" the failsafe.
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



//The follwing code is for enabling alternate interrupt pins on
//the Arduino.  Our initial plan was to use 2 encoders to
//calculate turning, but we ended up using only 1 encoder due to
//failures and bugs in the encoders.

////Sets the first two analog pins as input pins.
//void InitializeIO(){
//  pinMode(A0,INPUT);
//  pinMode(A1,INPUT); 
////  Serial.println("initialized inputs");
//}
//
//
////Disable all interrupt capabilities while interrupt pins are
////being set up (cli()).  Enable interrupts on pin registry 2 
////(analogs).  On registry 2 (PCMSK1) watch for interrupts on 
////the first two pins (00000011). Watch for falling and rising
////edges (MCUCR with ISC 10 and 11. Re-enable interrupts (sei()).
//
//void InitializeInterrupt(){
//  cli();
//  PCICR =0x02;
////  Serial.print("PCICR:");
////  Serial.println(PCICR);
//  PCMSK1 = 0b00000011;
////  Serial.print("PCMSK:");
////  Serial.println(PCMSK1);
//  MCUCR = (1<<ISC10)|(1<<ISC11);
////  Serial.print("MCUCR:");
////  Serial.println(MCUCR);
//  sei();
////  Serial.println("arduino initialized");
//}
//
//
////Interrupt Service Routine (ISR):
////Perform these functions every time an interrupt is detected
////on the pins defined above.
//
//ISR(PCINT1_vect){
////  Serial.print("sig_");
//
//  //Detects if the state of A read from the pin differs from the
//  //logged state of A.  i.e. pin change.
//
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
//
//  //Detects if the state of B read from the pin differs from the
//  //logged state of B.  i.e. pin change.
//
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


