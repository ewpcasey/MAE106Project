const int mosfet = 4;
const int motDir = 8;
const int motSpd = 10;
const int rotA = 2;
const int rotB = 3;
const int CPR = 48;

boolean A_SIG = HIGH, B_SIG = HIGH;

int counter = 0;
int dt = 1;

double pastAngle = 0;
double currentAngle = 0;
double desiredAngle;

double error = 0;
double motorVelocity = 0;
double errorIntegral = 0;
double controlSignal = 0;

int PGain = 35;
int DGain = 0;
double IGain = 0.025;

void setup()
{
  pinMode(mosfet, OUTPUT);
  pinMode(motDir, INPUT); 
  pinMode(motSpd, OUTPUT);
  digitalWrite(mosfet, HIGH);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);

  attachInterrupt(rotA, A_CHANGE, A_SIG);
  attachInterrupt(rotB, B_CHANGE, B_SIG);
}

void loop()
{
  // You can map out the timing scheme you want here:
  if(counter = 4000){
    counter = 0;  
  }
  if(counter = 0){
    desiredAngle =  60;
  }
  if(counter = 2000){
    desiredAngle = -60; 
  }




  error = currentAngle - desiredAngle;
  motorVelocity = (currentAngle - desiredAngle)/dt;
  errorIntegral = errorIntegral + error*(dt/1000);

  controlSignal = -1*PGain*error - DGain*motorVelocity - IGain*errorIntegral;

  if(controlSignal <=0){
    digitalWrite(motDir, LOW);
  }
  else{
    digitalWrite(motDir, HIGH); 
  }

  analogWrite(motSpd, constrain(abs(controlSignal), 0, 255));
  delay(dt);

  counter = counter + dt;
}

boolean XOR(boolean A, boolean B){
  return((A&&(!B))||(B&(!A))); 
}

void A_CHANGE()
{
  detachInterrupt(rotA);
  if (!XOR(A_SIG,B_SIG)){
    currentAngle = currentAngle + (360 / 48);
  } 
  else{
    currentAngle = currentAngle - (360 / 48);
  }
  A_SIG = !A_SIG;
  attachInterrupt(rotA, A_CHANGE, A_SIG);
}

void B_CHANGE()
{
  detachInterrupt(rotB);
  if(XOR(A_SIG,B_SIG)){
    currentAngle = currentAngle - (360 / 48);
  }
  else{
    currentAngle = currentAngle + (360 / 48);
  }
  B_SIG = !B_SIG;
  attachInterrupt(rotB, B_CHANGE, B_SIG);
}


