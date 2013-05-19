#include <avr/interrupt.h>
#include <avr/io.h>


int pulses,high=pulses,low=pulses;
int count,hcount,lcount;
const int countMax=4,diff=1;
boolean A_STATE = HIGH, B_STATE = HIGH;
boolean A_SIG = 0, B_SIG = 0;


void setup(){
  Serial.begin(115200);
  Serial.println("booted");
  InitializeIO();
  InitializeInterrupt();
}

void loop(){
}

void A_CHANGE(){
  //  Serial.print(A_SIG);
  //  Serial.print(B_SIG);
  //  Serial.print(" ");

  if(XOR(A_SIG,B_SIG)){
    pulses++;
    reMinMax(&high,&max(high,pulses),&hcount);
  }
  else{
    pulses--;
    reMinMax(&low,&max(low,pulses),&lcount);
  }
  checkCount();

  A_SIG=!A_SIG;
  Serial.println(pulses);
}

void B_CHANGE(){
  //  Serial.print(A_SIG);
  //  Serial.print(B_SIG);
  //  Serial.print(" ");

  if(!XOR(A_SIG,B_SIG)){
    pulses++;
    reMinMax(&high,&max(high,pulses),&hcount); 
  }
  else{
    pulses--; 
    reMinMax(&low,&min(low,pulses),&lcount);
  }
  checkCount();

  B_SIG=!B_SIG;
  Serial.println(pulses);
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

void InitializeIO(){
  pinMode(A0,INPUT);
  pinMode(A1,INPUT); 
//  Serial.println("initialized inputs");
}

void InitializeInterrupt(){
  cli();
  PCICR =0x02;
//  Serial.print("PCICR:");
//  Serial.println(PCICR);
  PCMSK1 = 0b00000001;
//  Serial.print("PCMSK:");
//  Serial.println(PCMSK1);
  MCUCR = (1<<ISC10)|(1<<ISC11);
//  Serial.print("MCUCR:");
//  Serial.println(MCUCR);
  sei();
//  Serial.println("arduino initialized");
}

ISR(PCINT1_vect){
//  Serial.print("sig_");
  if(digitalRead(A0)!=A_STATE){
//    Serial.println("A");
//    Serial.print("A_SIG: ");
//    Serial.println(A_SIG);
    A_STATE=digitalRead(A0);
//    Serial.print("A_STATE: ");
//    Serial.println(A_STATE);
    if(A_STATE==A_SIG){
      A_CHANGE();
    }
  }
  if(digitalRead(A1)!=B_STATE){
//    Serial.println("B");
//    Serial.print("B_SIG: ");
//    Serial.println(B_SIG);
    B_STATE=digitalRead(A1);
//    Serial.print("B_STATE: ");
//    Serial.println(B_STATE);
    if(B_STATE==B_SIG){
      B_CHANGE();
    }
  }
//  Serial.print(" ");
//  Serial.print(A_SIG);
//  Serial.println(B_SIG);
}



