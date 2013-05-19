int pulses,high=pulses,low=pulses;
int count,hcount,lcount;
const int countMax=4,diff=1;
boolean A_SIG = HIGH, B_SIG = HIGH;

void setup(){
  Serial.begin(115200);
  attachInterrupt(0,A_CHANGE,A_SIG);
  attachInterrupt(1,B_CHANGE,B_SIG);
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
  attachInterrupt(0,A_CHANGE,A_SIG);
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
      A_SIG = HIGH;
      B_SIG = HIGH;
      hcount=0;
      lcount=0;
    }
  }
}

