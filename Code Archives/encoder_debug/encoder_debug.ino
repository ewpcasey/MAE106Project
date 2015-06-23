boolean A_SIG = HIGH, B_SIG = HIGH;

void setup(){
  Serial.begin(115200);
  attachInterrupt(0,A_CHANGE,A_SIG);
  attachInterrupt(1,B_CHANGE,B_SIG);
}

void loop(){
}

void A_CHANGE(){
  A_SIG=!A_SIG;
  Serial.print("A: ");
  Serial.println(A_SIG);
  attachInterrupt(0,A_CHANGE,A_SIG);
}

void B_CHANGE(){
  B_SIG=!B_SIG;
  Serial.print("B: ");
  Serial.println(B_SIG);
  attachInterrupt(1,B_CHANGE,B_SIG);
}


