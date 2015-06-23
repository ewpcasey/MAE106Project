int valve = 12;

void setup(){
  pinMode(valve, OUTPUT);
//  digitalWrite(valve, LOW);
}


void loop(){
  digitalWrite(valve, HIGH);
  delay(500);
  digitalWrite(valve, LOW);
  delay(1500);
}
