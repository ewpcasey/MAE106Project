const int mosfet = 4;
const int piston = 9;
const int motDir = 8;
const int motSpd = 10;

void setup()
{
  pinMode(piston, OUTPUT);     
  pinMode(mosfet,OUTPUT);
  pinMode(motDir, OUTPUT);
  digitalWrite(mosfet,HIGH);
  pinMode(7,OUTPUT);
  digitalWrite(7,LOW);
}

void loop()
{
  digitalWrite(piston, HIGH);
  digitalWrite(motDir, HIGH);
  analogWrite(motSpd, 100);
  delay(1000);
  digitalWrite(motDir, LOW);
  delay(1000);
  digitalWrite(piston, LOW);
  digitalWrite(motDir, HIGH);
  analogWrite(motSpd, 200);
  delay(1000);
  digitalWrite(motDir, LOW);
  delay(1000);
}

