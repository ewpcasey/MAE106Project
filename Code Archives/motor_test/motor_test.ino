//const int analogInPin = A4;
const int analogOutPin = 6;
const int digitalOutPin = 7;


void setup()
{
  Serial.begin(115200);
  pinMode(digitalOutPin, OUTPUT);
}

void loop()
{
  digitalWrite(digitalOutPin, HIGH);
  analogWrite(analogOutPin, 255);
  delay(2000);
  digitalWrite(digitalOutPin, LOW);
  analogWrite(analogOutPin, 255);
  delay(2000);
  digitalWrite(digitalOutPin, HIGH);
  analogWrite(analogOutPin, 30);
  delay(2000);
  digitalWrite(digitalOutPin, LOW);
  analogWrite(analogOutPin, 30);
  delay(2000);
}

