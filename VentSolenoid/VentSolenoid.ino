int tPin = 7;

void setup() {
  // put your setup code here, to run once:
  pinMode(tPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(tPin, HIGH);
  delay(2000);
  digitalWrite(tPin, LOW);
  delay(2000);

}
