void setup() {
  Serial.begin(115200);
  pinMode(33, OUTPUT); // Set the pin as output
}

// Remember that the pin work with inverted logic
// LOW to Turn on and HIGH to turn off
void loop() {
  Serial.println("Down!");
  digitalWrite(33, LOW); //Turn on
  delay (1000); //Wait 1 sec
  Serial.println("UP!");
  digitalWrite(33, HIGH); //Turn off
  delay (1000); //Wait 1 sec
}