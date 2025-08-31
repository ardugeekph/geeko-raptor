void setup() {
  // Set LED pins as OUTPUT
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
}

void loop() {
  // Turn the LEDs ON and OFF with an interval of 250ms
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, LOW);
  delay(250);

  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_RIGHT, HIGH);
  delay(250);
}