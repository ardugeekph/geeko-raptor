#include <IRremote.h>


void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Start serial for debugging
  Serial.begin(9600);

  // Start the IR receiver
  IrReceiver.begin(IR_REMOTE_SENSOR); // Start the receiver
}

void loop() {
  // Check if IR signal is received
  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
      
    // Beep when IR signal is detected
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);

    IrReceiver.resume(); // Enable receiving of the next value
  }
}