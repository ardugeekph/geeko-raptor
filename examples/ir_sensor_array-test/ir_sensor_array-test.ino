void setup() {
  Serial.begin(9600);
  // Set the pins connected to the Multiplexer 3-bit signal control as an OUTPUT
  pinMode(IR_MUX_A, OUTPUT);
  pinMode(IR_MUX_B, OUTPUT);
  pinMode(IR_MUX_C, OUTPUT);

  // Set the pin connected to the Multiplexer output pin as an INPUT
  pinMode(IR_MUX_OUTPUT, INPUT);

  // Set the pin connected to the Auxiliary Sensor as an INPUT
  pinMode(IR_BACK_SENSOR, INPUT);
}

void loop() {
  for (int i = 0; i < 8; i++) {
		selectMUXChannel_(i);
    Serial.print("SEN" + String(i) + ": ");
		Serial.print(analogRead(IR_MUX_OUTPUT));
		Serial.print("\t");
	}
	Serial.print("AUX: ");
  Serial.println(analogRead(IR_BACK_SENSOR));
}

void selectMUXChannel_(int channel) {
	// Set Multiplexer 3-bit signal to 
  digitalWrite(IR_MUX_A, (channel & 0b001) ? HIGH : LOW);
  digitalWrite(IR_MUX_B, (channel & 0b010) ? HIGH : LOW);
  digitalWrite(IR_MUX_C, (channel & 0b100) ? HIGH : LOW);
}