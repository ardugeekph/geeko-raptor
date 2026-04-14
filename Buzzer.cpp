#include "Buzzer.h"


void Buzzer::begin() {
	pinMode(BUZZER_PIN, OUTPUT);
}

void Buzzer::beep(bool on) {
	digitalWrite(BUZZER_PIN, on);
}
