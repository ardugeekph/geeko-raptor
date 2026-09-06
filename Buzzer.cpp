#include "Buzzer.h"


void Buzzer::begin() {
	pinMode(BUZZER_PIN, OUTPUT);
}

void Buzzer::beep(bool on) {
	digitalWrite(BUZZER_PIN, on);
}

void Buzzer::pulse(unsigned long durationMs) {
	beep(true);
	pulseEndMs_ = millis() + durationMs;
}

void Buzzer::tick() {
	if (pulseEndMs_ != 0 && millis() >= pulseEndMs_) {
		pulseEndMs_ = 0;
		beep(false);
	}
}
