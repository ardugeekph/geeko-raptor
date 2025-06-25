#include "PIDController.h"



void PIDController::setConstants(float p, float i, float d) {
	kP_ = p;
	kI_ = i;
	kD_ = d;
}


float PIDController::output(float error) {
	unsigned long now = millis();
    unsigned long deltaTime = now - lastUpdate;
	

    if (deltaTime >= 20) {
		// Calculate P, I, D
		pOut = error * kP_;
		iOut += kI_ * error;
		dOut = (error - lastError) * kD_;
		lastError = error;

		// Serial.print("\t");
		// Serial.print(error);
		// Serial.print("\t");
		// Serial.print(kP_);
		// Serial.print("\t");
		// Serial.print(kI_);
		// Serial.print("\t");
		// Serial.print(kD_);
		// Serial.print("\t");
		// Serial.print(pOut);
		// Serial.print("\t");
		// Serial.print(iOut);
		// Serial.print("\t");
		// Serial.print(dOut);
		// Serial.print("\t");

		// Get summation
		pid_output = pOut + iOut + dOut;
		// Serial.println(pid_output);
		
		// Timers
        lastUpdate = now;
    }

	return pid_output;
}

void PIDController::reset() {
	lastError = 0;
	pOut = 0;
	iOut = 0;
	dOut = 0;
	pid_output = 0;
}