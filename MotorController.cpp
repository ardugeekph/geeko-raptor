#include <Arduino.h>
#include "MotorController.h"
#include "PIDController.h"


void MotorController::begin(int motorA, int motorB, int motorPwm, int motorC1, int motorC2, int motorRPM_, float wheelDiameter_) {
		motorApin = motorA;
		motorBpin = motorB;
		motorPwmPin = motorPwm;

		pinMode(motorApin, OUTPUT);
		pinMode(motorBpin, OUTPUT);
		pinMode(motorPwmPin, OUTPUT);
		setMotorRPM(motorRPM_);

		encoder.begin(motorC1, motorC2);
		encoder.setWheelDiameter(wheelDiameter_);
		pidController.setConstants(0.2, 0.00, 0.02);
}


int MotorController::getMotorRPM() {
    return motorRPM;
}


void MotorController::setMotorRPM(int rpm) {
    motorRPM = rpm;
}


void MotorController::setSpeed(int pwm){
	if(pwm > 255) pwm = 255;
  	if(pwm < -255) pwm = -255;

	if(pwm < 0) {
		digitalWrite(motorApin, true);
		digitalWrite(motorBpin, false);
  	} else {
		digitalWrite(motorApin, false);
		digitalWrite(motorBpin, true);
  	}
	analogWrite(motorPwmPin, abs(pwm));
}


void MotorController::setRpmSpeed(float targetRPM, float accel) {
	unsigned long now = millis();
	if (now - lastUpdate < MOTOR_CONTROL_UPDATE_INTERVAL) return;
	lastUpdate = now;

	// Convert acceleration m/s^2 → RPM per loop interval
	float wheelDiameterMeters = encoder.getWheelDiameter() * 0.0254;
	float accelRpmPerSec = (accel / (PI * wheelDiameterMeters)) * 60.0;
	float accelRpmPerUpdate = accelRpmPerSec * (MOTOR_CONTROL_UPDATE_INTERVAL / 1000.0);

	// Ramp up/down current setpoint toward target RPM
	if (currentSetpointRPM < targetRPM) {
		currentSetpointRPM += accelRpmPerUpdate;
		if (currentSetpointRPM > targetRPM) currentSetpointRPM = targetRPM;
	} else if (currentSetpointRPM > targetRPM) {
		currentSetpointRPM -= accelRpmPerUpdate;
		if (currentSetpointRPM < targetRPM) currentSetpointRPM = targetRPM;
	}

	// Serial.print(wheelDiameterMeters);
	// Serial.print("\t");
	// Serial.print(accelRpmPerSec);
	// Serial.print("\t");
	// Serial.print(accelRpmPerUpdate);
	// Serial.print("\t");
	// Serial.print(currentSetpointRPM);


	// Serial.print("RPM: \t");
	// Serial.print(rpm);
	// Serial.print("\t");
	// Serial.print(encoder.getRpm());

	float currentRPM = encoder.getRpm();
	float rpmCorrection = pidController.output(currentSetpointRPM - currentRPM);
	float finalRPM = currentSetpointRPM + rpmCorrection;
	float pwm = (255/float(motorRPM))*finalRPM;

	// Serial.print("\t");
	// Serial.print(targetRPM);
	// Serial.print("\t");
	// Serial.println(currentRPM);

	setSpeed(pwm);
}


void MotorController::stop(){
	digitalWrite(motorApin, HIGH);
  	digitalWrite(motorBpin, HIGH);
  	analogWrite(motorPwmPin, 255);
}
