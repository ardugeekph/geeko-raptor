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
		motorRPM = motorRPM_;

		encoder.begin(motorC1, motorC2);
		encoder.setWheelDiameter(wheelDiameter_);
		pidController.setConstants(10000, 2000, 4000);
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


void MotorController::setRpmSpeed(float targetRPM, float accel, bool reverse) {
	encoder.update();  // Ensure encoder is updated before calculations
	targetRPM = abs(targetRPM);

    unsigned long now = millis();
    float deltaT = (now - lastUpdate) / 1000.0;
    if (deltaT < 0.1) return;  // prevent over-updating
    lastUpdate = now;

    // Convert acceleration m/s^2 → RPM per loop interval
    float wheelDiameterMeters = encoder.getWheelDiameter() * 0.0254;
    float accelRpmPerSec = (accel / (PI * wheelDiameterMeters)) * 60.0;
	float accelRpmPerUpdate = accelRpmPerSec * deltaT;

    float currentRPM = abs(encoder.getRpm());

    // Smoothly ramp current setpoint RPM toward the target
    if (abs(currentSetpointRPM - targetRPM) > accelRpmPerUpdate) {
        if (currentSetpointRPM < targetRPM) {
            currentSetpointRPM += accelRpmPerUpdate;
        } else {
            currentSetpointRPM -= accelRpmPerUpdate;
        }
    } else {
        currentSetpointRPM = targetRPM;
    }

    // PID control based on the current RPM error
    float rpmCorrection = pidController.output(currentSetpointRPM - currentRPM);
    float finalRPM = currentSetpointRPM + rpmCorrection;

    float pwm = (255.0 / float(motorRPM)) * finalRPM;
	pwm = 0.00007 * finalRPM * finalRPM + 0.08 * finalRPM + 40;
    pwm = constrain(pwm, 0, 255);

	if (reverse) {
		pwm = -pwm;  // Reverse direction if needed
	}

    // // Debugging output
    // Serial.print("T: ");
    // Serial.print(now);
    // Serial.print(" | Dia: ");
    // Serial.print(wheelDiameterMeters);
    // Serial.print(" | AccelRPM/s: ");
    // Serial.print(accelRpmPerSec);
    // Serial.print(" | RPM/update: ");
    // Serial.print(accelRpmPerUpdate);
    // Serial.print(" | Setpoint: ");
    // Serial.print(currentSetpointRPM);
    // Serial.print(" | RPM: ");
    // Serial.print(currentRPM);
    // Serial.print(" | Correc: ");
    // Serial.print(rpmCorrection);
    // Serial.print(" | PWM: ");
    // Serial.println(pwm);

    setSpeed(pwm);
}


void MotorController::stop(){
	digitalWrite(motorApin, HIGH);
  	digitalWrite(motorBpin, HIGH);
  	analogWrite(motorPwmPin, 255);

	currentSetpointRPM = 0;
	pidController.reset();
	lastUpdate = millis();
}
