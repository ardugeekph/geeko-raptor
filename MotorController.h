#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>
#include "MotorEncoder.h"
#include "PIDController.h"


class MotorController {
	public:
		void begin(int motorA, int motorB, int motorPwm, int motorC1, int motorC2, int motorRPM, float wheelDiameter);
		void setSpeed(int pwm);
		void setRpmSpeed(float targetRPM, float accel = 1, bool reverse=false); // accel in m/s^2
		void stop();

		MotorEncoder encoder;

	private:
		PIDController pidController;

		int motorRPM;
		int motorApin;
		int motorBpin;
		int motorPwmPin;

		float currentSetpointRPM = 0;
		unsigned long lastUpdate = 0;
};

#endif

