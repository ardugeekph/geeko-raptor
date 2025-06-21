
#include <Arduino.h>
#include <EEPROM.h>
#include "GeekoBot.h"
#include "PIDController.h"


// Initialization 
void GeekoBot::begin(int motorRpm, float wheelDiameter) {
	// Initialize MUX A-C
	DDRC |= (1 << PC3) | (1 << PC4) | (1 << PC5);

	// Initialize MUX OUTPUT
	pinMode(IR_MUX_OUTPUT, INPUT);

	// Initialize motors
	motorLeft.begin(L_MOTOR_1, L_MOTOR_2, L_MOTOR_PWM, L_MOTOR_C1, L_MOTOR_C2, motorRpm, wheelDiameter);
	motorRight.begin(R_MOTOR_2, R_MOTOR_1, R_MOTOR_PWM, R_MOTOR_C1, R_MOTOR_C2, motorRpm, wheelDiameter);

	// Initialize sensors
	sensor.begin();

	// Controllers
	moveStraightPIDController.setConstants(0.5, 0.0, 0.02);
}


void GeekoBot::calibrateSensors() {
	sensor.calibrate(motorLeft, motorRight);
}


void GeekoBot::moveStraight(int rpm) {
	update();

	// Right motor to adjust speed based on left motor RPM
	float rpmL = motorLeft.encoder.getRpm();
	float rpmR = motorRight.encoder.getRpm();

	// Compute error between distances
	float error = rpmL - rpmR;
	int adjust = moveStraightPIDController.output(error);


	Serial.println("Target RPM: " + String(rpm) + 
		" \t " + String(rpmL) + 
		" \t " + String(rpmR) + 
		" \t " + String(adjust));

	// // Apply correction to left motor speed
	motorLeft.setRpmSpeed(constrain(rpm-adjust, 0, motorLeft.getMotorRPM()));

	// // Apply correction to right motor speed
	motorRight.setRpmSpeed(constrain(rpm+adjust, 0, motorRight.getMotorRPM()));
}

void GeekoBot::stop() {
	motorLeft.stop();
	motorRight.stop();
}

void GeekoBot::update() {
	motorLeft.encoder.update();
	motorRight.encoder.update();
}














