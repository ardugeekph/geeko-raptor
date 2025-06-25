
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
	moveStraightPIDController.setConstants(3.00, 0.05, 0.50);
}


void GeekoBot::calibrateSensors() {
	sensor.calibrate(motorLeft, motorRight);
}


void GeekoBot::moveStraight(int rpm, bool (*stopCallback)()) {

	while (true) {
		if (stopCallback()) {
			stop();
			return;
		}
		
		// Update encoders and sensors
		update();
		
		// Get distance traveled by each wheel
		float distanceL = motorLeft.encoder.getDistance();  // in inches
		float distanceR = motorRight.encoder.getDistance(); // in inches

		// Compute error in distance
		float error = distanceL - distanceR;

		// PID output based on distance error
		int adjust = moveStraightPIDController.output(error);

		// // Debug output
		// Serial.println("Target RPM: " + String(rpm) + 
		// 	" \t Dist L: " + String(distanceL) + 
		// 	" \t Dist R: " + String(distanceR) + 
		// 	" \t Error: " + String(error) + 
		// 	" \t Adjust: " + String(adjust));

		// Apply correction: slow down the wheel that went farther
		int rpmL = rpm - adjust;
		int rpmR = rpm + adjust;

		motorLeft.setRpmSpeed(rpmL, 1, rpmL < 0);
		motorRight.setRpmSpeed(rpmR, 1, rpmL < 0);
	}
}

void GeekoBot::stop() {
	motorLeft.stop();
	motorRight.stop();
}

void GeekoBot::update() {
	motorLeft.encoder.update();
	motorRight.encoder.update();
}














