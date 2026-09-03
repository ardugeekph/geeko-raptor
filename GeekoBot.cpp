
#include <Arduino.h>
#include <EEPROM.h>
#include "GeekoBot.h"
#include "PIDController.h"

static constexpr int MOTOR_ENCODER_TICKS_PER_REV = 135;
static constexpr uint8_t STRAIGHT_CAL_EEPROM_MAGIC = 0x53;
static constexpr int STRAIGHT_CAL_EEPROM_MAGIC_ADDR = 39;
static constexpr int STRAIGHT_CAL_EEPROM_TRIM_ADDR = 40;
static constexpr unsigned long STRAIGHT_CAL_DURATION_MS = 10000;
static constexpr unsigned long STRAIGHT_CAL_BLINK_MS = 250;


// Initialization 
void GeekoBot::begin(int motorRpm, float wheelDiameter, float trackWidth) {
	trackWidth_ = trackWidth;

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

	// Aux
	buzzer.begin();

	restoreStraightCalibration_();
}


void GeekoBot::saveStraightCalibration_(int16_t pwmTrim) {
	EEPROM.write(STRAIGHT_CAL_EEPROM_MAGIC_ADDR, STRAIGHT_CAL_EEPROM_MAGIC);
	EEPROM.write(STRAIGHT_CAL_EEPROM_TRIM_ADDR, (uint8_t)(pwmTrim & 0xFF));
	EEPROM.write(STRAIGHT_CAL_EEPROM_TRIM_ADDR + 1, (uint8_t)((pwmTrim >> 8) & 0xFF));
}


void GeekoBot::restoreStraightCalibration_() {
	if (EEPROM.read(STRAIGHT_CAL_EEPROM_MAGIC_ADDR) != STRAIGHT_CAL_EEPROM_MAGIC) {
		straightPwmTrim_ = 0;
		straightCalValid_ = false;
		return;
	}

	const int16_t trim = (int16_t)(
		(uint16_t)EEPROM.read(STRAIGHT_CAL_EEPROM_TRIM_ADDR) |
		((uint16_t)EEPROM.read(STRAIGHT_CAL_EEPROM_TRIM_ADDR + 1) << 8)
	);
	straightPwmTrim_ = trim;
	straightCalValid_ = true;
}


void GeekoBot::calibrateSensors() {
	sensor.calibrate(motorLeft, motorRight);
}


float GeekoBot::turnTargetWheelDiffInches(float degrees) const {
	return (degrees * PI / 180.0f) * trackWidth_;
}


long GeekoBot::turnTargetTickDiff(float degrees) const {
	const float diffInches = turnTargetWheelDiffInches(degrees);
	const float wheelDiameter = motorLeft.encoder.getWheelDiameter();
	const float circumference = PI * wheelDiameter;
	const float ticksPerInch = MOTOR_ENCODER_TICKS_PER_REV / circumference;
	return (long)(diffInches * ticksPerInch);
}


void GeekoBot::calibrateStraightDrive() {
	const float initL = motorLeft.encoder.getDistance();
	const float initR = motorRight.encoder.getDistance();

	pinMode(LED_LEFT, OUTPUT);
	pinMode(LED_RIGHT, OUTPUT);

	const unsigned long startMs = millis();
	bool leftOn = true;
	unsigned long lastToggle = startMs;

	digitalWrite(LED_LEFT, HIGH);
	digitalWrite(LED_RIGHT, LOW);
	buzzer.beep(true);

	while (millis() - startMs < STRAIGHT_CAL_DURATION_MS) {
		update();

		if (millis() - lastToggle >= STRAIGHT_CAL_BLINK_MS) {
			lastToggle = millis();
			leftOn = !leftOn;
			digitalWrite(LED_LEFT, leftOn ? HIGH : LOW);
			digitalWrite(LED_RIGHT, leftOn ? LOW : HIGH);
			buzzer.beep(leftOn);
		}
	}

	digitalWrite(LED_LEFT, LOW);
	digitalWrite(LED_RIGHT, LOW);
	buzzer.beep(false);

	const float deltaL = motorLeft.encoder.getDistance() - initL;
	const float deltaR = motorRight.encoder.getDistance() - initR;
	const float error = deltaL - deltaR;

	const float trimKp = 2.0f;
	int trim = (int)(trimKp * error);
	if (trim > 15) trim = 15;
	if (trim < -15) trim = -15;

	straightPwmTrim_ = (int16_t)trim;
	straightCalValid_ = true;
	saveStraightCalibration_(straightPwmTrim_);
}


void GeekoBot::moveStraight(int rpm, bool (*stopCallback)()) {
	float initDistanceL = motorLeft.encoder.getDistance();  // in inches
	float initDistanceR = motorRight.encoder.getDistance(); // in inches

	while (true) {
		if (stopCallback()) {
			stop();
			return;
		}
		
		// Update encoders and sensors
		update();
		
		// Get distance traveled by each wheel
		float distanceL = motorLeft.encoder.getDistance() - initDistanceL;  // in inches
		float distanceR = motorRight.encoder.getDistance() - initDistanceR; // in inches

		// Compute error in distance
		float error = distanceL - distanceR;

		// PID output based on distance error
		int adjust = moveStraightPIDController.output(error);

		// Apply correction: slow down the wheel that went farther
		int rpmL = rpm - adjust;
		int rpmR = rpm + adjust;

		motorLeft.setRpmSpeed(rpmL, 1, rpmL < 0);
		motorRight.setRpmSpeed(rpmR, 1, rpmR < 0);
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
