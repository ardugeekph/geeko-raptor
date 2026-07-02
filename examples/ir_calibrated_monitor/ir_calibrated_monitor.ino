/**
 * IR calibrated monitor — print readIrCalibrated() values for tuning LineRules triggers.
 *
 * Workflow:
 * 1. Upload and open Serial Monitor at 115200.
 * 2. Run robot.calibrateSensors() once (uncomment below) or rely on EEPROM from a prior run.
 * 3. Place the robot at the pose where a route trigger should fire.
 * 4. Note each channel value from Serial (F0–F7 front left→right, BACK = rear).
 * 5. Build rules with irRule(ch, min, max) using observed values ± ~50–100 margin.
 * 6. Use makeLineRulesTrigger(rules, TriggerLogic::All) or TriggerLogic::Any in your route plan.
 *
 * LineRules compare these calibrated values directly (no LinePolarity).
 */

#include <GeekoBot.h>

#define MOTOR_RPM 1500
#define WHEEL_DIAMETER 1.1

GeekoBot robot;

void setup() {
	Serial.begin(115200);
	robot.begin(MOTOR_RPM, WHEEL_DIAMETER);

	// Uncomment once per surface to store calibration in EEPROM:
	// robot.calibrateSensors();
}

void loop() {
	robot.update();

	static unsigned long lastPrint;
	if (millis() - lastPrint >= 200) {
		lastPrint = millis();
		robot.sensor.printIrCalibrated(Serial);
	}
}
