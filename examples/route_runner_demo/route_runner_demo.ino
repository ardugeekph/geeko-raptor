/**
 * RouteRunner demo: define a const RouteStep[] plan and call runner.tick(robot)
 * after robot.update() each loop. Motors are stopped while waiting for a trigger.
 * Use makeNoTrigger() + makeNoAction() on the first step to start line-follow immediately.
 *
 * Calibrate sensors for LineMask triggers; tune lineThreshold for your surface.
 */

#include <GeekoBot.h>
#include <IRremote.h>
#include <RouteRunner.h>

#define MOTOR_RPM 1500
#define WHEEL_DIAMETER 1.1

GeekoBot robot;
RouteRunner runner;

// Bit i = front sensor i (0 = leftmost, 7 = rightmost on 8 MUX); bit 8 = back.
// Example: both outer front sensors over line (high calibrated reading).
static const uint16_t kOuterLineMask = (1u << 0) | (1u << 7);
// Front sensors 1, 2, 3 all on black line (calibrated reading >= lineThreshold).
static const uint16_t kLineMask123 = (1u << 1) | (1u << 2) | (1u << 3);
static const uint32_t IR_KEY_0 = 0xFF6897;
static const uint32_t IR_KEY_1 = 0xFF30CF;
static const uint32_t IR_KEY_2 = 0xFF18E7;

// RouteStep: Trigger, Action, SpeedA, and optional SpeedB (3-arg makeStep omits SpeedB).
// SpeedA/SpeedB are line-follow segments after Action.
const RouteStep PLAN[] = {
	makeStep(
		makeNoTrigger(),
		makeNoAction(),
		makeSpeedSegment(130, stopByTime(900)),
		makeSpeedSegment(90, stopByDistance(2.0f))
	),
	makeStep(
		makeLineTrigger(kLineMask123, 450),
		makeActionForward(120, stopByTime(120)),
		makeSpeedSegment(140, stopByTime(1200), lineFollowPID(0.13f, 0.0f, 0.15f)),
		makeSpeedSegment(95, stopByTime(500), LineFollowMode::WhiteOnBlack)
	),
	makeStep(
		makeDistanceTrigger(0.f),
		makeActionForward(100, stopByDistance(1.5f)),
		makeSpeedSegment(130, lineFollowPID(0.13f, 0.0f, 0.15f))
	),
};

void setup() {
	Serial.begin(115200);
	robot.begin(MOTOR_RPM, WHEEL_DIAMETER);
	robot.motorLeft.encoder.attachEncoderInterrupt(leftEncoderISR);
	robot.motorRight.encoder.attachEncoderInterrupt(rightEncoderISR);
	IrReceiver.begin(IR_REMOTE_SENSOR);

	// robot.calibrateSensors();

	runner.begin(PLAN, sizeof(PLAN) / sizeof(PLAN[0]));
}

void loop() {
	if (IrReceiver.decode()) {
		const uint32_t code = IrReceiver.decodedIRData.decodedRawData;
		int targetIndex = -1;
		if (code == IR_KEY_0) targetIndex = 0;
		else if (code == IR_KEY_1) targetIndex = 1;
		else if (code == IR_KEY_2) targetIndex = 2;

		if (targetIndex >= 0) {
			const bool ok = runner.setIndex((uint16_t)targetIndex, robot);
			Serial.print(F("IR resume to idx "));
			Serial.print(targetIndex);
			Serial.print(F(" -> "));
			Serial.println(ok ? F("OK") : F("INVALID"));
		}
		IrReceiver.resume();
	}

	robot.update();
	runner.tick(robot);

#ifdef ROUTE_RUNNER_DEMO_SERIAL
	static unsigned long lastPrint;
	if (millis() - lastPrint > 200) {
		lastPrint = millis();
		Serial.print(F("idx="));
		Serial.print(runner.currentIndex());
		Serial.print(F(" state="));
		Serial.print((int)runner.state());
		Serial.print(F(" fin="));
		Serial.println(runner.finished());
	}
#endif
}

void leftEncoderISR() {
	bool a = digitalRead(L_MOTOR_C1);
	bool b = digitalRead(L_MOTOR_C2);
	if (a == b) {
		robot.motorLeft.encoder.decrement();
	} else {
		robot.motorLeft.encoder.increment();
	}
}

void rightEncoderISR() {
	bool a = digitalRead(R_MOTOR_C1);
	bool b = digitalRead(R_MOTOR_C2);
	if (a == b) {
		robot.motorRight.encoder.increment();
	} else {
		robot.motorRight.encoder.decrement();
	}
}
