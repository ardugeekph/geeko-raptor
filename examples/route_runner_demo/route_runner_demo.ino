/**
 * RouteRunner demo: define a const RouteStep[] plan and call runner.tick(robot)
 * after robot.update() each loop. Motors are stopped while waiting for a trigger.
 * Use makeNoTrigger() + makeNoAction() on the first step to start line-follow immediately.
 *
 * Calibrate sensors for LineMask triggers; use LineLevel or raw lineThreshold.
 *
 * IR remote (IrRemoteKeys.h):
 *   OK -> start / restart route from step 0
 *   *  -> calibrateSensors()
 *   #  -> calibrateStraightDrive() (10s manual push, saves EEPROM trim)
 *   0/1/2 -> resume route at step index (after OK)
 */

#include <GeekoBot.h>
#include <IRremote.h>
#include <IrRemoteKeys.h>
#include <RouteRunner.h>

#define MOTOR_RPM 1500
#define WHEEL_DIAMETER GEEKO_WHEEL_DIAMETER_IN

GeekoBot robot;
RouteRunner runner;
static bool routeStarted = false;

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
		makeLineTrigger(IR_MASK_INNER_LEFT, LineLevel::Mid),
		makeActionForward(120, stopByTime(120)),
		makeSpeedSegment(140, stopByTime(1200), lineFollowPID(0.13f, 0.0f, 0.15f)),
		makeSpeedSegment(95, stopByTime(500), LinePolarity::Light)
	),
	makeStep(
		makeDistanceTrigger(0.f),
		makeActionForward(100, stopByDistance(1.5f)),
		makeSpeedSegment(130, lineFollowPID(0.13f, 0.0f, 0.15f))
	),
	makeStep(makeActionTurnLeft(150, 90.0f)),
	makeStep(makeActionForward(100, stopByDistance(1.0f))),
};

void setup() {
	Serial.begin(115200);
	robot.begin(MOTOR_RPM, WHEEL_DIAMETER);
	robot.motorLeft.encoder.attachEncoderInterrupt(leftEncoderISR);
	robot.motorRight.encoder.attachEncoderInterrupt(rightEncoderISR);
	// robot.motorLeft.reverse();
	// robot.motorRight.reverse();
	IrReceiver.begin(IR_REMOTE_SENSOR);

	runner.begin(PLAN, sizeof(PLAN) / sizeof(PLAN[0]));
	Serial.println(F("Press OK on remote to start route."));
}

void loop() {
	if (IrReceiver.decode()) {
		const uint32_t code = IrReceiver.decodedIRData.decodedRawData;

		if (code == IR_KEY_OK) {
			routeStarted = true;
			runner.reset();
			robot.stop();
			Serial.println(F("IR OK -> route started"));
		} else if (code == IR_KEY_STAR) {
			robot.stop();
			Serial.println(F("IR * -> calibrateSensors()"));
			robot.calibrateSensors();
			Serial.println(F("Sensor calibration done."));
		} else if (code == IR_KEY_HASH) {
			robot.stop();
			Serial.println(F("IR # -> calibrateStraightDrive() (10s, push straight)"));
			robot.calibrateStraightDrive();
			Serial.print(F("Straight cal done. pwmTrim="));
			Serial.println(robot.getStraightPwmTrim());
		} else if (routeStarted) {
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
		}
		IrReceiver.resume();
	}

	robot.update();
	if (routeStarted) {
		runner.tick(robot);
	} else {
		robot.stop();
	}

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
