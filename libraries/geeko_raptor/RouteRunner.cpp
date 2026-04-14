#include "RouteRunner.h"

void RouteRunner::begin(const RouteStep* plan, uint16_t count) {
	plan_ = plan;
	count_ = count;
	index_ = 0;
	state_ = (plan && count > 0) ? RouteRunnerState::WaitingTrigger : RouteRunnerState::Finished;
	triggerDistBaseline_ = 0.f;
	segmentStartMs_ = 0;
	segmentDistBaseline_ = 0.f;
	rearmTriggerBaseline_ = true;
	lineFollowIntegral_ = 0.0f;
	lineFollowLastError_ = 0.0f;
	lineFollowLastUpdateMs_ = 0;
}

void RouteRunner::reset() {
	begin(plan_, count_);
}

void RouteRunner::setActionForwardControl(float kp, int16_t maxCorrection) {
	actionForwardKp_ = kp;
	maxActionForwardCorrection_ = maxCorrection;
}

void RouteRunner::setLineFollowTunings(float kp, float ki, float kd) {
	lineFollowKp_ = kp;
	lineFollowKi_ = ki;
	lineFollowKd_ = kd;
}

void RouteRunner::setLineFollowCorrectionLimit(int16_t maxCorrection) {
	maxLineFollowCorrection_ = maxCorrection;
}

float RouteRunner::maxWheelDistance_(GeekoBot& robot) {
	float dl = robot.motorLeft.encoder.getDistance();
	float dr = robot.motorRight.encoder.getDistance();
	return (dl > dr) ? dl : dr;
}

bool RouteRunner::lineTriggerFired_(const RouteTrigger& tr, int irVals[9]) {
	if (tr.kind != TriggerKind::LineMask || tr.sensorMask == 0) {
		return false;
	}
	for (int i = 0; i < 9; i++) {
		if (tr.sensorMask & (1u << i)) {
			if ((int)irVals[i] < (int)tr.lineThreshold) {
				return false;
			}
		}
	}
	return true;
}

bool RouteRunner::distanceTriggerFired_(const RouteTrigger& tr, float baseline, GeekoBot& robot) {
	if (tr.kind != TriggerKind::DistanceTravelled) {
		return false;
	}
	return (maxWheelDistance_(robot) - baseline) >= tr.travelInches;
}

bool RouteRunner::stopIsNoOp_(const StopCondition& stop) {
	if (stop.kind == StopKind::ByTime) {
		return stop.timeMs == 0;
	}
	return stop.distanceInches <= 0.f;
}

bool RouteRunner::stopSatisfied_(const StopCondition& stop, unsigned long startMs, float startDist, GeekoBot& robot) {
	if (stop.kind == StopKind::ByTime) {
		return (millis() - startMs) >= stop.timeMs;
	}
	return (maxWheelDistance_(robot) - startDist) >= stop.distanceInches;
}

void RouteRunner::enterWaitingTrigger_(GeekoBot& robot) {
	state_ = RouteRunnerState::WaitingTrigger;
	rearmTriggerBaseline_ = true;
	robot.stop();
	lineFollowIntegral_ = 0.0f;
	lineFollowLastError_ = 0.0f;
	lineFollowLastUpdateMs_ = millis();
}

void RouteRunner::startSegment_(GeekoBot& robot) {
	segmentStartMs_ = millis();
	segmentDistBaseline_ = maxWheelDistance_(robot);
}

void RouteRunner::applyActionForwardControlTick_(GeekoBot& robot, int16_t baseSpeed) {
	const float currentL = robot.motorLeft.encoder.getDistance();
	const float currentR = robot.motorRight.encoder.getDistance();
	const float deltaL = currentL - actionDistBaselineL_;
	const float deltaR = currentR - actionDistBaselineR_;
	const float error = deltaL - deltaR;

	int correction = (int)(error * actionForwardKp_);
	if (correction > maxActionForwardCorrection_) correction = maxActionForwardCorrection_;
	if (correction < -maxActionForwardCorrection_) correction = -maxActionForwardCorrection_;

	int left = (int)baseSpeed - correction;
	int right = (int)baseSpeed + correction;
	if (left > 255) left = 255;
	if (left < -255) left = -255;
	if (right > 255) right = 255;
	if (right < -255) right = -255;

	robot.motorLeft.setSpeed(left);
	robot.motorRight.setSpeed(right);
}

void RouteRunner::applyLineFollowTick_(GeekoBot& robot, int16_t baseSpeed) {
	unsigned long now = millis();
	float dt = (lineFollowLastUpdateMs_ == 0) ? 0.02f : ((now - lineFollowLastUpdateMs_) / 1000.0f);
	if (dt <= 0.0f) dt = 0.02f;
	lineFollowLastUpdateMs_ = now;

	float error = (float)robot.sensor.getPos();
	lineFollowIntegral_ += error * dt;
	const float derivative = (error - lineFollowLastError_) / dt;
	lineFollowLastError_ = error;

	float output = (lineFollowKp_ * error) + (lineFollowKi_ * lineFollowIntegral_) + (lineFollowKd_ * derivative);
	int correction = (int)output;
	if (correction > maxLineFollowCorrection_) correction = maxLineFollowCorrection_;
	if (correction < -maxLineFollowCorrection_) correction = -maxLineFollowCorrection_;

	int left = (int)baseSpeed + correction;
	int right = (int)baseSpeed - correction;
	if (left > 255) left = 255;
	if (left < -255) left = -255;
	if (right > 255) right = 255;
	if (right < -255) right = -255;

	robot.motorLeft.setSpeed(left);
	robot.motorRight.setSpeed(right);
}

void RouteRunner::applyActionATick_(GeekoBot& robot, const RouteActionA& action) {
	const int speedMag = abs((int)action.speed);

	switch (action.kind) {
		case RouteActionKind::TurnLeft:
			robot.motorLeft.setSpeed(-speedMag);
			robot.motorRight.setSpeed(speedMag);
			break;
		case RouteActionKind::TurnRight:
			robot.motorLeft.setSpeed(speedMag);
			robot.motorRight.setSpeed(-speedMag);
			break;
		case RouteActionKind::Backward:
			robot.motorLeft.setSpeed(-speedMag);
			robot.motorRight.setSpeed(-speedMag);
			break;
		case RouteActionKind::Forward:
		default:
			applyActionForwardControlTick_(robot, speedMag);
			break;
	}
}

void RouteRunner::tick(GeekoBot& robot) {
	if (!plan_ || count_ == 0 || state_ == RouteRunnerState::Finished) {
		if (state_ == RouteRunnerState::Finished) {
			robot.stop();
		}
		return;
	}

	const RouteStep& step = plan_[index_];

	switch (state_) {
		case RouteRunnerState::WaitingTrigger: {
			if (rearmTriggerBaseline_) {
				triggerDistBaseline_ = maxWheelDistance_(robot);
				rearmTriggerBaseline_ = false;
			}

			robot.stop();

			bool needLine = (step.trigger.kind == TriggerKind::LineMask);
			if (needLine) {
				robot.sensor.readIrCalibrated(irVals_);
			}

			bool fired = false;
			if (step.trigger.kind == TriggerKind::LineMask) {
				fired = lineTriggerFired_(step.trigger, irVals_);
			} else if (step.trigger.kind == TriggerKind::DistanceTravelled) {
				fired = distanceTriggerFired_(step.trigger, triggerDistBaseline_, robot);
			}

			if (fired) {
				actionDistBaselineL_ = robot.motorLeft.encoder.getDistance();
				actionDistBaselineR_ = robot.motorRight.encoder.getDistance();
				startSegment_(robot);
				state_ = RouteRunnerState::RunningAction;
			}
			break;
		}

		case RouteRunnerState::RunningAction: {
			applyActionATick_(robot, step.action);
			if (stopIsNoOp_(step.action.stop) ||
				stopSatisfied_(step.action.stop, segmentStartMs_, segmentDistBaseline_, robot)) {
				if (stopIsNoOp_(step.speedA.stop)) {
					if (stopIsNoOp_(step.speedB.stop)) {
						index_++;
						if (index_ >= count_) {
							state_ = RouteRunnerState::Finished;
							robot.stop();
						} else {
							enterWaitingTrigger_(robot);
						}
					} else {
						startSegment_(robot);
						state_ = RouteRunnerState::RunningSpeedB;
					}
				} else {
					startSegment_(robot);
					state_ = RouteRunnerState::RunningSpeedA;
				}
			}
			break;
		}

		case RouteRunnerState::RunningSpeedA: {
			applyLineFollowTick_(robot, step.speedA.speed);

			if (stopIsNoOp_(step.speedA.stop) ||
				stopSatisfied_(step.speedA.stop, segmentStartMs_, segmentDistBaseline_, robot)) {
				if (stopIsNoOp_(step.speedB.stop)) {
					index_++;
					if (index_ >= count_) {
						state_ = RouteRunnerState::Finished;
						robot.stop();
					} else {
						enterWaitingTrigger_(robot);
					}
				} else {
					startSegment_(robot);
					state_ = RouteRunnerState::RunningSpeedB;
				}
			}
			break;
		}

		case RouteRunnerState::RunningSpeedB: {
			applyLineFollowTick_(robot, step.speedB.speed);

			if (stopIsNoOp_(step.speedB.stop) ||
				stopSatisfied_(step.speedB.stop, segmentStartMs_, segmentDistBaseline_, robot)) {
				index_++;
				if (index_ >= count_) {
					state_ = RouteRunnerState::Finished;
					robot.stop();
				} else {
					enterWaitingTrigger_(robot);
				}
			}
			break;
		}

		case RouteRunnerState::Finished:
		default:
			robot.stop();
			break;
	}
}
