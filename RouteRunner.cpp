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
	lineFollowPid_.setConstants(lineFollowKp_, lineFollowKi_, lineFollowKd_);
	actionForwardPid_.setConstants(actionForwardKp_, 0.0f, 0.0f);
	lineFollowPid_.reset();
	actionForwardPid_.reset();
}

void RouteRunner::reset() {
	begin(plan_, count_);
}

bool RouteRunner::setIndex(uint16_t index, GeekoBot& robot) {
	if (index >= count_ || plan_ == nullptr) {
		return false;
	}

	index_ = index;
	segmentStartMs_ = 0;
	segmentDistBaseline_ = 0.f;
	actionDistBaselineL_ = robot.motorLeft.encoder.getDistance();
	actionDistBaselineR_ = robot.motorRight.encoder.getDistance();
	enterWaitingTrigger_(robot);
	return true;
}

void RouteRunner::restartFromIndex(uint16_t index, GeekoBot& robot) {
	(void)setIndex(index, robot);
}

void RouteRunner::setActionForwardControl(float kp, int16_t maxCorrection) {
	actionForwardKp_ = kp;
	maxActionForwardCorrection_ = maxCorrection;
	actionForwardPid_.setConstants(kp, 0.0f, 0.0f);
}

void RouteRunner::setLineFollowTunings(float kp, float ki, float kd) {
	lineFollowKp_ = kp;
	lineFollowKi_ = ki;
	lineFollowKd_ = kd;
	lineFollowPid_.setConstants(kp, ki, kd);
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
	if (stop.kind == StopKind::UntilNextTrigger) {
		return false;
	}
	if (stop.kind == StopKind::ByTime) {
		return stop.timeMs == 0;
	}
	return stop.distanceInches <= 0.f;
}

bool RouteRunner::stopIsUntilNextTrigger_(const StopCondition& stop) {
	return stop.kind == StopKind::UntilNextTrigger;
}

bool RouteRunner::nextStepTriggerFired_(GeekoBot& robot) {
	if (index_ + 1 >= count_ || plan_ == nullptr) {
		return false;
	}

	const RouteTrigger& tr = plan_[index_ + 1].trigger;
	if (tr.kind == TriggerKind::None) {
		return true;
	}
	if (tr.kind == TriggerKind::LineMask) {
		robot.sensor.readIrCalibrated(irVals_);
		return lineTriggerFired_(tr, irVals_);
	}
	if (tr.kind == TriggerKind::DistanceTravelled) {
		return distanceTriggerFired_(tr, triggerDistBaseline_, robot);
	}
	return false;
}

bool RouteRunner::speedSegmentDone_(
	const StopCondition& stop,
	unsigned long startMs,
	float startDist,
	GeekoBot& robot
) {
	if (stopIsUntilNextTrigger_(stop)) {
		return nextStepTriggerFired_(robot);
	}
	return stopIsNoOp_(stop) || stopSatisfied_(stop, startMs, startDist, robot);
}

void RouteRunner::advanceToNextStepAction_(GeekoBot& robot) {
	index_++;
	if (index_ >= count_) {
		state_ = RouteRunnerState::Finished;
		robot.stop();
		return;
	}

	triggerDistBaseline_ = maxWheelDistance_(robot);
	const RouteStep& step = plan_[index_];
	if (step.action.kind == RouteActionKind::None) {
		skipActionAndBeginSpeedSegments_(robot, step);
		return;
	}

	actionDistBaselineL_ = robot.motorLeft.encoder.getDistance();
	actionDistBaselineR_ = robot.motorRight.encoder.getDistance();
	startSegment_(robot);
	lineFollowPid_.reset();
	state_ = RouteRunnerState::RunningAction;
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
	lineFollowPid_.reset();
	actionForwardPid_.reset();
}

void RouteRunner::skipActionAndBeginSpeedSegments_(GeekoBot& robot, const RouteStep& step) {
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
			applySpeedSegmentTunings_(step.speedB);
			state_ = RouteRunnerState::RunningSpeedB;
		}
	} else {
		startSegment_(robot);
		applySpeedSegmentTunings_(step.speedA);
		state_ = RouteRunnerState::RunningSpeedA;
	}
}

void RouteRunner::startSegment_(GeekoBot& robot) {
	segmentStartMs_ = millis();
	segmentDistBaseline_ = maxWheelDistance_(robot);
}

void RouteRunner::applySpeedSegmentTunings_(const RouteSpeedSegment& segment) {
	if (segment.lineFollowPid.custom) {
		lineFollowPid_.setConstants(segment.lineFollowPid.kp, segment.lineFollowPid.ki, segment.lineFollowPid.kd);
	} else {
		lineFollowPid_.setConstants(lineFollowKp_, lineFollowKi_, lineFollowKd_);
	}
	lineFollowPid_.reset();
}

void RouteRunner::applyActionForwardControlTick_(GeekoBot& robot, int16_t baseSpeed) {
	const float currentL = robot.motorLeft.encoder.getDistance();
	const float currentR = robot.motorRight.encoder.getDistance();
	const float deltaL = currentL - actionDistBaselineL_;
	const float deltaR = currentR - actionDistBaselineR_;
	const float error = deltaL - deltaR;

	int correction = (int)actionForwardPid_.output(error);
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
	const float error = (float)robot.sensor.getPos();
	int correction = (int)lineFollowPid_.output(error);

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
		case RouteActionKind::None:
			robot.stop();
			break;
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
			if (step.trigger.kind == TriggerKind::None) {
				fired = true;
			} else if (step.trigger.kind == TriggerKind::LineMask) {
				fired = lineTriggerFired_(step.trigger, irVals_);
			} else if (step.trigger.kind == TriggerKind::DistanceTravelled) {
				fired = distanceTriggerFired_(step.trigger, triggerDistBaseline_, robot);
			}

			if (fired) {
				if (step.action.kind == RouteActionKind::None) {
					skipActionAndBeginSpeedSegments_(robot, step);
				} else {
					actionDistBaselineL_ = robot.motorLeft.encoder.getDistance();
					actionDistBaselineR_ = robot.motorRight.encoder.getDistance();
					startSegment_(robot);
					state_ = RouteRunnerState::RunningAction;
				}
			}
			break;
		}

		case RouteRunnerState::RunningAction: {
			if (step.action.kind == RouteActionKind::None) {
				skipActionAndBeginSpeedSegments_(robot, step);
				break;
			}

			applyActionATick_(robot, step.action);
			if (stopIsNoOp_(step.action.stop) ||
				stopSatisfied_(step.action.stop, segmentStartMs_, segmentDistBaseline_, robot)) {
				skipActionAndBeginSpeedSegments_(robot, step);
			}
			break;
		}

		case RouteRunnerState::RunningSpeedA: {
			applyLineFollowTick_(robot, step.speedA.speed);

			if (speedSegmentDone_(step.speedA.stop, segmentStartMs_, segmentDistBaseline_, robot)) {
				if (stopIsUntilNextTrigger_(step.speedA.stop)) {
					advanceToNextStepAction_(robot);
				} else if (stopIsNoOp_(step.speedB.stop)) {
					index_++;
					if (index_ >= count_) {
						state_ = RouteRunnerState::Finished;
						robot.stop();
					} else {
						enterWaitingTrigger_(robot);
					}
				} else {
					startSegment_(robot);
					applySpeedSegmentTunings_(step.speedB);
					state_ = RouteRunnerState::RunningSpeedB;
				}
			}
			break;
		}

		case RouteRunnerState::RunningSpeedB: {
			applyLineFollowTick_(robot, step.speedB.speed);

			if (speedSegmentDone_(step.speedB.stop, segmentStartMs_, segmentDistBaseline_, robot)) {
				if (stopIsUntilNextTrigger_(step.speedB.stop)) {
					advanceToNextStepAction_(robot);
				} else {
					index_++;
					if (index_ >= count_) {
						state_ = RouteRunnerState::Finished;
						robot.stop();
					} else {
						enterWaitingTrigger_(robot);
					}
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
