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
	linePolarity_ = LinePolarity::Dark;
	lineFollowPid_.setConstants(lineFollowKp_, lineFollowKi_, lineFollowKd_);
	actionForwardPid_.setConstants(actionForwardKp_, actionForwardKi_, 0.0f);
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
	linePolarity_ = LinePolarity::Dark;
	actionDistBaselineL_ = robot.motorLeft.encoder.getDistance();
	actionDistBaselineR_ = robot.motorRight.encoder.getDistance();
	actionDirBaselineL_ = robot.motorLeft.encoder.getDirectionalDistance();
	actionDirBaselineR_ = robot.motorRight.encoder.getDirectionalDistance();
	enterWaitingTrigger_(robot);
	return true;
}

void RouteRunner::restartFromIndex(uint16_t index, GeekoBot& robot) {
	(void)setIndex(index, robot);
}

void RouteRunner::setActionForwardControl(float kp, int16_t maxCorrection) {
	setActionForwardControl(kp, 0.0f, maxCorrection);
}

void RouteRunner::setActionForwardControl(float kp, float ki, int16_t maxCorrection) {
	actionForwardKp_ = kp;
	actionForwardKi_ = ki;
	maxActionForwardCorrection_ = maxCorrection;
	actionForwardPid_.setConstants(kp, ki, 0.0f);
}

void RouteRunner::setTurnScale(float scale) {
	turnScale_ = scale;
}

void RouteRunner::setLineFollowPID(float kp, float ki, float kd) {
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
	const bool inverse = (tr.linePolarity == LinePolarity::Light);
	for (int i = 0; i < 9; i++) {
		if (tr.sensorMask & (1u << i)) {
			const int v = inverse ? (1023 - irVals[i]) : irVals[i];
			if (v < (int)tr.lineThreshold) {
				return false;
			}
		}
	}
	return true;
}

static bool ruleMatches_(const IrChannelRule& r, int irVals[9]) {
	if (r.channel > 8) {
		return false;
	}
	const int v = irVals[r.channel];
	return v >= (int)r.min && v <= (int)r.max;
}

bool RouteRunner::lineRulesTriggerFired_(const RouteTrigger& tr, int irVals[9]) {
	if (tr.kind != TriggerKind::LineRules || tr.rules == nullptr || tr.ruleCount == 0) {
		return false;
	}

	if (tr.logic == TriggerLogic::All) {
		for (uint8_t i = 0; i < tr.ruleCount; i++) {
			if (!ruleMatches_(tr.rules[i], irVals)) {
				return false;
			}
		}
		return true;
	}

	for (uint8_t i = 0; i < tr.ruleCount; i++) {
		if (ruleMatches_(tr.rules[i], irVals)) {
			return true;
		}
	}
	return false;
}

bool RouteRunner::triggerNeedsIr_(TriggerKind kind) {
	return kind == TriggerKind::LineMask || kind == TriggerKind::LineRules;
}

bool RouteRunner::triggerFired_(const RouteTrigger& tr, int irVals[9], float distBaseline, GeekoBot& robot) {
	switch (tr.kind) {
		case TriggerKind::None:
			return true;
		case TriggerKind::LineMask:
			return lineTriggerFired_(tr, irVals);
		case TriggerKind::LineRules:
			return lineRulesTriggerFired_(tr, irVals);
		case TriggerKind::DistanceTravelled:
			return distanceTriggerFired_(tr, distBaseline, robot);
		default:
			return false;
	}
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
	if (triggerNeedsIr_(tr.kind)) {
		robot.sensor.readIrCalibrated(irVals_);
	}
	return triggerFired_(tr, irVals_, triggerDistBaseline_, robot);
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

	startAction_(robot);
	state_ = RouteRunnerState::RunningAction;
}

bool RouteRunner::stopSatisfied_(const StopCondition& stop, unsigned long startMs, float startDist, GeekoBot& robot) {
	if (stop.kind == StopKind::ByTime) {
		return (millis() - startMs) >= stop.timeMs;
	}
	return (maxWheelDistance_(robot) - startDist) >= stop.distanceInches;
}

bool RouteRunner::turnSatisfied_(
	const RouteActionA& action,
	float dirBaselineL,
	float dirBaselineR,
	GeekoBot& robot,
	float turnScale
) {
	if (action.turnDegrees <= 0.f) {
		return true;
	}

	const float deltaL = robot.motorLeft.encoder.getDirectionalDistance() - dirBaselineL;
	const float deltaR = robot.motorRight.encoder.getDirectionalDistance() - dirBaselineR;
	const float diff = deltaR - deltaL;
	const float targetDiff = robot.turnTargetWheelDiffInches(action.turnDegrees) * turnScale;

	if (action.kind == RouteActionKind::TurnLeft) {
		return diff >= targetDiff;
	}
	if (action.kind == RouteActionKind::TurnRight) {
		return diff <= -targetDiff;
	}
	return false;
}

void RouteRunner::enterWaitingTrigger_(GeekoBot& robot) {
	state_ = RouteRunnerState::WaitingTrigger;
	rearmTriggerBaseline_ = true;
	robot.stop();
	lineFollowPid_.reset();
	actionForwardPid_.reset();
}

void RouteRunner::advanceToNextStep_(GeekoBot& robot) {
	index_++;
	if (index_ >= count_) {
		state_ = RouteRunnerState::Finished;
		robot.stop();
		return;
	}

	triggerDistBaseline_ = maxWheelDistance_(robot);
	const RouteStep& step = plan_[index_];

	if (step.trigger.kind != TriggerKind::None) {
		enterWaitingTrigger_(robot);
		return;
	}

	if (step.action.kind != RouteActionKind::None) {
		startAction_(robot);
		state_ = RouteRunnerState::RunningAction;
	} else {
		skipActionAndBeginSpeedSegments_(robot, step);
	}
}

void RouteRunner::skipActionAndBeginSpeedSegments_(GeekoBot& robot, const RouteStep& step) {
	if (stopIsNoOp_(step.speedA.stop)) {
		if (stopIsNoOp_(step.speedB.stop)) {
			advanceToNextStep_(robot);
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

void RouteRunner::startAction_(GeekoBot& robot) {
	actionDistBaselineL_ = robot.motorLeft.encoder.getDistance();
	actionDistBaselineR_ = robot.motorRight.encoder.getDistance();
	actionDirBaselineL_ = robot.motorLeft.encoder.getDirectionalDistance();
	actionDirBaselineR_ = robot.motorRight.encoder.getDirectionalDistance();
	startSegment_(robot);
	lineFollowPid_.reset();
	actionForwardPid_.reset();
}

void RouteRunner::applySpeedSegmentTunings_(const RouteSpeedSegment& segment) {
	if (segment.lineFollowPid.custom) {
		lineFollowPid_.setConstants(segment.lineFollowPid.kp, segment.lineFollowPid.ki, segment.lineFollowPid.kd);
	} else {
		lineFollowPid_.setConstants(lineFollowKp_, lineFollowKi_, lineFollowKd_);
	}
	linePolarity_ = segment.linePolarity;
	lineFollowPid_.reset();
}

void RouteRunner::applyDifferentialDriveTick_(GeekoBot& robot, int16_t signedSpeed) {
	const float currentL = robot.motorLeft.encoder.getDistance();
	const float currentR = robot.motorRight.encoder.getDistance();
	const float deltaL = currentL - actionDistBaselineL_;
	const float deltaR = currentR - actionDistBaselineR_;
	float error = deltaL - deltaR;
	if (signedSpeed < 0) {
		error = -error;
	}

	int correction = (int)actionForwardPid_.output(error);
	if (correction > maxActionForwardCorrection_) correction = maxActionForwardCorrection_;
	if (correction < -maxActionForwardCorrection_) correction = -maxActionForwardCorrection_;

	const int trim = robot.getStraightPwmTrim();
	int left = (int)signedSpeed - correction - trim;
	int right = (int)signedSpeed + correction + trim;
	if (left > 255) left = 255;
	if (left < -255) left = -255;
	if (right > 255) right = 255;
	if (right < -255) right = -255;

	robot.motorLeft.setSpeed(left);
	robot.motorRight.setSpeed(right);
}

void RouteRunner::applyLineFollowTick_(GeekoBot& robot, int16_t baseSpeed) {
	const bool inverse = (linePolarity_ == LinePolarity::Light);
	const float error = (float)robot.sensor.getPos(inverse);
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
			applyDifferentialDriveTick_(robot, (int16_t)-speedMag);
			break;
		case RouteActionKind::Forward:
		default:
			applyDifferentialDriveTick_(robot, (int16_t)speedMag);
			break;
	}
}

static bool isTurnAction_(RouteActionKind kind) {
	return kind == RouteActionKind::TurnLeft || kind == RouteActionKind::TurnRight;
}

bool RouteRunner::actionDone_(const RouteStep& step, GeekoBot& robot) {
	const RouteActionA& action = step.action;
	if (isTurnAction_(action.kind)) {
		return turnSatisfied_(action, actionDirBaselineL_, actionDirBaselineR_, robot, turnScale_);
	}
	return stopIsNoOp_(action.stop) ||
		stopSatisfied_(action.stop, segmentStartMs_, segmentDistBaseline_, robot);
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

			if (triggerNeedsIr_(step.trigger.kind)) {
				robot.sensor.readIrCalibrated(irVals_);
			}

			const bool fired = triggerFired_(step.trigger, irVals_, triggerDistBaseline_, robot);

			if (fired) {
				if (step.action.kind == RouteActionKind::None) {
					skipActionAndBeginSpeedSegments_(robot, step);
				} else {
					startAction_(robot);
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
			if (actionDone_(step, robot)) {
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
					advanceToNextStep_(robot);
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
					advanceToNextStep_(robot);
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
