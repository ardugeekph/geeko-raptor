#ifndef RouteRunner_h
#define RouteRunner_h

#include <Arduino.h>
#include "GeekoBot.h"
#include "PIDController.h"

enum class RouteActionKind : uint8_t { None, Forward, Backward, TurnLeft, TurnRight };
enum class StopKind : uint8_t { ByTime, ByDistance, UntilNextTrigger };
enum class TriggerKind : uint8_t { None, LineMask, LineRules, DistanceTravelled };
enum class LinePolarity : uint8_t { Dark, Light };

#include "LineTriggers.h"

struct StopCondition {
	StopKind kind;
	uint32_t timeMs;
	float distanceInches;
};

struct RouteTrigger {
	TriggerKind kind;
	float travelInches;
	uint16_t sensorMask;
	uint16_t lineThreshold;
	LinePolarity linePolarity;
	const IrChannelRule* rules;
	uint8_t ruleCount;
	TriggerLogic logic;
};

struct RouteActionA {
	RouteActionKind kind;
	int16_t speed;
	StopCondition stop;
	float turnDegrees;
};

struct LineFollowPID {
	float kp;
	float ki;
	float kd;
	bool custom;
};

struct RouteSpeedSegment {
	int16_t speed;
	StopCondition stop;
	LineFollowPID lineFollowPid;
	LinePolarity linePolarity;
};

struct RouteStep {
	RouteTrigger trigger;
	RouteActionA action;
	RouteSpeedSegment speedA;
	RouteSpeedSegment speedB;
};

inline StopCondition stopByTime(uint32_t ms) {
	StopCondition stop = {StopKind::ByTime, ms, 0.f};
	return stop;
}

inline StopCondition stopByDistance(float inches) {
	StopCondition stop = {StopKind::ByDistance, 0, inches};
	return stop;
}

inline StopCondition stopUntilNextTrigger() {
	StopCondition stop = {StopKind::UntilNextTrigger, 0, 0.f};
	return stop;
}

inline RouteTrigger makeNoTrigger() {
	RouteTrigger t = {
		TriggerKind::None, 0.f, 0, 0, LinePolarity::Dark, nullptr, 0, TriggerLogic::All
	};
	return t;
}

inline RouteTrigger makeLineTrigger(
	uint16_t sensorMask,
	uint16_t lineThreshold,
	LinePolarity linePolarity = LinePolarity::Dark
) {
	RouteTrigger t = {
		TriggerKind::LineMask,
		0.f,
		sensorMask,
		lineThreshold,
		linePolarity,
		nullptr,
		0,
		TriggerLogic::All
	};
	return t;
}

inline RouteTrigger makeLineTrigger(
	uint16_t sensorMask,
	LineLevel level,
	LinePolarity linePolarity = LinePolarity::Dark
) {
	return makeLineTrigger(sensorMask, static_cast<uint16_t>(level), linePolarity);
}

template<uint8_t N>
inline RouteTrigger makeLineRulesTrigger(
	const IrChannelRule (&rules)[N],
	TriggerLogic logic = TriggerLogic::All
) {
	RouteTrigger t = {
		TriggerKind::LineRules,
		0.f,
		0,
		0,
		LinePolarity::Dark,
		rules,
		N,
		logic
	};
	return t;
}

inline RouteTrigger makeDistanceTrigger(float travelInches) {
	RouteTrigger t = {
		TriggerKind::DistanceTravelled,
		travelInches,
		0,
		0,
		LinePolarity::Dark,
		nullptr,
		0,
		TriggerLogic::All
	};
	return t;
}

inline RouteActionA makeNoAction() {
	RouteActionA a = {RouteActionKind::None, 0, stopByTime(0), 0.f};
	return a;
}

inline RouteActionA makeActionForward(int16_t speed, const StopCondition& stop) {
	RouteActionA a = {RouteActionKind::Forward, speed, stop, 0.f};
	return a;
}

inline RouteActionA makeActionBackward(int16_t speed, const StopCondition& stop) {
	RouteActionA a = {RouteActionKind::Backward, speed, stop, 0.f};
	return a;
}

inline RouteActionA makeActionTurnLeft(int16_t speed, float degrees) {
	RouteActionA a = {RouteActionKind::TurnLeft, speed, stopByTime(0), degrees};
	return a;
}

inline RouteActionA makeActionTurnRight(int16_t speed, float degrees) {
	RouteActionA a = {RouteActionKind::TurnRight, speed, stopByTime(0), degrees};
	return a;
}

inline LineFollowPID defaultLineFollowPID() {
	LineFollowPID pid = {0.f, 0.f, 0.f, false};
	return pid;
}

inline LineFollowPID lineFollowPID(float kp, float ki, float kd) {
	LineFollowPID pid = {kp, ki, kd, true};
	return pid;
}

inline RouteSpeedSegment makeSpeedSegment(
	int16_t speed,
	LinePolarity linePolarity = LinePolarity::Dark
) {
	RouteSpeedSegment segment = {speed, stopUntilNextTrigger(), defaultLineFollowPID(), linePolarity};
	return segment;
}

inline RouteSpeedSegment makeSpeedSegment(
	int16_t speed,
	const StopCondition& stop,
	LinePolarity linePolarity = LinePolarity::Dark
) {
	RouteSpeedSegment segment = {speed, stop, defaultLineFollowPID(), linePolarity};
	return segment;
}

inline RouteSpeedSegment makeSpeedSegment(
	int16_t speed,
	const StopCondition& stop,
	const LineFollowPID& lineFollowPid,
	LinePolarity linePolarity = LinePolarity::Dark
) {
	RouteSpeedSegment segment = {speed, stop, lineFollowPid, linePolarity};
	return segment;
}

inline RouteSpeedSegment makeSpeedSegment(
	int16_t speed,
	const LineFollowPID& lineFollowPid,
	LinePolarity linePolarity = LinePolarity::Dark
) {
	RouteSpeedSegment segment = {speed, stopUntilNextTrigger(), lineFollowPid, linePolarity};
	return segment;
}

inline RouteSpeedSegment makeNoSpeedSegment() {
	return makeSpeedSegment(0, stopByTime(0));
}

inline RouteStep makeStep(
	const RouteTrigger& trigger,
	const RouteActionA& action,
	const RouteSpeedSegment& speedA
) {
	RouteStep step = {trigger, action, speedA, makeNoSpeedSegment()};
	return step;
}

inline RouteStep makeStep(
	const RouteTrigger& trigger,
	const RouteActionA& action,
	const RouteSpeedSegment& speedA,
	const RouteSpeedSegment& speedB
) {
	RouteStep step = {trigger, action, speedA, speedB};
	return step;
}

inline RouteStep makeStep(
	const RouteTrigger& trigger,
	const RouteActionA& action
) {
	return makeStep(trigger, action, makeNoSpeedSegment());
}

inline RouteStep makeStep(const RouteActionA& action) {
	return makeStep(makeNoTrigger(), action);
}

enum class RouteRunnerState : uint8_t {
	WaitingTrigger,
	RunningAction,
	RunningSpeedA,
	RunningSpeedB,
	Finished
};

class RouteRunner {
public:
	void begin(const RouteStep* plan, uint16_t count);
	void tick(GeekoBot& robot);
	void reset();
	bool setIndex(uint16_t index, GeekoBot& robot);
	void restartFromIndex(uint16_t index, GeekoBot& robot);
	uint16_t stepCount() const { return count_; }
	void setActionForwardControl(float kp, int16_t maxCorrection);
	void setActionForwardControl(float kp, float ki, int16_t maxCorrection);
	void setTurnScale(float scale);
	void setLineFollowTunings(float kp, float ki, float kd);

	bool finished() const { return state_ == RouteRunnerState::Finished; }
	uint16_t currentIndex() const { return index_; }
	RouteRunnerState state() const { return state_; }

private:
	static float maxWheelDistance_(GeekoBot& robot);
	static bool lineTriggerFired_(const RouteTrigger& tr, int irVals[9]);
	static bool lineRulesTriggerFired_(const RouteTrigger& tr, int irVals[9]);
	static bool distanceTriggerFired_(const RouteTrigger& tr, float baseline, GeekoBot& robot);
	static bool triggerFired_(const RouteTrigger& tr, int irVals[9], float distBaseline, GeekoBot& robot);
	static bool triggerNeedsIr_(TriggerKind kind);
	static bool stopIsNoOp_(const StopCondition& stop);
	static bool stopIsUntilNextTrigger_(const StopCondition& stop);
	static bool stopSatisfied_(const StopCondition& stop, unsigned long startMs, float startDist, GeekoBot& robot);
	static bool turnSatisfied_(const RouteActionA& action, float dirBaselineL, float dirBaselineR, GeekoBot& robot, float turnScale);

	bool nextStepTriggerFired_(GeekoBot& robot);
	bool actionDone_(const RouteStep& step, GeekoBot& robot);
	bool speedSegmentDone_(const StopCondition& stop, unsigned long startMs, float startDist, GeekoBot& robot);
	void advanceToNextStepAction_(GeekoBot& robot);
	void advanceToNextStep_(GeekoBot& robot);

	void enterWaitingTrigger_(GeekoBot& robot);
	void skipActionAndBeginSpeedSegments_(GeekoBot& robot, const RouteStep& step);
	void startSegment_(GeekoBot& robot);
	void startAction_(GeekoBot& robot);
	void applySpeedSegmentTunings_(const RouteSpeedSegment& segment);
	void applyActionATick_(GeekoBot& robot, const RouteActionA& action);
	void applyDifferentialDriveTick_(GeekoBot& robot, int16_t signedSpeed);
	void applyLineFollowTick_(GeekoBot& robot, int16_t baseSpeed);

	const RouteStep* plan_ = nullptr;
	uint16_t count_ = 0;
	uint16_t index_ = 0;
	RouteRunnerState state_ = RouteRunnerState::Finished;

	float triggerDistBaseline_ = 0.f;
	unsigned long segmentStartMs_ = 0;
	float segmentDistBaseline_ = 0.f;
	float actionDistBaselineL_ = 0.f;
	float actionDistBaselineR_ = 0.f;
	float actionDirBaselineL_ = 0.f;
	float actionDirBaselineR_ = 0.f;
	bool rearmTriggerBaseline_ = true;

	float actionForwardKp_ = 8.0f;
	float actionForwardKi_ = 0.0f;
	int16_t maxActionForwardCorrection_ = 40;
	float turnScale_ = 1.0f;

	float lineFollowKp_ = 0.13f;
	float lineFollowKi_ = 0.0f;
	float lineFollowKd_ = 0.15f;

	LinePolarity linePolarity_ = LinePolarity::Dark;

	PIDController lineFollowPid_;
	PIDController actionForwardPid_;

	int irVals_[9];
};

#endif
