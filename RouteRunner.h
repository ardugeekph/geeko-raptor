#ifndef RouteRunner_h
#define RouteRunner_h

#include <Arduino.h>
#include "GeekoBot.h"
#include "PIDController.h"

enum class RouteActionKind : uint8_t { None, Forward, Backward, TurnLeft, TurnRight };
enum class StopKind : uint8_t { ByTime, ByDistance, UntilNextTrigger };
enum class TriggerKind : uint8_t { None, LineMask, DistanceTravelled };

struct StopCondition {
	StopKind kind;
	uint32_t timeMs;
	float distanceInches;
};

struct RouteTrigger {
	TriggerKind kind;
	uint16_t sensorMask;
	uint16_t lineThreshold;
	float travelInches;
};

struct RouteActionA {
	RouteActionKind kind;
	int16_t speed;
	StopCondition stop;
};

struct RouteSpeedSegment {
	int16_t speed;
	StopCondition stop;
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
	RouteTrigger t = {TriggerKind::None, 0, 0, 0.f};
	return t;
}

inline RouteTrigger makeLineTrigger(uint16_t sensorMask, uint16_t lineThreshold) {
	RouteTrigger t = {TriggerKind::LineMask, sensorMask, lineThreshold, 0.f};
	return t;
}

inline RouteTrigger makeDistanceTrigger(float travelInches) {
	RouteTrigger t = {TriggerKind::DistanceTravelled, 0, 0, travelInches};
	return t;
}

inline RouteActionA makeNoAction() {
	RouteActionA a = {RouteActionKind::None, 0, stopByTime(0)};
	return a;
}

inline RouteActionA makeActionForward(int16_t speed, const StopCondition& stop) {
	RouteActionA a = {RouteActionKind::Forward, speed, stop};
	return a;
}

inline RouteActionA makeActionBackward(int16_t speed, const StopCondition& stop) {
	RouteActionA a = {RouteActionKind::Backward, speed, stop};
	return a;
}

inline RouteActionA makeActionTurnLeft(int16_t speed, const StopCondition& stop) {
	RouteActionA a = {RouteActionKind::TurnLeft, speed, stop};
	return a;
}

inline RouteActionA makeActionTurnRight(int16_t speed, const StopCondition& stop) {
	RouteActionA a = {RouteActionKind::TurnRight, speed, stop};
	return a;
}

inline RouteSpeedSegment makeSpeedSegment(int16_t speed) {
	RouteSpeedSegment segment = {speed, stopUntilNextTrigger()};
	return segment;
}

inline RouteSpeedSegment makeSpeedSegment(int16_t speed, const StopCondition& stop) {
	RouteSpeedSegment segment = {speed, stop};
	return segment;
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
	void setLineFollowTunings(float kp, float ki, float kd);

	bool finished() const { return state_ == RouteRunnerState::Finished; }
	uint16_t currentIndex() const { return index_; }
	RouteRunnerState state() const { return state_; }

private:
	static float maxWheelDistance_(GeekoBot& robot);
	static bool lineTriggerFired_(const RouteTrigger& tr, int irVals[9]);
	static bool distanceTriggerFired_(const RouteTrigger& tr, float baseline, GeekoBot& robot);
	static bool stopIsNoOp_(const StopCondition& stop);
	static bool stopIsUntilNextTrigger_(const StopCondition& stop);
	static bool stopSatisfied_(const StopCondition& stop, unsigned long startMs, float startDist, GeekoBot& robot);

	bool nextStepTriggerFired_(GeekoBot& robot);
	bool speedSegmentDone_(const StopCondition& stop, unsigned long startMs, float startDist, GeekoBot& robot);
	void advanceToNextStepAction_(GeekoBot& robot);

	void enterWaitingTrigger_(GeekoBot& robot);
	void skipActionAndBeginSpeedSegments_(GeekoBot& robot, const RouteStep& step);
	void startSegment_(GeekoBot& robot);
	void applyActionATick_(GeekoBot& robot, const RouteActionA& action);
	void applyActionForwardControlTick_(GeekoBot& robot, int16_t baseSpeed);
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
	bool rearmTriggerBaseline_ = true;

	float actionForwardKp_ = 8.0f;
	int16_t maxActionForwardCorrection_ = 40;

	float lineFollowKp_ = 0.13f;
	float lineFollowKi_ = 0.0f;
	float lineFollowKd_ = 0.15f;

	PIDController lineFollowPid_;
	PIDController actionForwardPid_;

	int irVals_[9];
};

#endif
