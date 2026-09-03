
 
#ifndef GeekoBot_h
#define GeekoBot_h
 
#include <Arduino.h>
#include "SensorArray.h"
#include "MotorController.h"
#include "PIDController.h"
#include "Buzzer.h"

static constexpr float GEEKO_WHEEL_DIAMETER_IN = 1.1f;
static constexpr float GEEKO_TRACK_WIDTH_CM = 10.0f;
static constexpr float GEEKO_TRACK_WIDTH_IN = GEEKO_TRACK_WIDTH_CM / 2.54f;

class GeekoBot{
	public:
		MotorController motorLeft;
		MotorController motorRight;
		SensorArray sensor;
		Buzzer buzzer;

		void begin(
			int motorRpm = 2000,
			float wheelDiameter = GEEKO_WHEEL_DIAMETER_IN,
			float trackWidth = GEEKO_TRACK_WIDTH_IN
		);
		void calibrateSensors();
		void calibrateStraightDrive();
		void moveStraight(int rpm, bool (*stopCallback)());
		void stop();
		void update();

		float getTrackWidth() const { return trackWidth_; }
		float turnTargetWheelDiffInches(float degrees) const;
		long turnTargetTickDiff(float degrees) const;
		int16_t getStraightPwmTrim() const { return straightPwmTrim_; }
		bool hasStraightCalibration() const { return straightCalValid_; }

	private:
		void saveStraightCalibration_(int16_t pwmTrim);
		void restoreStraightCalibration_();

		PIDController moveStraightPIDController;
		float trackWidth_ = GEEKO_TRACK_WIDTH_IN;
		int16_t straightPwmTrim_ = 0;
		bool straightCalValid_ = false;
};

#endif
