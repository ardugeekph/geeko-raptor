
 
#ifndef GeekoBot_h
#define GeekoBot_h
 
#include <Arduino.h>
#include "SensorArray.h"
#include "MotorController.h"
#include "PIDController.h"
#include "Buzzer.h"


class GeekoBot{
	public:
		MotorController motorLeft;
		MotorController motorRight;
		SensorArray sensor;
		Buzzer buzzer;

 		void begin(int motorRpm = 2000, float wheelDiameter = 1.1);
		void calibrateSensors();
		void moveStraight(int rpm, bool (*stopCallback)());
		void stop();
		void update();

	private:
		PIDController moveStraightPIDController;
};

#endif

















