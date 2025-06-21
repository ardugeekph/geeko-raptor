
 
#ifndef GeekoBot_h
#define GeekoBot_h
 
#include <Arduino.h>
#include "SensorArray.h"
#include "MotorController.h"
#include "PIDController.h"


class GeekoBot{
	public:
		MotorController motorLeft;
		MotorController motorRight;
		SensorArray sensor;

 		void begin(int motorRpm = 2000, float wheelDiameter = 1.1);
		void calibrateSensors();
		void moveStraight(int rpm);
		void stop();
		void update();

	private:
		PIDController moveStraightPIDController;
};

#endif

















