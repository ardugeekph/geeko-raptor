#ifndef PIDController_h
#define PIDController_h

#include <Arduino.h>


class PIDController {
    public:
        void setConstants(float kP, float kI, float kD);
        float output(float error);

    private:
		float kP_, kI_, kD_;
		float lastError;
        unsigned long lastUpdate = 0;
		float pOut = 0, iOut = 0, dOut = 0;
		float pid_output = 0;
};

#endif
