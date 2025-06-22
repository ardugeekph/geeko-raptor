#ifndef EncoderCounter_h
#define EncoderCounter_h

#include <Arduino.h>


class MotorEncoder {
    public:
        void begin(int motorC1, int motorC2);
        void attachEncoderInterrupt(void (*isr)());
        void increment();
        void decrement();
        void reset();
        long getTicks();
        void update();
        float getRpm();
        int getTicksPerRevolution();
        float getDistance();
        void setWheelDiameter(float diameter = 1.2);
        float getWheelDiameter();

    private:
        int motorC1Pin;
        int motorC2Pin;

        volatile long tickCount = 0;
        volatile long lastTickCount = 0;
        unsigned long lastUpdate = 0;
        float rpm = 0;
        int ticksPerRevolution = 135;
        float wheelDiameter = 0;
};

#endif
