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
        float getRpm();
        float getDistance();
        float getDirectionalDistance();
        void setWheelDiameter(float diameter = 1.2);
        float getWheelDiameter();
        void update();

    private:
        int motorC1Pin;
        int motorC2Pin;

        volatile long tickCount = 0;
        volatile long lastTickCount = 0;
        unsigned long lastUpdate = 0;
        volatile long cummulativeTickCount = 0;
        float rpm = 0;
        int ticksPerRevolution = 135;
        float wheelDiameter = 0;
};

#endif
