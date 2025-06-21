#include "MotorEncoder.h"


void MotorEncoder::begin(int motorC1, int motorC2) {
    motorC1Pin = motorC1;
    motorC2Pin = motorC2;

    pinMode(motorC1Pin, INPUT);
    pinMode(motorC2Pin, INPUT);
}


void MotorEncoder::increment() {
    noInterrupts();
    tickCount++;
    interrupts();
}


void MotorEncoder::decrement() {
    noInterrupts();
    tickCount--;
    interrupts();
}


void MotorEncoder::reset() {
    noInterrupts();
    tickCount = 0;
    interrupts();
}


long MotorEncoder::getTicks() {
    return tickCount;
}


void MotorEncoder::attachEncoderInterrupt(void (*isr)()) {
    Serial.println("Attaching encoder interrupt");
    attachInterrupt(digitalPinToInterrupt(motorC1Pin), isr, CHANGE);
}


void MotorEncoder::update() {
    unsigned long now = millis();
    unsigned long deltaTime = now - lastUpdate;
    if (deltaTime >= 200) {
        // Serial.print(tickCount);
        // Serial.print("\t");
        noInterrupts();
        long ticks = tickCount - lastTickCount;
        interrupts();
        // Serial.print(ticks);
        // Serial.print("\t");
        // Serial.print(ticks / ticksPerRevolution);
        // Serial.print("\t");
        // Serial.print(deltaTime);
        // Serial.print("\t");


        rpm = (float(ticks) / float(ticksPerRevolution)) * (60000.00 / float(deltaTime));
        lastTickCount = tickCount;
        lastUpdate = now;

        // Serial.println(rpm);
    }
}


float MotorEncoder::getRpm() {
    return rpm;
}


int MotorEncoder::getTicksPerRevolution() {
    return ticksPerRevolution;
}


void MotorEncoder::setWheelDiameter(float diameter) {
    wheelDiameter = diameter;
}


float MotorEncoder::getWheelDiameter() {
    return wheelDiameter;
}


float MotorEncoder::getDistance() {
	float circumference = 3.14159 * wheelDiameter; // in inches
	return (getTicks() / float(ticksPerRevolution)) * circumference; // return distance in inches
}