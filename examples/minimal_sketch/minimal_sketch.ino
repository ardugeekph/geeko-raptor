#include <GeekoBot.h>
#include <PIDController.h>

#define kP 0.15
#define kI 0.000
#define kD 0.15
#define MOTOR_RPM 1500
#define WHEEL_DIAMETER 1.1

GeekoBot robot;
PIDController pid;

int basePWMSpeed = 0;


void setup() {
  // Initialize robot
  robot.begin(MOTOR_RPM, WHEEL_DIAMETER);
  robot.motorLeft.encoder.attachEncoderInterrupt(leftEncoderISR);
  robot.motorRight.encoder.attachEncoderInterrupt(rightEncoderISR);

  // Set PID constants
  pid.setConstants(kP, kI, kD);

  // Uncomment if you need to calibrate the sensors
  // robot.calibrateSensors();
}

void loop() {
  // 1. This updates the motor encoders
  robot.update();

  // 2. Get robot position
  int pos = robot.sensor.getPos();
  
  // 3. Get PID corrections
  int correction = pid.output(pos);
  int leftSpeed = basePWMSpeed + correction;
  int rightSpeed = basePWMSpeed - correction;
  robot.motorLeft.setSpeed(basePWMSpeed + correction);
  robot.motorRight.setSpeed(basePWMSpeed - correction);  
}

// Interrupt Service Routine (ISR) for the motor encoders
void leftEncoderISR() {
  bool A = digitalRead(L_MOTOR_C1);
  bool B = digitalRead(L_MOTOR_C2);

  if (A == B) {
    robot.motorLeft.encoder.decrement();
  } else {
    robot.motorLeft.encoder.increment();
  }
}

void rightEncoderISR() {
  bool A = digitalRead(R_MOTOR_C1);
  bool B = digitalRead(R_MOTOR_C2);

  if (A == B) {
    robot.motorRight.encoder.increment();
  } else {
    robot.motorRight.encoder.decrement();
  }
}
