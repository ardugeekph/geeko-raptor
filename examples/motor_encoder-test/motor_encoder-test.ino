
#include <GeekoBot.h>


#define MOTOR_RPM 1500
#define WHEEL_DIAMETER 1.1

GeekoBot robot;

void setup() {
  Serial.begin(9600);

  // Initialize left and right motors
  robot.begin(MOTOR_RPM, WHEEL_DIAMETER);
	robot.motorLeft.begin(L_MOTOR_1, L_MOTOR_2, L_MOTOR_PWM, L_MOTOR_C1, L_MOTOR_C2, MOTOR_RPM, WHEEL_DIAMETER);
	robot.motorRight.begin(R_MOTOR_2, R_MOTOR_1, R_MOTOR_PWM, R_MOTOR_C1, R_MOTOR_C2, MOTOR_RPM, WHEEL_DIAMETER);

  // Attach the interrupt routines for reading the motor encoder pulses
  robot.motorLeft.encoder.attachEncoderInterrupt(leftEncoderISR);
  robot.motorRight.encoder.attachEncoderInterrupt(rightEncoderISR);
}

void loop() {
  // Update the encoders
  robot.update();

  // Set PWM Speed (0-255)
  robot.motorLeft.setSpeed(150);
  robot.motorRight.setSpeed(150);

  // Print motor RPMs
  Serial.print("LEFT RPM: ");
  Serial.print(robot.motorLeft.encoder.getRpm());
  Serial.print("\t RIGHT RPM: ");
  Serial.println(robot.motorRight.encoder.getRpm());
}

// Left Motor Interrupt Service Routine (ISR)
void leftEncoderISR() {
  bool A = digitalRead(L_MOTOR_C1);
  bool B = digitalRead(L_MOTOR_C2);

  if (A == B) {
    robot.motorLeft.encoder.decrement();
  } else {
    robot.motorLeft.encoder.increment();
  }
}

// Left Motor Interrupt Service Routine (ISR)
void rightEncoderISR() {
  bool A = digitalRead(R_MOTOR_C1);
  bool B = digitalRead(R_MOTOR_C2);

  if (A == B) {
    robot.motorRight.encoder.increment();
  } else {
    robot.motorRight.encoder.decrement();
  }
}
