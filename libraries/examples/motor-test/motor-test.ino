
#include <GeekoBot.h>


const int MOTOR_RPM = 1500;
const float WHEEL_DIAMETER = 1.1;


GeekoBot robot;

void setup() {
  // Declare the motor pins as output
  pinMode(L_MOTOR_1, OUTPUT);
  pinMode(L_MOTOR_2, OUTPUT);
  pinMode(L_MOTOR_PWM, OUTPUT);

  pinMode(R_MOTOR_1, OUTPUT);
  pinMode(R_MOTOR_2, OUTPUT);
  pinMode(R_MOTOR_PWM, OUTPUT);
}

void loop() {
  // Turn LEFT motor in reverse @ 50% speed; 255 PWM value is the MAX speed
  digitalWrite(L_MOTOR_2, LOW);
  digitalWrite(L_MOTOR_1, HIGH);
  analogWrite(L_MOTOR_PWM, 127); 

  // Turn RIGHT motor forward @ 50% speed; 255 PWM value is the MAX speed
  digitalWrite(R_MOTOR_1, HIGH);
  digitalWrite(R_MOTOR_2, LOW);
  analogWrite(R_MOTOR_PWM, 127);

  delay(1000);

  // Turn LEFT motor forward @ 50% speed; 255 PWM value is the MAX speed
  digitalWrite(L_MOTOR_2, HIGH);
  digitalWrite(L_MOTOR_1, LOW);
  analogWrite(L_MOTOR_PWM, 127);

  // Turn RIGHT motor in reverse @ 50% speed; 255 PWM value is the MAX speed
  digitalWrite(R_MOTOR_1, LOW);
  digitalWrite(R_MOTOR_2, HIGH);
  analogWrite(R_MOTOR_PWM, 127);

  delay(1000);
}