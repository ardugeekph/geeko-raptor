# Geeko Raptor Line Follower Library - Essential Functions

## Quick Start
```cpp
#include "GeekoBot.h"
GeekoBot robot;

void setup() {
  Serial.begin(9600);
  robot.begin(1500, 1.1);          // Initialize with 1500 RPM, 1.1" wheels
  robot.calibrateSensors();        // Calibrate IR sensors
}

void loop() {
  robot.update();                  // Update encoders and sensors
  // Your line following code here
}
```

## Essential Functions

### GeekoBot (`robot`)
```cpp
robot.begin(int motorRpm = 2000, float wheelDiameter = 1.1);  // Initialize robot
robot.calibrateSensors();                                     // Calibrate IR sensors
robot.moveStraight(int rpm, stopCallback);                    // Move straight with PID
robot.stop();                                                  // Stop both motors
robot.update();                                                // Update encoders and sensors
```

### SensorArray (`robot.sensor`)
```cpp
int irVals[9];
robot.sensor.readIrRaw(irVals);                    // Read raw sensor values (0-1023)
robot.sensor.readIrCalibrated(irVals);             // Read calibrated values (0-1023)
bool isOutside = robot.sensor.isOut();             // Check if robot is off track
int position = robot.sensor.getPos();              // Get line position (-3500 to 3500)
int contrast = robot.sensor.getContrast();         // Get current contrast level
bool isCheckpoint = robot.sensor.isCheckpoint();   // Detect checkpoints
```

### MotorController (`robot.motorLeft` / `robot.motorRight`)
```cpp
robot.motorLeft.setSpeed(int pwm);                           // Set PWM speed (-255 to 255)
robot.motorLeft.setRpmSpeed(float targetRPM, float accel = 1, bool reverse = false);
robot.motorLeft.stop();                                      // Stop motor
```

### MotorEncoder (`robot.motorLeft.encoder` / `robot.motorRight.encoder`)
```cpp
// Setup encoder interrupts (required for RPM calculation)
robot.motorLeft.encoder.attachEncoderInterrupt(leftEncoderISR);
robot.motorRight.encoder.attachEncoderInterrupt(rightEncoderISR);

// Encoder ISR functions
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

// Read encoder data
float rpm = robot.motorLeft.encoder.getRpm();                    // Get current RPM
float distance = robot.motorLeft.encoder.getDistance();          // Get distance (inches)
float dirDistance = robot.motorLeft.encoder.getDirectionalDistance(); // Get signed distance
```

### Buzzer (`robot.buzzer`)
```cpp
robot.buzzer.beep(bool on);                        // Turn buzzer on/off
```

### PIDController (Internal Usage)
```cpp
// PID is used internally by the library
// You can access it through motor controllers for advanced control
robot.motorLeft.pidController.setConstants(kP, kI, kD);
float output = robot.motorLeft.pidController.output(error);
```

## Advanced Usage

### Manual Motor Initialization (Optional)
```cpp
// Note: Motors are already initialized by robot.begin()
// Only use this if you need custom pin configurations
robot.motorLeft.begin(L_MOTOR_1, L_MOTOR_2, L_MOTOR_PWM, L_MOTOR_C1, L_MOTOR_C2, MOTOR_RPM, WHEEL_DIAMETER);
robot.motorRight.begin(R_MOTOR_2, R_MOTOR_1, R_MOTOR_PWM, R_MOTOR_C1, R_MOTOR_C2, MOTOR_RPM, WHEEL_DIAMETER);
```

### Additional Encoder Functions
```cpp
robot.motorLeft.encoder.getTicks();                    // Get raw encoder ticks
robot.motorLeft.encoder.setWheelDiameter(float dia);   // Set wheel diameter
robot.motorLeft.encoder.reset();                       // Reset encoder count
```

### Additional Sensor Functions
```cpp
// Direct multiplexer control (low-level)
void selectMUXChannel(int channel) {
  digitalWrite(IR_MUX_A, (channel & 0b001) ? HIGH : LOW);
  digitalWrite(IR_MUX_B, (channel & 0b010) ? HIGH : LOW);
  digitalWrite(IR_MUX_C, (channel & 0b100) ? HIGH : LOW);
}
```

### Hardware Control
```cpp
// LED control
pinMode(LED_LEFT, OUTPUT);
pinMode(LED_RIGHT, OUTPUT);
digitalWrite(LED_LEFT, HIGH);

// Manual buzzer control
digitalWrite(BUZZER_PIN, HIGH);

// IR remote control
#include <IRremote.h>
IrReceiver.begin(IR_REMOTE_SENSOR);
```

## Typical Line Following Pattern

```cpp
void loop() {
  robot.update();
  
  int position = robot.sensor.getPos();
  
  if (robot.sensor.isOut()) {
    // Handle off-track situation
    robot.stop();
  } else {
    // Calculate motor speeds based on position
    int baseSpeed = 150;
    int correction = position / 100;  // Adjust as needed
    
    robot.motorLeft.setRpmSpeed(baseSpeed - correction);
    robot.motorRight.setRpmSpeed(baseSpeed + correction);
  }
  
  // Check for checkpoints
  if (robot.sensor.isCheckpoint()) {
    robot.buzzer.beep(true);
    delay(100);
    robot.buzzer.beep(false);
  }
}
```

## Hardware Pin Definitions

### Motor Pins
- `L_MOTOR_1`, `L_MOTOR_2`, `L_MOTOR_PWM`, `L_MOTOR_C1`, `L_MOTOR_C2` - Left motor
- `R_MOTOR_1`, `R_MOTOR_2`, `R_MOTOR_PWM`, `R_MOTOR_C1`, `R_MOTOR_C2` - Right motor

### Sensor Pins
- `IR_MUX_A`, `IR_MUX_B`, `IR_MUX_C` - Multiplexer control pins
- `IR_MUX_OUTPUT` - Multiplexer output pin
- `IR_BACK_SENSOR` - Back IR sensor pin
- `IR_REMOTE_SENSOR` - IR remote receiver pin

### Other Pins
- `BUZZER_PIN` - Buzzer control pin
- `LED_LEFT`, `LED_RIGHT` - Status LED pins

## Key Notes
- Position values range from -3500 (far left) to +3500 (far right)
- Sensor array has 9 channels (8 front + 1 back)
- Calibration data is automatically saved to EEPROM
- All distance measurements are in inches
- RPM control includes built-in acceleration limiting
- Encoder interrupts are required for accurate RPM calculation
- Default motor RPM: 1500, Wheel diameter: 1.1 inches

## More Information
Check the examples folder for complete working examples and additional usage patterns.