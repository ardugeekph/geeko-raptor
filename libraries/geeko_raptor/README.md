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

### **PIDController (For motor correction)**

```cpp

#include <PIDController.h>

PIDController pid;

void setup() {
  ...
  pid.setConstants(kP, kI, kD);
  ...
}

void loop() {
  ...
  int correction = pid.output(error);
  ...
}
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


## Hardware Pin Definitions

### **LEDs**
- `LED_LEFT` = 6 (PD6)
- `LED_RIGHT` = 5 (PD5)

### **Buzzer**
- `BUZZER_PIN` = 13 (PB5)

### **IR Sensors**
- `IR_BACK_SENSOR` = A7 (ADC7)
- `IR_MUX_OUTPUT` = A6 (ADC6)
- `IR_MUX_A` = A3 (ADC3) - Multiplexer control pin A
- `IR_MUX_B` = A4 (ADC4) - Multiplexer control pin B
- `IR_MUX_C` = A5 (ADC5) - Multiplexer control pin C
- `IR_REMOTE_SENSOR` = A2 (ADC2)

### **Left Motor**
- `L_MOTOR_1` = 8 (PB0)
- `L_MOTOR_2` = 7 (PD7)
- `L_MOTOR_PWM` = 9 (PB1)

### **Right Motor**
- `R_MOTOR_1` = 11 (PB3)
- `R_MOTOR_2` = 12 (PB4)
- `R_MOTOR_PWM` = 10 (PB2)

### **Motor Encoders**
- `L_MOTOR_C1` = 2 (PD2) - Left motor encoder channel 1
- `L_MOTOR_C2` = A0 (PC0) - Left motor encoder channel 2
- `R_MOTOR_C1` = 3 (PD3) - Right motor encoder channel 1
- `R_MOTOR_C2` = A1 (PC1) - Right motor encoder channel 2

### **Board Specifications**
- **Microcontroller**: ATmega328P-AU SMD
- **Digital Pins**: 20 total
- **Analog Inputs**: 6 total (A0-A5)
- **PWM Pins**: 3, 5, 6, 9, 10, 11
- **Crystal Frequency**: 16 MHz
- **Upload Speed**: 115200 baud

### **Special Features**
- **Interrupt Pins**: 2 (digital pin 2) and 3 (digital pin 3)
- **SPI Pins**: SS(10), MOSI(11), MISO(12), SCK(13)
- **I2C Pins**: SDA(A4), SCL(A5)

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