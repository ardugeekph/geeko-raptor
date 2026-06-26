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

## RouteRunner (declarative routes)

Define a table of `RouteStep` entries using the schema:
`Trigger -> Action -> SpeedA -> [optional SpeedB]`.
The library runs a non-blocking state machine; your sketch only calls `update` then `tick`.

**Contract:** call `GeekoBot::update()` **before** `RouteRunner::tick(robot)` every `loop()` iteration. `tick()` does not update encoders for you.

```cpp
#include <GeekoBot.h>
#include <RouteRunner.h>

GeekoBot robot;
RouteRunner runner;

const RouteStep PLAN[] = {
  makeStep(
    makeNoTrigger(),
    makeNoAction(),
    makeSpeedSegment(130, stopByTime(900)),
    makeSpeedSegment(90, stopByDistance(2.0f))
  ),
};

void setup() {
  robot.begin(1500, 1.1);
  robot.motorLeft.encoder.attachEncoderInterrupt(leftEncoderISR);
  robot.motorRight.encoder.attachEncoderInterrupt(rightEncoderISR);
  runner.begin(PLAN, sizeof(PLAN) / sizeof(PLAN[0]));
}

void loop() {
  robot.update();
  runner.tick(robot);
}
```

### Behaviour summary

- **While waiting for a trigger:** both motors are **stopped**. Use `makeNoTrigger()` to skip waiting and start the step immediately (useful for the first step). Distance triggers measure `max(left, right)` cumulative `getDistance()` since that wait started.
- **Line trigger:** `makeLineTrigger(sensorMask, lineThreshold)` — every masked bit (0–8 = IR channels, matching `readIrCalibrated`) must read **≥** `lineThreshold` on the 0–1023 calibrated scale. `sensorMask == 0` never fires.
- **Stop conditions:** use `stopByTime(ms)` or `stopByDistance(inches)` on `Action` and on `Speed` when you want a timed or distance limit. `stopByTime(0)` / `stopByDistance(0)` skip that segment immediately.
- **Speed until next step:** `makeSpeedSegment(speed)` (or `makeSpeedSegment(speed, stopUntilNextTrigger())`) line-follows at `speed` until the **next** step’s trigger fires, then starts that step’s `Action` without stopping in between. Use `makeSpeedSegment(speed, lineFollowPID(kp, ki, kd))` for the same behaviour with per-segment PID.
- **Action:** semantic action with speed and stop condition, e.g. `makeActionForward(speed, stop)`, `makeActionTurnLeft(speed, stop)`. Use `makeNoAction()` to skip the action phase and go straight to SpeedA line-follow.
- **SpeedA / SpeedB:** after `Action` ends, runner enters line-follow mode for SpeedA then optionally SpeedB. Use the 3-arg `makeStep(trigger, action, speedA)` to omit SpeedB. With a finite SpeedA stop (`stopByTime` / `stopByDistance`), the robot **stops** and waits for the next step’s trigger; with `makeSpeedSegment(speed)` (until-next-trigger), it keeps moving until that trigger fires. If SpeedA uses until-next-trigger in a 4-arg step, SpeedB is skipped when the next step’s trigger fires.
- **Turn semantics:** turns apply opposite polarity automatically (`TurnLeft => left=-speed,right=+speed`, `TurnRight => left=+speed,right=-speed`).
- **Line-follow tuning:** set global defaults with `runner.setLineFollowTunings(kp, ki, kd)`, or per speed segment with `lineFollowPID(kp, ki, kd)` as the second argument to `makeSpeedSegment(speed, pid)` or the third argument to `makeSpeedSegment(speed, stop, pid)`. Segments without `lineFollowPID(...)` use the runner defaults.
- **Line-follow mode:** use `LineFollowMode::BlackOnWhite` (default) for a dark line on a light surface, or `LineFollowMode::WhiteOnBlack` for a light line on a dark surface. Pass as the optional trailing argument to any `makeSpeedSegment(...)` overload, e.g. `makeSpeedSegment(130, stopByTime(900), LineFollowMode::WhiteOnBlack)`. This uses inverted `getPos(true)` internally, including correct out-of-bounds search direction. Line triggers may need different `lineThreshold` values on white-on-black tracks.
- **Resume API:** use `runner.setIndex(index, robot)` to jump to any step and re-enter `WaitingTrigger` safely. Use `runner.stepCount()` for bounds checks.

See `examples/route_runner_demo/route_runner_demo.ino` for a full plan and encoder ISRs.

### Resume from index (IR in sketch layer)

Keep IR decoding in your sketch/app code, then call the runner resume API:

```cpp
#include <IRremote.h>
IrReceiver.begin(IR_REMOTE_SENSOR);

if (IrReceiver.decode()) {
  uint32_t code = IrReceiver.decodedIRData.decodedRawData;
  int target = -1;
  if (code == 0xFF6897) target = 0; // key 0
  else if (code == 0xFF30CF) target = 1; // key 1
  else if (code == 0xFF18E7) target = 2; // key 2

  if (target >= 0) {
    runner.setIndex((uint16_t)target, robot);
  }
  IrReceiver.resume();
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
int position = robot.sensor.getPos();              // Black-on-white line position (-3500 to 3500)
int positionInv = robot.sensor.getPos(true);     // White-on-black (inverted readings + recovery)
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