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
- **Line trigger (mask):** `makeLineTrigger(sensorMask, lineThreshold)` — every masked bit (0–8 = IR channels, matching `readIrCalibrated`) must read **≥** `lineThreshold` on the 0–1023 calibrated scale. `sensorMask == 0` never fires. Use `makeLineTrigger(mask, LineLevel::Mid)` for named thresholds (`Low` = 256, `Mid` = 512, `High` = 768). Pass `LinePolarity::Light` as the third argument for a light line on a dark surface (inverts readings before compare).
- **Line trigger (rules):** `makeLineRulesTrigger(rules, TriggerLogic::All)` — per-channel `min`/`max` on the calibrated scale. `TriggerLogic::All` requires every rule to match; `TriggerLogic::Any` fires when any rule matches. Array length is deduced automatically — do not pass a count. No `LinePolarity` on rules; tune `min`/`max` from live readings. Example:
  ```cpp
  static const IrChannelRule kLeftJunction[] = {
    irRule(IrChannel::F1, 400, 1023),
    irRule(IrChannel::F2, 400, 1023),
    irRule(IrChannel::F6, 0, 150),
  };
  makeLineRulesTrigger(kLeftJunction, TriggerLogic::All)
  ```
- **Line trigger masks:** preset constants in `LineTriggers.h` — front layout is `0` (outer left) through `7` (outer right), `8` = back center. Presets: `IR_MASK_OUTER_LEFT` (0), `IR_MASK_INNER_LEFT` (1–3), `IR_MASK_INNER_RIGHT` (4–6), `IR_MASK_OUTER_RIGHT` (7), `IR_MASK_OUTERS` (0,7), `IR_MASK_T_JUNCTION` (1–6), `IR_MASK_T_JUNCTION_BACK` (1–6,8). Build custom masks with `irMask(IrChannel::F2, IrChannel::F3)` etc.
- **Tuning trigger values:** run `examples/ir_calibrated_monitor/ir_calibrated_monitor.ino`, place the robot at the trigger pose, and read Serial output (`robot.sensor.printIrCalibrated()`). Use those numbers to set each `irRule(ch, min, max)`.
- **Stop conditions:** use `stopByTime(ms)` or `stopByDistance(inches)` on `Action` and on `Speed` when you want a timed or distance limit. `stopByTime(0)` / `stopByDistance(0)` skip that segment immediately.
- **Speed until next step:** `makeSpeedSegment(speed)` (or `makeSpeedSegment(speed, stopUntilNextTrigger())`) line-follows at `speed` until the **next** step’s trigger fires, then starts that step’s `Action` without stopping in between. Use `makeSpeedSegment(speed, lineFollowPID(kp, ki, kd))` for the same behaviour with per-segment PID.
- **Next-trigger arm (curved track):** on narrow sensor arrays (~80 mm), the next step’s line trigger can fire early on curves while still in a until-next-trigger segment. Delay when the next trigger is checked with `armNextTriggerAfterDistance(inches)` or `armNextTriggerAfterTime(ms)` as the last argument to `makeSpeedSegment(...)`. The arm timer starts when **that** speed segment begins. Start with ~1.0–2.0 in on curved sections and tune on your track.
  ```cpp
  makeSpeedSegment(255, armNextTriggerAfterDistance(1.5f))
  makeSpeedSegment(150, lineFollowPID(0.13f, 0.0f, 0.15f), armNextTriggerAfterDistance(1.5f))
  makeSpeedSegment(130, stopUntilNextTrigger(), lineFollowPID(0.13f, 0.0f, 0.15f),
                   armNextTriggerAfterDistance(1.5f))
  ```
- **Action:** semantic action with speed and stop condition, e.g. `makeActionForward(speed, stop)`, `makeActionBackward(speed, stop)`. Turns use degrees: `makeActionTurnLeft(speed, degrees)`, `makeActionTurnRight(speed, degrees)`. Use `makeNoAction()` to skip the action phase and go straight to SpeedA line-follow.
- **Action-only steps:** use `makeStep(action)` or `makeStep(trigger, action)` to omit speed segments (both default to `makeNoSpeedSegment()`). Chain manual moves — forward, backward, turn — and the runner advances **without stopping** between steps that use `makeNoTrigger()` (the default for 1-arg `makeStep`). When the next step has a real trigger (line, distance, rules), motors stop and wait as usual.
  ```cpp
  const RouteStep PLAN[] = {
    makeStep(makeActionForward(200, stopByDistance(2.0f))),
    makeStep(makeActionBackward(200, stopByDistance(1.0f))),
    makeStep(makeActionTurnLeft(200, 90.0f)),
  };
  ```
- **Turn semantics:** in-place turns apply opposite wheel PWM (`TurnLeft => left=-speed,right=+speed`). Stop when encoder wheel differential reaches `(degrees × π/180) × trackWidth` using `getDirectionalDistance()`. Default track width is `GEEKO_TRACK_WIDTH_IN` (10 cm between wheel centers). Optional `runner.setTurnScale(f)` if turns are consistently short/long (slip).
- **Straight action skew:** forward and backward actions use differential PID on wheel distance error plus EEPROM trim from `calibrateStraightDrive()`. Run once via `examples/straight_calibrate/straight_calibrate.ino` (uncomment `robot.calibrateStraightDrive()`): LEDs blink and buzzer beeps for 10 s while you manually push the robot straight; trim is saved to EEPROM and auto-loaded in `begin()`. RouteRunner reads `robot.getStraightPwmTrim()` each tick. Tune with `runner.setActionForwardControl(kp, ki, maxCorrection)`.
- **SpeedA / SpeedB:** after `Action` ends, runner enters line-follow mode for SpeedA then optionally SpeedB. Use `makeStep(action)` or `makeStep(trigger, action)` for action-only steps, or the 3-arg `makeStep(trigger, action, speedA)` to omit SpeedB. With a finite SpeedA stop (`stopByTime` / `stopByDistance`), the robot **stops** and waits for the next step’s trigger; with `makeSpeedSegment(speed)` (until-next-trigger), it keeps moving until that trigger fires. If SpeedA uses until-next-trigger in a 4-arg step, SpeedB is skipped when the next step’s trigger fires.
- **Line-follow tuning:** set runner defaults with `runner.setLineFollowPID(kp, ki, kd)`, or per speed segment with `lineFollowPID(kp, ki, kd)` as the second argument to `makeSpeedSegment(speed, pid)` or the third argument to `makeSpeedSegment(speed, stop, pid)`. Segments without `lineFollowPID(...)` use the runner defaults.
- **Line polarity:** use `LinePolarity::Dark` (default) for a dark line on a light surface, or `LinePolarity::Light` for a light line on a dark surface. Pass as the optional trailing argument to any `makeSpeedSegment(...)` overload or as the third argument to `makeLineTrigger(...)`, e.g. `makeSpeedSegment(130, stopByTime(900), LinePolarity::Light)` or `makeLineTrigger(IR_MASK_INNER_LEFT, LineLevel::Mid, LinePolarity::Light)`. Speed segments use inverted `getPos(true)` internally; mask line triggers invert each masked channel reading (`1023 - value`) before threshold compare.
- **Resume API:** use `runner.setIndex(index, robot)` to jump to any step and re-enter `WaitingTrigger` safely. Use `runner.stepCount()` for bounds checks.

See `examples/route_runner_demo/route_runner_demo.ino` for a full plan and encoder ISRs. In that demo, **OK** starts/restarts the route; **\*** runs `calibrateSensors()` and **#** runs `calibrateStraightDrive()`; **0** stops the route; keys **1 / 2** resume at step index (after OK).

### Resume from index (IR in sketch layer)

Keep IR decoding in your sketch/app code, then call the runner resume API:

```cpp
#include <IRremote.h>
#include <IrRemoteKeys.h>
IrReceiver.begin(IR_REMOTE_SENSOR);

if (IrReceiver.decode()) {
  uint32_t code = IrReceiver.decodedIRData.decodedRawData;
  if (code == IR_KEY_OK) {
    routeStarted = true;
    runner.reset();
    robot.stop();
  } else if (code == IR_KEY_STAR) {
    robot.stop();
    robot.calibrateSensors();
  } else if (code == IR_KEY_HASH) {
    robot.stop();
    robot.calibrateStraightDrive();
  } else if (code == IR_KEY_0) {
    routeStarted = false;
    robot.stop();
  } else if (routeStarted) {
    int target = -1;
    if (code == IR_KEY_1) target = 1;
    else if (code == IR_KEY_2) target = 2;
    if (target >= 0) {
      runner.setIndex((uint16_t)target, robot);
    }
  }
  IrReceiver.resume();
}
```

### IrRemoteKeys (`IrRemoteKeys.h`)

`decodedRawData` constants for the Geeko Raptor IR remote. Include with `IRremote.h` in sketches:

```cpp
#include <IrRemoteKeys.h>

// Digits 0–9: IR_KEY_0 .. IR_KEY_9
// * # : IR_KEY_STAR, IR_KEY_HASH
// D-pad: IR_KEY_UP, IR_KEY_DOWN, IR_KEY_LEFT, IR_KEY_RIGHT, IR_KEY_OK
```

Debug unknown codes: `Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);`

## Essential Functions

### GeekoBot (`robot`)
```cpp
// Geometry defaults: GEEKO_WHEEL_DIAMETER_IN (1.1), GEEKO_TRACK_WIDTH_CM (10), GEEKO_TRACK_WIDTH_IN (~3.94)
robot.begin(int motorRpm = 2000, float wheelDiameter = GEEKO_WHEEL_DIAMETER_IN,
            float trackWidth = GEEKO_TRACK_WIDTH_IN);
robot.calibrateSensors();                                     // Calibrate IR sensors
robot.calibrateStraightDrive();                               // Manual 10s straight cal → EEPROM
robot.getStraightPwmTrim();                                   // Trim loaded from EEPROM in begin()
robot.hasStraightCalibration();                               // true if EEPROM magic present
robot.getTrackWidth();                                        // Track width in inches
robot.turnTargetWheelDiffInches(float degrees);               // Wheel diff for in-place turn
robot.moveStraight(int rpm, stopCallback);                    // Move straight with PID
robot.stop();                                                  // Stop both motors
robot.update();                                                // Update encoders and sensors
```

### SensorArray (`robot.sensor`)
```cpp
int irVals[9];
robot.sensor.readIrRaw(irVals);                    // Read raw sensor values (0-1023)
robot.sensor.readIrCalibrated(irVals);             // Read calibrated values (0-1023)
robot.sensor.printIrCalibrated(Serial);            // Print all channels (for tuning LineRules)
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
robot.motorLeft.reverse();                                   // Swap direction pins + flip encoder signed counting
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
bool inverted = robot.motorLeft.encoder.isCountInverted();       // true after motor.reverse()
```

### Buzzer (`robot.buzzer`)
```cpp
robot.buzzer.beep(bool on);                        // Turn buzzer on/off
robot.buzzer.pulse(durationMs);                    // Short non-blocking beep (default 80 ms)
```

Call `robot.update()` each loop so `buzzer.tick()` can end a pulse without blocking. RouteRunner beeps once per completed step automatically.

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
- EEPROM layout: bytes 0–38 IR calibration; byte 39 magic `0x53`, bytes 40–41 straight-drive `pwmTrim` (int16)
- All distance measurements are in inches
- RPM control includes built-in acceleration limiting
- Encoder interrupts are required for accurate RPM calculation
- Default motor RPM: 1500, Wheel diameter: 1.1 inches

## More Information
Check the examples folder for complete working examples and additional usage patterns.