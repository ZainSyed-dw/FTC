# Robot Tuning Guide

## Table of Contents
1. [Dual Odometry System](#dual-odometry-system)
2. [Odometry Tuning](#odometry-tuning)
3. [Shooter Tuning](#shooter-tuning)
4. [Troubleshooting](#troubleshooting)

## Dual Odometry System

### Overview
The robot features two independent odometry systems that work together for maximum accuracy:
- **GoBilda Odometry**: A lightweight implementation using two encoders
- **Pinpoint Odometry**: A more advanced implementation using three encoders

### Key Features
- **Real-time Comparison**: Compare both systems side by side
- **Automatic Calibration**: Tune parameters with live feedback
- **Redundancy**: If one system fails, the other can take over

### When to Use Each
- **Use GoBilda Odometry** when:
  - You need maximum performance
  - Only two encoders are available
  - Basic position tracking is sufficient

- **Use Pinpoint Odometry** when:
  - You need maximum accuracy
  - You have three encoders available
  - You need to track both position and heading precisely

## Odometry Tuning

### Prerequisites
- Robot with odometry wheels properly installed
- Measuring tape (at least 10 feet)
- Tape or markers for the floor
- Level surface for testing

### Step 1: Initial Setup
1. Connect to your robot via the Driver Station app
2. Open the "Odometry Tuner" teleop
3. Ensure all odometry wheels move freely and encoders are connected

### Step 2: Track Width Tuning (Press A)
1. Place your robot at a known starting position
2. Drive the robot forward exactly 48 inches (4 feet)
3. Note the reported distance in the telemetry
4. If the actual distance doesn't match:
   - Press A to enter track width tuning mode
   - Use D-pad up/down to adjust the track width
   - Increase value if robot went too far
   - Decrease value if robot didn't go far enough
   - Press Start to save

### Step 3: Forward Offset Tuning (Press B)
1. Make a 90-degree turn in place
2. Measure the actual angle turned
3. If the angle is incorrect:
   - Press B to enter forward offset mode
   - Use D-pad up/down to adjust
   - Increase value if turn was too large
   - Decrease value if turn was too small
   - Press Start to save

### Step 4: System Comparison (Press LB)
1. Press LB to enter comparison mode
2. Observe both odometry systems side by side
3. They should show similar values when moving
4. If they diverge significantly:
   - Check for mechanical issues
   - Verify encoder connections
   - Recalibrate the system showing more error

## Shooter Tuning

### Prerequisites
- Fully charged battery
- Game controller connected
- Safe space to test (at least 10 feet clear)

### Step 1: Initial Setup
1. Open the "Shooter Tuner" teleop
2. Ensure the shooter motor is properly connected

### Step 2: RPM Tuning
1. Press A to start the shooter
2. Note the current RPM in the telemetry
3. Use D-pad up/down to adjust target RPM
   - D-pad up: Increase RPM by 10
   - D-pad down: Decrease RPM by 10
4. Find the optimal RPM for consistent shots
5. Press Start to save the configuration

## Troubleshooting

### Odometry Issues
- **Encoders not moving**: 
  - Check connections and ensure wheels spin freely
  - Verify encoder names in code match configuration
- **Inconsistent readings**:
  - Check for wheel slip
  - Ensure floor is clean and level
  - Verify track width and wheel diameter
- **Systems disagree**:
  - Check for mechanical binding
  - Verify all encoders are functioning
  - Recalibrate both systems

### Shooter Issues
- **RPM fluctuating**: 
  - Check battery level
  - Verify motor connections
  - Check for mechanical binding
- **Inconsistent shots**:
  - Ensure all mechanical components are tight
  - Check for worn parts
  - Verify consistent power delivery

## Recommended Starting Values
- **Track Width**: 15.0 inches
- **Forward Offset**: 6.0 inches
- **Shooter RPM**: 1450
- **Wheel Diameter**: 1.38 inches (35mm)

## Code Integration

### PinpointOdometry Initialization
```java
PinpointOdometry odometry = new PinpointOdometry(
    hardwareMap,           // Hardware map from OpMode
    "left_encoder",        // Left encoder name
    "right_encoder",       // Right encoder name
    "horizontal_encoder",  // Horizontal encoder name
    15.0,                  // Track width (inches)
    6.0                    // Forward offset (inches)
);