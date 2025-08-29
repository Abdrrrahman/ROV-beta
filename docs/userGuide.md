# ROV Control System User Guide

## Table of Contents
1. [Overview](#overview)
2. [System Components](#system-components)
3. [Getting Started](#getting-started)
4. [Command Reference](#command-reference)
5. [PID Controller Tuning](#pid-controller-tuning)
6. [Thruster Control](#thruster-control)
7. [Debug and Monitoring](#debug-and-monitoring)
8. [Safety Features](#safety-features)
9. [Troubleshooting](#troubleshooting)
10. [Technical Specifications](#technical-specifications)

---

## Overview

This ROV (Remotely Operated Vehicle) control system is designed for underwater operations with 6 degrees of freedom (6-DOF) control. The system uses PID controllers to maintain precise positioning and orientation in water.

### Degrees of Freedom
- **Heave**: Vertical movement (up/down)
- **Yaw**: Rotation around vertical axis (left/right turn)
- **Surge**: Forward/backward movement
- **Sway**: Left/right lateral movement

### Control Philosophy
The system operates autonomously using sensor feedback and PID controllers to maintain target positions. You can adjust targets and tune PID parameters through serial commands.

---

## System Components

### Thrusters
The ROV uses 6 thrusters arranged in a specific configuration:

**Horizontal Thrusters (4):**
- **FR (Front Right)**: Pin 3, oriented 45° (NorthEast)
- **FL (Front Left)**: Pin 5, oriented 135° (NorthWest)  
- **BR (Back Right)**: Pin 6, oriented 315° (SouthEast)
- **BL (Back Left)**: Pin 9, oriented 225° (SouthWest)

**Vertical Thrusters (2):**
- **VR (Vertical Right)**: Pin 10, downward thrust
- **VL (Vertical Left)**: Pin 11, downward thrust

### Sensors
- **Depth Sensor**: Connected to analog pin A0 for heave control
- **IMU (Inertial Measurement Unit)**: For yaw, roll, and pitch measurements
- **Position Sensors**: Computer vision system for surge and sway positioning

### Controllers
Four independent PID controllers manage each degree of freedom with configurable parameters (Kp, Ki, Kd).

---

## Getting Started

### Initial Setup
1. Connect your Arduino to a computer via USB
2. Open the Serial Monitor (9600 baud rate)
3. Power on the ROV system
4. Wait for initialization message: "ROV System initialized."

### First Steps
1. Type `help` to see all available commands
2. Check thruster status with `debug -a`
3. Test individual thrusters with `test [thruster] [power]`
4. Begin with low power values (10-20) for safety

---

## Command Reference

### Getting Help
```
help
```
Displays all available commands with syntax.

### PID Tuning Commands

#### Heave (Depth) Control
```
heave -kp [value]     # Set proportional gain
heave -ki [value]     # Set integral gain  
heave -kd [value]     # Set derivative gain
heave -target [value] # Set target depth
```

#### Yaw (Rotation) Control
```
yaw -kp [value]       # Set proportional gain
yaw -ki [value]       # Set integral gain
yaw -kd [value]       # Set derivative gain  
yaw -target [value]   # Set target angle (-180 to 180 degrees)
```

#### Surge (Forward/Back) Control
```
surge -kp [value]     # Set proportional gain
surge -ki [value]     # Set integral gain
surge -kd [value]     # Set derivative gain
surge -target [value] # Set target position
```

#### Sway (Left/Right) Control
```
sway -kp [value]      # Set proportional gain
sway -ki [value]      # Set integral gain
sway -kd [value]      # Set derivative gain
sway -target [value]  # Set target position
```

### Thruster Control Commands

#### Testing Individual Thrusters
```
test [thruster_name] [power]
```
- **thruster_name**: FR, FL, BR, BL, VR, VL
- **power**: -100 to +100 (percentage)
- Example: `test FR 25` (test front-right thruster at 25% power)

#### Enabling/Disabling Thrusters
```
enable [thruster_name]    # Enable specific thruster
disable [thruster_name]   # Disable specific thruster
enable -a                 # Enable all thrusters
disable -a                # Disable all thrusters
enable -v                 # Enable vertical thrusters only
disable -v                # Disable vertical thrusters
enable -h                 # Enable horizontal thrusters only
disable -h                # Disable horizontal thrusters
```

#### Emergency Stop Commands
```
stop                      # Emergency stop all thrusters
emergency                 # Emergency stop all thrusters
stop [thruster_name]      # Stop specific thruster
stop -a                   # Stop all thrusters
stop -v                   # Stop vertical thrusters
stop -h                   # Stop horizontal thrusters
```

### Reset Commands
```
reset                     # Reset all PID controllers
reset -a                  # Reset all PID controllers
reset -heave              # Reset heave PID only
reset -yaw                # Reset yaw PID only
reset -surge              # Reset surge PID only
reset -sway               # Reset sway PID only
```

### Debug Commands
```
debug -dof                # Show degrees of freedom info
debug [thruster_name]     # Show specific thruster info
debug -a                  # Show all thruster information
debug -v                  # Show vertical thrusters info
debug -h                  # Show horizontal thrusters info
```

---

## PID Controller Tuning

### Understanding PID Parameters

**Proportional (Kp):**
- Controls immediate response to error
- Higher values = faster response but may cause oscillation
- Start with small values (0.1-1.0)

**Integral (Ki):**
- Eliminates steady-state error over time
- Higher values = faster elimination of persistent error
- Start with very small values (0.01-0.1)

**Derivative (Kd):**
- Dampens oscillations and improves stability
- Higher values = more damping but may slow response
- Start with small values (0.01-0.5)

### Tuning Process
1. **Start with all gains at zero**
2. **Increase Kp** until you get a reasonable response with slight oscillation
3. **Add Ki** to eliminate steady-state error
4. **Add Kd** to reduce oscillations and improve stability
5. **Fine-tune** by small increments

### Example Tuning Session
```
# Start tuning heave control
heave -kp 0.5
heave -target -1.0    # Set target to 1 meter depth
debug -dof            # Monitor response

# If oscillating, add derivative
heave -kd 0.1

# If steady-state error exists, add integral  
heave -ki 0.05
```

---

## Thruster Control

### Power Levels
- **Range**: -100 to +100 (percentage)
- **Positive values**: Forward/up thrust
- **Negative values**: Reverse/down thrust
- **Zero**: Neutral/stopped

### Testing Procedure
1. **Safety first**: Start with low power (±10-20)
2. **Test verticals**: `test VL 20`, `test VR 20`
3. **Test horizontals**: `test FR 20`, `test FL 20`, etc.
4. **Check directions**: Verify thrust directions match expectations

### Thruster Mixing
The system automatically combines control outputs:
- **Heave control**: Uses both vertical thrusters equally
- **Yaw control**: Uses horizontal thrusters differentially
- **Surge/Sway**: Uses horizontal thrusters with 45° geometry

---

## Debug and Monitoring

### Real-time Monitoring
The system automatically prints thruster power levels every second. Monitor these values to understand system behavior.

### Debug Information Types

**DOF Status (`debug -dof`):**
Shows current sensor readings vs. targets for all degrees of freedom.

**Thruster Information:**
- Power level (-100 to +100)
- PWM signal (1000-2000 microseconds)
- Enable/disable status
- Pin assignment

**System Status:**
Monitor for proper sensor readings and reasonable control outputs.

---

## Safety Features

### Emergency Procedures
- **Immediate stop**: Type `stop` or `emergency`
- **Selective stopping**: Use `stop -v` (vertical) or `stop -h` (horizontal)
- **Power limits**: All outputs automatically limited to ±100%
- **Integral windup protection**: Prevents PID integral term from growing too large

### Safe Operating Practices
1. Always test thrusters individually at low power first
2. Keep emergency stop command ready
3. Monitor thruster power outputs regularly
4. Use gradual target changes, not sudden jumps
5. Verify sensor readings make sense before trusting control

### Fail-Safe Features
- Automatic PWM signal limiting (1000-2000 μs)
- Power percentage constraints (-100 to +100)
- Thruster enable/disable functionality
- PID output limits to prevent excessive control actions

---

## Troubleshooting

### Common Issues

**ROV Not Responding to Commands:**
- Check serial connection and baud rate (9600)
- Verify thrusters are enabled: `debug -a`
- Test individual thrusters: `test FR 20`

**Oscillating Behavior:**
- Reduce Kp gain: `[axis] -kp [lower_value]`
- Increase Kd gain: `[axis] -kd [higher_value]`
- Reset PID: `reset -[axis]`

**Poor Depth Control:**
- Check depth sensor: `debug -dof`
- Verify vertical thrusters: `debug -v`
- Tune heave PID parameters

**ROV Spinning Uncontrollably:**
- Emergency stop: `stop`
- Check yaw PID gains
- Test horizontal thrusters individually

**No Sensor Readings:**
- Current sensor functions return 0.0 (placeholder)
- Implement actual sensor reading code
- Check sensor connections and power

### Diagnostic Commands
```
debug -dof            # Check sensor readings vs targets
debug -a              # Check all thruster status
stop                  # Emergency stop if needed
reset                 # Reset all PID controllers
```

---

## Technical Specifications

### Hardware Requirements
- **Microcontroller**: Arduino (Uno, Nano, etc.)
- **ESCs**: 6x Electronic Speed Controllers
- **Thrusters**: 6x brushless motors with propellers
- **Sensors**: Depth sensor, IMU, position tracking system

### PWM Signal Specifications
- **Frequency**: Standard servo frequency (~50Hz)
- **Neutral**: 1500 microseconds
- **Forward/Up**: 1500-2000 microseconds
- **Reverse/Down**: 1000-1500 microseconds

### PID Controller Limits
- **Integral Limit**: ±5.0 (prevents windup)
- **Output Limit**: ±10.0 (constrains control action)
- **Update Rate**: ~200Hz (5ms loop delay)

### Communication
- **Serial Baud Rate**: 9600 bps
- **Command Format**: Text-based with space separation
- **Case Sensitivity**: Commands are case-insensitive

---

## Quick Reference Card

### Essential Commands
| Command | Description |
|---------|-------------|
| `help` | Show all commands |
| `stop` | Emergency stop all |
| `debug -dof` | Show current status |
| `test FR 20` | Test front-right at 20% |
| `heave -target -2.0` | Set depth to 2m |
| `reset` | Reset all PIDs |

### Safety Commands
| Command | Description |
|---------|-------------|
| `emergency` | Immediate full stop |
| `disable -a` | Disable all thrusters |
| `stop -v` | Stop vertical thrusters |
| `stop -h` | Stop horizontal thrusters |

### Tuning Examples
```
# Basic depth control tuning
heave -kp 1.0
heave -ki 0.1  
heave -kd 0.2
heave -target -1.5

# Basic yaw control tuning
yaw -kp 0.8
yaw -ki 0.05
yaw -kd 0.15
yaw -target 0
```

---

## Notes for Developers

### Code Structure
- **Setup**: Initializes hardware and PID controllers
- **Main Loop**: Continuously updates control system at 200Hz
- **Sensor Updates**: Reads all sensors each cycle
- **Control Computation**: Calculates PID outputs
- **Thruster Mixing**: Converts control commands to individual thruster powers
- **Serial Processing**: Handles user commands

### Customization Points
- **Sensor Functions**: Currently return 0.0, implement actual sensor reading
- **PID Parameters**: Tune for your specific ROV characteristics  
- **Thruster Layout**: Modify mixing equations if using different geometry
- **Communication**: Add wireless communication modules as needed

### Safety Considerations
- Always implement proper waterproofing for electronics
- Test thoroughly in controlled environment before open water
- Include physical emergency stop mechanisms
- Monitor battery levels and add low-voltage protection
- Implement depth limits to prevent damage

---

*End of User Guide*

**Document Version**: 1.0  
**Last Updated**: August 29, 2025  
**Compatible with**: Arduino ROV Control System v1.0