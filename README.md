# ROV Control System

A 6-DOF (Six Degrees of Freedom) autonomous control system for underwater ROVs (Remotely Operated Vehicles) built on Arduino platform.

## Features

- **6-DOF Control**: Heave, Yaw, Surge, and Sway with independent PID controllers
- **Real-time Control**: 200Hz control loop for responsive underwater operation
- **Serial Command Interface**: Easy-to-use text commands for tuning and operation
- **Safety Systems**: Emergency stop, thruster enable/disable, and power limiting
- **Modular Design**: Easy to customize for different ROV configurations
- **Debug Tools**: Comprehensive monitoring and diagnostic capabilities

## Quick Start

### Hardware Requirements

- Arduino Uno/Nano or compatible microcontroller
- 6x Brushless thrusters with ESCs
- Depth sensor (analog output)
- IMU module (I2C interface)
- Position tracking system (computer vision recommended)

### Pin Configuration

| Component | Pin | Description |
|-----------|-----|-------------|
| Front Right Thruster | 3 | 45° orientation |
| Front Left Thruster | 5 | 135° orientation |
| Back Right Thruster | 6 | 315° orientation |
| Back Left Thruster | 9 | 225° orientation |
| Vertical Right Thruster | 10 | Downward thrust |
| Vertical Left Thruster | 11 | Downward thrust |
| Depth Sensor | A0 | Analog input |

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/rov-control-system.git
   ```

2. Open the Arduino IDE and load `rov_control.ino`

3. Install required libraries:
   - Wire.h (included with Arduino IDE)
   - Servo.h (included with Arduino IDE)

4. Upload to your Arduino board

5. Open Serial Monitor (9600 baud) and type `help` to get started

## Usage

### Basic Commands

```bash
help                    # Show all available commands
stop                    # Emergency stop all thrusters
debug -dof             # Show current status
test FR 20             # Test front-right thruster at 20% power
```

### PID Tuning Example

```bash
# Tune depth control
heave -kp 1.0          # Set proportional gain
heave -ki 0.1          # Set integral gain
heave -kd 0.2          # Set derivative gain
heave -target -2.0     # Set target depth to 2 meters

# Monitor response
debug -dof             # Check current vs target values
```

### Thruster Management

```bash
# Enable/disable thrusters
enable -a              # Enable all thrusters
disable -v             # Disable vertical thrusters
stop -h                # Stop horizontal thrusters

# Test individual thrusters
test VL 30             # Test vertical left at 30%
test FR -25            # Test front right reverse at 25%
```

## System Architecture

### Control Flow
```
Sensors → PID Controllers → Thruster Mixing → ESC Output
   ↑              ↑               ↑             ↑
Depth/IMU    Error Calc    Force Vector   PWM Signals
Position     Control       Distribution   (1000-2000μs)
```

### Degrees of Freedom

- **Heave**: Vertical movement controlled by both vertical thrusters
- **Yaw**: Rotation around Z-axis using differential horizontal thrust
- **Surge**: Forward/backward movement using coordinated horizontal thrusters
- **Sway**: Left/right movement using coordinated horizontal thrusters

### Thruster Mixing Algorithm

The system uses a 45-degree thruster arrangement for optimal maneuverability:
- Each horizontal thruster contributes to multiple DOFs
- Vertical thrusters handle heave independently
- Mathematical mixing ensures smooth, coordinated movement

## Configuration

### PID Tuning Guidelines

1. **Start Conservative**: Begin with low gains (Kp=0.1, Ki=0.01, Kd=0.05)
2. **Tune Proportional First**: Increase Kp until you get reasonable response
3. **Add Integral**: Increase Ki to eliminate steady-state error
4. **Add Derivative**: Increase Kd to reduce oscillations

### Safety Limits

- Power output: -100% to +100%
- PWM signals: 1000μs to 2000μs
- PID integral windup protection: ±5.0
- PID output limits: ±10.0

## Development
### Adding New Commands

Extend the `checkForSerialCommand()` function to add custom commands:

```cpp
else if (command.startsWith("mycmd ")) {
    // Your custom command implementation
}
```

### Customizing Thruster Layout

Modify the `mixThrusters()` function if using a different thruster arrangement. The current implementation assumes 45-degree positioning.

## Monitoring

The system provides real-time feedback through serial output:

- **Automatic Updates**: Thruster power levels printed every second
- **Debug Commands**: Detailed system status on demand  
- **Error Messages**: Clear feedback for invalid commands
- **Status Indicators**: Thruster enable/disable states

## Troubleshooting

### Common Issues

**ROV oscillating underwater:**
```bash
# Reduce proportional gain
heave -kp 0.5
# Increase derivative gain  
heave -kd 0.3
```

**Poor depth holding:**
```bash
# Check sensor readings
debug -dof
# Test vertical thrusters
test VL 25
test VR 25
```

**Communication issues:**
- Verify 9600 baud rate
- Check USB connection
- Restart Arduino if needed

## Contributing

Contributions are welcome! Please read our [Contributing Guide](docs/CONTRIBUTING.md) for details on:

- Code style and conventions
- Testing procedures  
- Pull request process
- Issue reporting

### Development Setup

1. Fork the repository
2. Create a feature branch
3. Test thoroughly with hardware
4. Submit pull request with detailed description

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

- **Documentation**: See the [User Guide](docs/USER_GUIDE.md) for detailed instructions
- **Issues**: Report bugs and request features via GitHub Issues

## Roadmap

- [ ] Add wireless communication support
- [ ] Create mission planning framework
- [ ] Develop GUI control interface
- [ ] Add data logging capabilities
- [ ] Implement advanced control modes (station keeping, waypoint following)

## Acknowledgments

- Arduino community for excellent servo and wire libraries
- ROV/AUV research community for control system insights
- Contributors and testers who helped improve this system

---

**Built with ❤️ for the underwater robotics community**