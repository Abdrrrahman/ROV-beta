# Contributing to ROV Control System

Thank you for your interest in contributing to the ROV Control System! This project welcomes contributions from the underwater robotics community, whether you're a beginner or an experienced developer.

## How to Contribute

There are many ways to contribute to this project:

- **Report bugs** and suggest fixes
- **Request features** or propose improvements  
- **Improve documentation** and add examples
- **Share hardware configurations** and build guides
- **Submit code improvements** and optimizations
- **Add sensor implementations** for specific hardware
- **Create testing procedures** and validation scripts

## Getting Started

### Prerequisites

- Arduino IDE or PlatformIO
- Basic understanding of C/C++ programming
- Access to ROV hardware for testing (recommended)
- Git knowledge for version control

### Development Setup

1. **Fork the repository**
   ```bash
   # Click "Fork" on GitHub, then clone your fork
   git clone https://github.com/Abdrrrahman/ROV-beta.git
   cd rov-control-system
   ```

2. **Create a feature branch**
   ```bash
   git checkout -b feature/your-feature-name
   # or
   git checkout -b fix/bug-description
   ```

3. **Set up your development environment**
   - Install Arduino IDE
   - Test compilation of existing code
   - Set up hardware if available

## Contribution Guidelines

### Code Style

**Follow Arduino conventions:**
- Use camelCase for variables and functions: `myVariable`, `readSensor()`
- Use PascalCase for structs and classes: `ThrusterSystem`, `PID`
- Use ALL_CAPS for constants: `MAX_POWER`, `MIN_PWM`
- Use descriptive variable names: `currentDepth` not `cd`

**Formatting standards:**
```cpp
// Good spacing and indentation
if (condition) {
    doSomething();
} else {
    doSomethingElse();
}

// Function documentation
/**
 * Calculates PID output for given setpoint and measurement
 * @param myPID: PID controller struct
 * @param setPoint: Target value
 * @param measuredValue: Current sensor reading
 * @return: Control output (-outputLimit to +outputLimit)
 */
float computePID(PID& myPID, float setPoint, float measuredValue) {
    // Implementation here
}
```

**Comments:**
- Comment complex algorithms and calculations
- Explain hardware-specific configurations
- Document safety-critical sections
- Use clear, concise language

### Code Quality Requirements

**Safety First:**
- Never remove safety limits or emergency stop functionality
- Test all changes with hardware when possible
- Add bounds checking for user inputs
- Document any safety implications

**Testing Requirements:**
- Test compilation on Arduino Uno/Nano
- Verify serial commands work correctly
- Test with hardware if available
- Include test procedures in pull request

**Documentation:**
- Update README if adding new features
- Add inline comments for complex code
- Update command help text if adding commands
- Include wiring diagrams for hardware changes

## Reporting Bugs

### Before Reporting
1. Check existing issues for duplicates
2. Try the latest version of the code
3. Test with minimal configuration if possible

### Bug Report Template
Include this information in your bug report:

```markdown
## Bug Description
Brief description of the problem

## Environment
- Arduino board: [Uno/Nano/Mega/etc.]
- IDE version: [Arduino IDE 2.x.x]
- Hardware configuration: [thruster types, sensors, etc.]

## Steps to Reproduce
1. Step one
2. Step two
3. Step three

## Expected Behavior
What should happen

## Actual Behavior  
What actually happens

## Serial Output
```
Paste any relevant serial monitor output here
```

## Additional Context
- Photos of hardware setup
- Oscilloscope traces if available
- Any modifications to the code
```

## Feature Requests

### Before Requesting
- Consider if it fits the project scope
- Think about backward compatibility

### Feature Request Template
```markdown
## Feature Description
Clear description of the proposed feature

## Use Case
Why is this feature needed? What problem does it solve?

## Proposed Implementation
How do you think this should work?

## Alternatives Considered
Other ways to achieve the same goal

## Additional Context
- Hardware requirements
- Compatibility concerns
- Examples from other projects
```

## Code Contributions

### Types of Contributions We Need

**High Priority:**
- Sensor implementation examples (depth, IMU, position tracking)
- PID auto-tuning algorithms
- Safety improvements and fail-safes
- Performance optimizations
- Documentation improvements

**Medium Priority:**
- Additional control modes (manual override, waypoint following)
- Data logging capabilities
- Communication protocols (WiFi, radio)
- GUI development tools

**Nice to Have:**
- Simulation tools
- Configuration management
- Advanced mixing algorithms
- Multi-vehicle coordination

### Pull Request Process

1. **Create focused PRs**
   - One feature or fix per pull request
   - Keep changes small and reviewable
   - Include tests when applicable

2. **Write good commit messages**
   ```bash
   # Good commit messages
   feat: add depth sensor calibration function
   fix: correct yaw angle normalization bug
   docs: update PID tuning guide with examples
   
   # Bad commit messages  
   update code
   fix stuff
   changes
   ```

3. **Fill out PR template**
   ```markdown
   ## Changes Made
   - Brief list of changes
   
   ## Testing Performed
   - How you tested the changes
   
   ## Hardware Tested
   - What hardware configuration you used
   
   ## Breaking Changes
   - Any changes that might break existing setups
   
   ## Screenshots/Videos
   - If applicable, show the feature working
   ```

4. **Respond to feedback**
   - Address review comments promptly
   - Ask questions if feedback is unclear
   - Update code based on suggestions

### Code Review Criteria

**Functionality:**
- Code compiles without warnings
- Features work as described
- No regression in existing functionality
- Safety features remain intact

**Code Quality:**
- Follows project coding style
- Includes appropriate comments
- Uses descriptive variable names
- No hardcoded magic numbers

**Documentation:**
- Updates README if needed
- Includes usage examples
- Updates command help text
- Documents any new dependencies

## Testing Guidelines

### Manual Testing Checklist
- [ ] Code compiles successfully
- [ ] Serial communication works at 9600 baud
- [ ] Help command shows updated information
- [ ] Emergency stop functions properly
- [ ] PID controllers respond to parameter changes
- [ ] No memory leaks or stack overflow

### Hardware Testing (if available)
- [ ] Thrusters respond correctly to power commands
- [ ] Emergency stop immediately stops all thrusters
- [ ] Enable/disable commands work properly
- [ ] PID control maintains stability
- [ ] No unexpected behavior or oscillations

### Testing Documentation
Include in your PR:
```markdown
## Testing Performed

### Compilation Testing
- Arduino Uno - no warnings
- Arduino Nano - compiled successfully

### Functionality Testing  
- New feature works as expected
- Existing commands still functional
- Emergency stop tested and working

### Hardware Testing
- Tested with [hardware description]
- No unexpected behavior observed
- Could not test underwater due to [reason]
```

## Communication

### Getting Help
- **Email**: [abdrahman24sc@gmail.com]

### Discussion Etiquette
- **Be respectful** and constructive
- **Search first** before asking questions
- **Provide context** when asking for help
- **Share solutions** when you find them
- **Help others** when you can

## Documentation Contributions

### Types of Documentation Needed
- **Setup guides** for specific hardware combinations
- **Troubleshooting guides** for common issues
- **Video tutorials** for complex procedures
- **Wiring diagrams** and schematics
- **Performance benchmarks** and test results

### Documentation Standards
- Use clear, concise language
- Include step-by-step instructions
- Add screenshots or diagrams when helpful
- Test instructions with fresh setup
- Keep formatting consistent

## Recognition

### Contributors
All contributors will be:
- Added to the project's contributors list
- Mentioned in release notes for significant contributions
- Given credit in documentation they help create

### Hall of Fame
Outstanding contributors may be featured in:
- Project README
- Special recognition in releases
- Invitation to be project maintainer

## Code of Conduct

### Our Standards
- **Be inclusive** and welcoming to all experience levels
- **Be patient** with newcomers to Arduino/robotics
- **Be constructive** in feedback and criticism
- **Be collaborative** and open to different approaches
- **Be safety-conscious** when dealing with hardware

### Enforcement
- Violations should be reported to project maintainers
- We reserve the right to remove comments or ban users
- Focus is on maintaining a positive, educational environment

## Questions?
- **Contribution questions**: Open an issue with the "question" label
- **Private concerns**: Contact maintainers directly

Thank you for helping make underwater robotics more accessible!

---

*This contributing guide is itself open to contributions - suggest improvements via pull request!*