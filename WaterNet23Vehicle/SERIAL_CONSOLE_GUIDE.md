# WaterNet23 Serial Console Interface

## Overview
The WaterNet23 vehicle now supports command input directly from the Serial console (COM port) in addition to XBee, BLE, and LTE communications. This provides a convenient way to interact with the vehicle during development, testing, and debugging.

## Getting Started

### Connecting to the Console
1. Connect the WaterNet23 vehicle to your computer via USB
2. Open a serial terminal (Arduino IDE Serial Monitor, PuTTY, etc.)
3. Configure: **115200 baud, 8N1**
4. After boot, you'll see the startup message:

```
=== WaterNet23 Bot 1 Console Ready ===
Type 'help' or 'hlp' for command list
Commands can be entered as: <cmd> <data>
Example: sim1, cms, ekf1, hlp
=====================================
```

### Command Formats
The console supports two command formats:

#### 1. Simple Format (Recommended)
Just type the command and data directly:
```
sim1
cms
ekf1
hlp
mtr090090
```

#### 2. Raw Format (Advanced)
Use the full protocol format:
```
B1CCsim1F
B1CCcms0E
B1CCekf1F
```

## Available Commands

### Basic System Commands
- **`help`** or **`hlp`** - Show command reference
- **`?`** - Alternative help command
- **`stp`** - Emergency stop (stops all motors immediately)

### Navigation & Control
- **`ctl <lat> <lon> <mode> <log> <led>`** - Set target position and drive mode
  - Example: `ctl 42.3601 -83.0732 1 1 0`
- **`mtr <lspeed><rspeed>`** - Control motors (3 digits each, 090 = stop)
  - Example: `mtr090090` (stop both), `mtr100080` (slow forward)

### Sensor & GPS
- **`req`** - Request current sensor data
- **`egp <lat> <lon>`** - Set emulated GPS position for testing
  - Example: `egp 42.3601 -83.0732`

### Compass System
- **`cms`** - Show compass manager status
- **`cms0`** - Switch to LSM303 compass
- **`cms1`** - Switch to LIS3MDL compass
- **`cms2`** - Auto-detect compass type
- **`cmp`** - Perform compass calibration

### Simulation System
- **`sim`** - Show simulation status
- **`sim0`** - Disable simulation (use real sensors)
- **`sim1`** - Enable static position simulation
- **`sim2`** - Enable waypoint navigation simulation
- **`sim3`** - Enable circular motion simulation
- **`sim4`** - Enable random walk simulation

### Extended Kalman Filter (EKF)
- **`ekf`** - Show EKF status
- **`ekf0`** - Disable EKF filtering
- **`ekf1`** - Enable EKF filtering
- **`ekf1 0.01 0.1`** - Enable EKF with custom noise parameters

### Debug & Utility
- **`pts <message>`** - Print message to console and log
  - Example: `pts Hello World`
- **`spc`** - Send status/ping
- **`dmp`** - Enter data dump mode for SD card offloading

## Interactive Examples

### Testing Simulation System
```
> sim
Simulation Status:
  Enabled: No
  Mode: 0

> sim1
> Simulation enabled: Static mode

> sim
Simulation Status:
  Enabled: Yes
  Mode: 1
  Current GPS: Lat=42.360100, Lon=-83.073200, Course=45.0
  Current Compass: Heading=45.0
```

### Compass Management
```
> cms
Compass Manager Status:
  Manager initialized: Yes
  Active compass: LIS3MDL
  Manager connected: Yes
  Current heading: 87.45

> cms0
> Switched to compass type: LSM303
```

### Motor Control
```
> mtr090090
> Emergency stop activated

> mtr100080
> Received Motor Command: LSpeed=100,RSpeed=80
```

### Getting Help
```
> help
=== WaterNet23 Vehicle Commands ===
[Full command reference displayed]

> ?
[Same help output]
```

## Features

### Command Echo
All commands are echoed back with a `>` prompt:
```
> sim1
> Simulation enabled: Static mode
```

### Error Handling
Invalid commands show helpful error messages:
```
> invalidcmd
Warning: Unknown command 'invalidcmd'
```

### Automatic Formatting
Simple commands are automatically formatted with proper bot addressing and checksums:
- Input: `sim1`
- Formatted: `B1CCsim1F` (for bot 1)

### Logging
All console commands are logged to the SD card for debugging and audit purposes.

## Integration with Existing Systems

### Works Alongside Other Communication Methods
- XBee (Serial1) - for wireless mesh communication
- BLE - for mobile app connectivity
- LTE - for cloud connectivity
- Serial Console - for direct development access

### Same Command Processing
Console commands use the same `processCommand()` function as other interfaces, ensuring consistent behavior across all communication methods.

### Mode Handling
Console commands are processed with mode `1` (similar to BLE), which means they have the same priority and handling as Bluetooth commands.

## Debugging Tips

### Enable Verbose Mode
Uncomment `#define VERBOSE` in the code to see detailed command processing:
```cpp
#define VERBOSE  // Add this line
```

### Monitor Command Flow
With verbose mode enabled, you'll see:
```
New Serial Console Command:
sim1
Formatted command: B1CCsim1F
Command: sim, Data: 1, Checksum: 0F
Simulation enabled: Static mode
```

### Check Logs
All console interactions are logged to the SD card file for later analysis.

## Troubleshooting

### No Response to Commands
1. Check baud rate (must be 115200)
2. Ensure line ending is set to newline (`\n`)
3. Verify bot number matches in commands

### Commands Not Recognized
1. Use `help` to see available commands
2. Check command spelling
3. Try raw format if simple format fails

### Simulation Not Working
1. Check simulation status with `sim`
2. Verify mode with status command
3. Use verbose output to debug

This Serial console interface significantly improves the development and testing experience by providing direct, real-time access to all vehicle systems without requiring wireless communication setup.
