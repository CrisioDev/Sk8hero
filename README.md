# Sk8hero - Skateboard Xbox Controller

An Arduino-based skateboard controller that converts real skateboard movements into Xbox controller inputs. Perfect for skateboard games like Skate, Tony Hawk's Pro Skater, or Session, with a special alternative mode for 2D sidescroller games.

## 🛹 Features

- **Real Skateboard Control**: Tilt your skateboard left/right for joystick inputs
- **Step Detection**: Forward/backward movement through weight shifting
- **Trick Recognition**: Nose-up movements trigger the A button
- **Dual Mode System**: Switchable between 3D skateboard games and 2D sidescroller games
- **Sensor-based Buttons**: Obstacles in front of sensors trigger various buttons
- **Anti-blocking Protection**: Automatic movement stop when sensors are blocked for too long

## 🔧 Hardware Components

### Sensors
- **4x VL53L0X**: Time-of-flight distance sensors (Left, Right, Front, Back)
- **2x MPU6050**: Accelerometer/gyroscope sensors for tilt detection
- **Arduino-compatible board** with sufficient digital and analog pins

### Buttons & Controls
- **6 digital buttons** (D8-D13)
- **Mode switch** (A2)
- **Analog slider** (A3)
- **Additional buttons** (A0, A1)

## 📌 Pin Mapping

### Sensors
- **D4**: Left distance sensor (XSHUT)
- **D5**: Back distance sensor (XSHUT) + D-Pad Down
- **D6**: Right distance sensor (XSHUT)
- **D7**: Front distance sensor (XSHUT) + Button Y

### Buttons (Normal Mode - 3D Skateboard Games)
- **D8**: SELECT (BACK)
- **D9**: Button B
- **D10**: Button START
- **D11**: Button RB
- **D12**: Button X
- **D13**: Button LB
- **A0**: Button R3 (Right stick press)
- **A1**: Button LT (Left trigger)
- **A2**: Mode Switch (Toggle between left/right stick for roll)
- **A3**: Analog RT (Right trigger)

### Alternative Mode (2D Sidescroller Games)
The alternative mode can be activated by pressing the START button 5 times within 10 seconds and is optimized for 2D sidescroller games:

- **A0**: Button B (instead of R3)
- **A1**: Button RT (instead of LT)
- **A2**: D-PAD UP (instead of Mode Switch)
- **A3**: Button LB (instead of RT Slider)
- **D9**: D-PAD Right (instead of Button B)
- **D11**: D-PAD Left (instead of Button RB)
- **D13**: Button RB (instead of Button LB)
- **Back Sensor**: D-PAD Down
- **Roll movement**: Always mapped to left stick (no mode switching)
- **Step detection**: Disabled for better 2D control

## 🎮 Controls

### Basic Movements
- **Tilt Left/Right**: Joystick input (left stick by default in normal mode)
- **Hold Mode Switch**: Switches roll input to right stick (normal mode only)
- **Weight Shifting**: Step detection for forward/backward movement (normal mode only)
- **Nose-Up**: Triggers Button A (for tricks in both modes)

### Sensor Activation
- **Front Sensor**: Button Y
- **Back Sensor**: D-Pad Down
- **Side Sensors**: Step detection and anti-blocking protection (normal mode)

### Safety Features
- **Block Detection**: Automatically stops movement when sensors are blocked for >500ms
- **Hysteresis**: Prevents flickering at sensor transitions
- **Deadzone**: Small movements are ignored

## 🎯 Game Compatibility

### Normal Mode (3D Skateboard Games)
- **Skate Series** (Skate, Skate 2, Skate 3)
- **Tony Hawk's Pro Skater Series**
- **Session**
- **Skater XL**

### Alternative Mode (2D Sidescroller Games)
- **OlliOlli Series**
- **Skateboard Party**
- **Epic Skater**
- **Any 2D platformer with skateboarding elements**

## 🔧 Installation

### Required Libraries
```cpp
#include <Wire.h>       // I2C communication
#include <VL53L0X.h>    // Time-of-flight sensors
#include <MPU6050.h>    // Accelerometer/gyroscope
#include <XInput.h>     // Xbox controller emulation
#include <math.h>       // Mathematical functions
```

### Setup
1. Install all libraries via Arduino Library Manager
2. Wire hardware according to pin mapping
3. Upload code to Arduino board
4. Skateboard controller is ready to use!

## 🎯 Configuration

### Important Constants
```cpp
const uint16_t NEAR_TH = 200;        // Distance threshold in mm
const uint16_t LASER_HYST = 20;      // Hysteresis in mm
const float MAX_ANGLE = 15.0f;       // Maximum tilt angle in degrees
const float ROLL_DEAD = 5.0f;        // Deadzone for roll movement
const float PITCH_THRESHOLD = 10.0f; // Threshold for nose-up detection
const uint32_t BLOCK_TIME = 500;     // Blocking time in ms
```

### Customizations
- **Sensitivity**: Adjust `MAX_ANGLE` and `ROLL_DEAD`
- **Step Size**: Modify `STEP_INC` for forward/backward movement
- **Sensor Range**: Change `NEAR_TH` for distance threshold

## 🔄 Mode Switching

### How to Switch Modes
1. **Normal → Alternative**: Press START button 5 times within 10 seconds
2. **Alternative → Normal**: Press START button 5 times within 10 seconds
3. **Visual Feedback**: 
   - Alternative mode ON: 3 short trigger pulses
   - Alternative mode OFF: 1 long trigger pulse

### Mode Differences

| Feature | Normal Mode (3D) | Alternative Mode (2D) |
|---------|------------------|----------------------|
| Step Detection | ✅ Active | ❌ Disabled |
| Roll Control | Left/Right stick switchable | Always left stick |
| D-Pad Control | Back sensor only | Full D-Pad control |
| Optimized for | 3D skateboard games | 2D sidescroller games |

## 🔧 Troubleshooting

### Common Issues
- **Sensor Initialization**: Check I2C connections and addresses
- **Flickering Inputs**: Adjust hysteresis values
- **No Controller Recognition**: Reinstall XInput library

### Debug Mode
Enable serial output for debugging:
```cpp
Serial.begin(9600);
Serial.println("Sensor values: " + String(distance));
```

## 📊 Technical Details

### Sensor Configuration
- **VL53L0X**: Continuous measurement every 20ms
- **MPU6050**: Dual-sensor setup for precise tilt measurement
- **I2C Addresses**: 0x30-0x33 for VL53L0X, 0x68-0x69 for MPU6050

### Performance
- **Loop Frequency**: ~50Hz (20ms delay)
- **Sensor Response Time**: <50ms
- **Battery Life**: Depends on board used

## 🤝 Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Create a pull request

## 📄 License

This project is licensed under the MIT License. See LICENSE file for details.

## 🙏 Acknowledgments

- VL53L0X library by Pololu
- MPU6050 library by Electronic Cats
- XInput library for Arduino
- Skateboarding community for inspiration

🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹🛹
