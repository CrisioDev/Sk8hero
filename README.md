# Sk8hero - Skateboard Xbox Controller

An Arduino-based skateboard controller built on the **Tony Hawk Ride** controller platform, originally designed for **SoulsLike games** but adaptable for skateboard games and 2D sidescrollers. This project involves a complete hardware modification of the Tony Hawk Ride controller with custom sensors and Arduino integration.

## 🎮 Original Purpose: SoulsLike Games

This controller was **originally designed for SoulsLike games** like Dark Souls, Elden Ring, or Bloodborne, where:
- **Tilt movements** control character movement and camera
- **Weight shifting** triggers dodge rolls and attacks
- **Sensor blocking** activates various combat actions
- **Physical skateboard movements** translate to intense gaming sessions

The dual-mode system also supports traditional skateboard games and 2D sidescrollers for versatility.

## 🛹 Hardware Base: Tony Hawk Ride Controller Modification

### **Important: This is a destructive modification!**

This project requires **completely disassembling** a Tony Hawk Ride controller:

#### Required Hardware:
- **Tony Hawk Ride Controller** (original skateboard controller)
- **~40 screws removal** (various sizes)
- **Griptape removal** (heat gun or hair dryer recommended)
- **Complete internal gutting** of original electronics

#### Modification Process:
1. **Remove Griptape**: Carefully peel off the griptape using heat
2. **Unscrew Everything**: Remove approximately 40 screws of different sizes
3. **Gut Internal Electronics**: Remove all original sensors and PCBs
4. **Install New Hardware**: Mount Arduino and custom sensors
5. **Rewire Completely**: Install new wiring harness
6. **Reassemble**: Put everything back together with new functionality

⚠️ **Warning**: This modification will **permanently destroy** the original Tony Hawk Ride controller functionality!

## 🔧 New Hardware Components

### Sensors (Replacing Original Hardware)
- **4x VL53L0X**: Time-of-flight distance sensors (Left, Right, Front, Back)
- **2x MPU6050**: Accelerometer/gyroscope sensors for precise tilt detection
- **Arduino-compatible board** (replaces original controller PCB)

### Buttons & Controls (Custom Installation)
- **6 digital buttons** (D8-D13) - mounted in original button locations
- **Mode switch** (A2) - replaces original mode button
- **Analog slider** (A3) - custom installation
- **Additional buttons** (A0, A1) - hidden trigger buttons

## 🛠️ Modification Guide

### Tools Required:
- Phillips head screwdrivers (multiple sizes)
- Heat gun or hair dryer
- Soldering iron and solder
- Wire strippers
- Drill (for new mounting holes)
- Hot glue gun
- Multimeter

### Step-by-Step:
1. **Documentation**: Photo everything before disassembly
2. **Griptape Removal**: Heat and carefully peel off
3. **Screw Inventory**: Keep screws organized by location
4. **Electronics Removal**: Disconnect and remove all original components
5. **Sensor Mounting**: Install VL53L0X sensors at board edges
6. **Arduino Installation**: Mount Arduino in center cavity
7. **Wiring Harness**: Create custom wiring following pin mapping
8. **Testing**: Test all connections before reassembly
9. **Reassembly**: Reverse disassembly process

## 📌 Pin Mapping (Custom Installation)

### Distance Sensors (Edge Mounted)
- **D4**: Left edge sensor (XSHUT)
- **D5**: Back edge sensor (XSHUT) + D-Pad Down trigger
- **D6**: Right edge sensor (XSHUT)
- **D7**: Front edge sensor (XSHUT) + Button Y trigger

### Buttons (Original Button Locations)
#### Normal Mode (SoulsLike/3D Games)
- **D8**: SELECT (BACK)
- **D9**: Button B (Light Attack)
- **D10**: Button START (Menu)
- **D11**: Button RB (Heavy Attack)
- **D12**: Button X (Use Item)
- **D13**: Button LB (Block/Parry)
- **A0**: Button R3 (Lock-on)
- **A1**: Button LT (Magic/Spell)
- **A2**: Mode Switch (Camera control toggle)
- **A3**: Analog RT (Sprint/Run)

#### Alternative Mode (2D Sidescroller Games)
- **A0**: Button B (Jump)
- **A1**: Button RT (Special Attack)
- **A2**: D-PAD UP (Menu/Up)
- **A3**: Button LB (Dash)
- **D9**: D-PAD Right (Move Right)
- **D11**: D-PAD Left (Move Left)
- **D13**: Button RB (Attack)
- **Back Sensor**: D-PAD Down (Crouch)

## 🎮 Controls

### SoulsLike Game Controls (Primary Purpose)
- **Tilt Left/Right**: Character movement/strafe
- **Weight Forward/Back**: Walk forward/backward
- **Sharp Tilt**: Quick dodge rolls
- **Nose-Up**: Jump/heavy attack
- **Sensor Blocking**: Various combat actions
- **Mode Switch**: Toggle between movement and camera control

### Skateboard Game Controls
- **Tilt movements**: Joystick control for tricks
- **Weight shifting**: Forward/backward movement
- **Sensor activation**: Trick triggers

### Safety Features
- **Block Detection**: Prevents infinite inputs when sensors stuck
- **Hysteresis**: Smooth sensor transitions
- **Deadzone**: Eliminates noise from small movements

## 🎯 Game Compatibility

### Primary Target (SoulsLike Games)
- **Dark Souls Series**
- **Elden Ring**
- **Bloodborne**
- **Sekiro: Shadows Die Twice**
- **Hollow Knight**
- **Salt and Sanctuary**

### Secondary Support
#### Normal Mode (3D Skateboard Games)
- **Skate Series**
- **Tony Hawk's Pro Skater Series**
- **Session**
- **Skater XL**

#### Alternative Mode (2D Sidescroller Games)
- **OlliOlli Series**
- **Skateboard Party**
- **Epic Skater**

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
1. **Complete Tony Hawk Ride modification** (see modification guide)
2. Install all libraries via Arduino Library Manager
3. Upload code to Arduino board
4. Calibrate sensors in open space
5. Ready for intense SoulsLike gaming!

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

### SoulsLike Tuning
- **Dodge Sensitivity**: Adjust `MAX_ANGLE` for dodge roll triggers
- **Movement Deadzone**: Tune `ROLL_DEAD` for precise movement
- **Attack Timing**: Modify sensor thresholds for combat timing

## 🔄 Mode Switching

### How to Switch Modes
- Press START button 5 times within 10 seconds
- **Visual Feedback**: 
  - Alternative mode ON: 3 short trigger pulses
  - Alternative mode OFF: 1 long trigger pulse

## ⚠️ Safety & Warnings

### Physical Safety
- **Sturdy mounting required**: Ensure all sensors are securely mounted
- **Weight limits**: Don't exceed original Tony Hawk Ride weight rating
- **Surface testing**: Test on appropriate surfaces only

### Electrical Safety
- **Proper insulation**: Ensure all wiring is properly insulated
- **Power management**: Use appropriate power supply for Arduino
- **Short circuit protection**: Include fuses where appropriate

## 🔧 Troubleshooting

### Common Modification Issues
- **Sensor mounting**: Use strong adhesive and backing plates
- **Wire management**: Ensure wires won't pinch during skateboard flex
- **Calibration**: Recalibrate after any physical changes

### Software Issues
- **Sensor conflicts**: Check I2C address conflicts
- **Input lag**: Optimize loop timing for responsive gaming
- **Mode confusion**: Clear visual/tactile feedback for mode switches

## 📊 Technical Details

### Modified Hardware Specs
- **Original Platform**: Tony Hawk Ride Controller
- **New Sensors**: 4x VL53L0X + 2x MPU6050
- **Processing**: Arduino-compatible microcontroller
- **Power**: USB or battery pack (depending on Arduino choice)
- **Weight**: Similar to original (depends on Arduino choice)

### Performance
- **Response Time**: <50ms for all inputs
- **Sensor Range**: 30-1200mm (VL53L0X dependent)
- **Tilt Precision**: ±0.1° (MPU6050 dependent)
- **Update Rate**: ~50Hz

## 💡 Why Tony Hawk Ride?

The Tony Hawk Ride controller provides:
- **Perfect form factor**: Real skateboard size and feel
- **Robust construction**: Built to handle weight and movement
- **Existing mounting points**: Easier sensor installation
- **Familiar interface**: Natural skateboard interaction
- **Cost effective**: Used controllers available at reasonable prices

## 🤝 Contributing

This is a complex hardware modification project. Contributions welcome for:
- **Improved sensor mounting techniques**
- **Alternative Arduino board recommendations**
- **Enhanced calibration procedures**
- **Additional game compatibility**

## 📄 License

This project is licensed under the MIT License. See LICENSE file for details.

**Note**: This modification voids any warranty on the Tony Hawk Ride controller.

## 🙏 Acknowledgments

- **Tony Hawk Ride** - Original hardware platform
- **VL53L0X library** by Pololu
- **MPU6050 library** by Electronic Cats  
- **XInput library** for Arduino
- **SoulsLike community** for inspiration and testing

## 📞 Contact

For questions about the modification process or technical issues, please create an issue in this repository.

---

**⚠️ Disclaimer**: This modification permanently alters the Tony Hawk Ride controller. Attempt at your own risk. The author is not responsible for damaged hardware or personal injury. Test in a safe environment! 🛹⚔️
