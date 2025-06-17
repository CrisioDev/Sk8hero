# Sk8hero - Skateboard Xbox Controller

An Arduino-based skateboard controller built on the **Tony Hawk Ride** controller platform, originally designed for **SoulsLike games** but adaptable for skateboard games and 2D sidescrollers. This project involves a complete hardware modification of the Tony Hawk Ride controller with custom sensors and Arduino integration.

## 🎮 Original Purpose: SoulsLike Games

This controller was **originally designed for SoulsLike games** like Dark Souls, Elden Ring, or Bloodborne, where:
- **Tilt movements** control character movement and camera
- **Weight shifting** triggers dodge rolls and attacks
- **Sensor blocking** activates various combat actions
- **Physical skateboard movements** translate to intense gaming sessions

The dual-mode system also supports traditional skateboard games and 2D sidescrollers for versatility.

## 🛹 Hardware Base: Tony Hawk Ride + Guitar Hero Controller Hybrid

### **Important: This is a complex dual-controller modification!**

This project combines **two gaming controllers** into one skateboard controller:

#### Required Hardware:
- **Tony Hawk Ride Controller** (skateboard platform)
- **Guitar Hero Controller** (button interface)
- **2x Patch Cables** (connecting both controllers)
- **~40 screws removal** from Tony Hawk Ride (various sizes)
- **Griptape removal** (heat gun or hair dryer recommended)
- **Complete internal modification** of both controllers

#### Hybrid Modification Process:
1. **Tony Hawk Ride Prep**:
   - Remove Griptape (carefully peel off using heat)
   - Unscrew Everything (approximately 40 screws of different sizes)
   - Gut Internal Electronics (remove all original sensors and PCBs)
   - Install Arduino and VL53L0X/MPU6050 sensors

2. **Guitar Hero Controller Prep**:
   - Disassemble controller housing
   - Identify button matrix and connections
   - Prepare for patch cable integration

3. **Hybrid Integration**:
   - Connect both controllers via **2x Patch Cables**
   - Route Guitar Hero buttons through Arduino
   - Integrate skateboard sensors with button interface
   - Create unified control system

4. **Final Assembly**:
   - Reassemble Tony Hawk Ride with new electronics
   - Secure patch cable connections
   - Test all button and sensor functions

⚠️ **Warning**: This modification will **permanently destroy** both the original Tony Hawk Ride and Guitar Hero controller functionality!

## 🔧 New Hardware Components

### Sensors (Replacing Original Hardware)
- **4x VL53L0X**: Time-of-flight distance sensors (Left, Right, Front, Back)
- **2x MPU6050**: Accelerometer/gyroscope sensors for precise tilt detection
- **Arduino-compatible board** (replaces original controller PCB)

### Buttons & Controls (Guitar Hero Integration)
- **6 digital buttons** (D8-D13) - **mapped from Guitar Hero controller**
- **Mode switch** (A2) - **Guitar Hero strum bar or whammy**
- **Analog slider** (A3) - **Guitar Hero whammy bar**
- **Additional buttons** (A0, A1) - **Guitar Hero face buttons**

## 🛠️ Modification Guide

### Tools Required:
- Phillips head screwdrivers (multiple sizes)
- Heat gun or hair dryer
- **2x Patch Cables** (appropriate length for controller separation)
- Soldering iron and solder
- Wire strippers
- Drill (for new mounting holes)
- Hot glue gun
- Multimeter
- **Cable management supplies** (zip ties, cable guides)

### Step-by-Step:
1. **Documentation**: Photo everything before disassembly (both controllers)
2. **Tony Hawk Ride Disassembly**: 
   - Heat and carefully peel off griptape
   - Remove ~40 screws (organize by location)
   - Disconnect and remove all original components
3. **Guitar Hero Controller Prep**:
   - Disassemble controller housing
   - Map button matrix and connections
   - Prepare interface points for patch cables
4. **Arduino Integration**:
   - Install Arduino in Tony Hawk Ride center cavity
   - Mount VL53L0X sensors at board edges
   - Install MPU6050 sensors for tilt detection
5. **Patch Cable Installation**:
   - Run 2x patch cables from Tony Hawk Ride to Guitar Hero controller
   - Connect Arduino pins to Guitar Hero button matrix
   - Ensure reliable connections with strain relief
6. **System Integration**:
   - Program Arduino to read both sensor data and Guitar Hero inputs
   - Test all connections before final assembly
7. **Final Assembly**:
   - Reassemble Tony Hawk Ride with new electronics
   - Secure all cable connections and routing
   - Test complete system functionality

## 📌 Pin Mapping (Custom Installation)

### Distance Sensors (Edge Mounted)
- **D4**: Left edge sensor (XSHUT)
- **D5**: Back edge sensor (XSHUT) + D-Pad Down trigger
- **D6**: Right edge sensor (XSHUT)
- **D7**: Front edge sensor (XSHUT) + Button Y trigger

### Buttons (Guitar Hero Controller Interface)
#### Normal Mode (SoulsLike/3D Games)
- **D8**: SELECT (BACK) - *Guitar Hero Back button*
- **D9**: Button B (Light Attack) - *Guitar Hero Green*
- **D10**: Button START (Menu) - *Guitar Hero Start button*
- **D11**: Button RB (Heavy Attack) - *Guitar Hero Red*
- **D12**: Button X (Use Item) - *Guitar Hero Yellow*
- **D13**: Button LB (Block/Parry) - *Guitar Hero Blue*
- **A0**: Button R3 (Lock-on) - *Guitar Hero Orange*
- **A1**: Button LT (Magic/Spell) - *Guitar Hero Select*
- **A2**: Mode Switch (Camera control toggle) - *Guitar Hero Strum Bar*
- **A3**: Analog RT (Sprint/Run) - *Guitar Hero Whammy Bar*

#### Alternative Mode (2D Sidescroller Games)
- **A0**: Button B (Jump) - *Guitar Hero Orange*
- **A1**: Button RT (Special Attack) - *Guitar Hero Select*
- **A2**: D-PAD UP (Menu/Up) - *Guitar Hero Strum Up*
- **A3**: Button LB (Dash) - *Guitar Hero Whammy Bar*
- **D9**: D-PAD Right (Move Right) - *Guitar Hero Green*
- **D11**: D-PAD Left (Move Left) - *Guitar Hero Red*
- **D13**: Button RB (Attack) - *Guitar Hero Blue*
- **Back Sensor**: D-PAD Down (Crouch) - *Tony Hawk Ride Back Sensor*

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
- **Patch cable reliability**: Use high-quality cables with strain relief
- **Controller interference**: Ensure proper grounding between both controllers
- **Cable management**: Prevent cables from interfering with skateboard movement
- **Sensor mounting**: Use strong adhesive and backing plates on Tony Hawk Ride
- **Guitar Hero integration**: Ensure stable button matrix connections

### Software Issues
- **Sensor conflicts**: Check I2C address conflicts
- **Input lag**: Optimize loop timing for responsive gaming
- **Mode confusion**: Clear visual/tactile feedback for mode switches

## 📊 Technical Details

### Modified Hardware Specs
- **Primary Platform**: Tony Hawk Ride Controller (sensors and movement)
- **Interface Platform**: Guitar Hero Controller (buttons and controls)
- **Connection**: 2x Patch Cables
- **New Sensors**: 4x VL53L0X + 2x MPU6050 (Tony Hawk Ride)
- **Processing**: Arduino-compatible microcontroller
- **Button Interface**: Guitar Hero button matrix
- **Power**: USB or battery pack (depending on Arduino choice)
- **Total Weight**: Combined weight of both modified controllers

### Performance
- **Response Time**: <50ms for all inputs
- **Sensor Range**: 30-1200mm (VL53L0X dependent)
- **Tilt Precision**: ±0.1° (MPU6050 dependent)
- **Update Rate**: ~50Hz

## 💡 Why Tony Hawk Ride + Guitar Hero?

**Tony Hawk Ride Controller** provides:
- **Perfect skateboard platform**: Real skateboard size and feel
- **Robust construction**: Built to handle weight and movement
- **Sensor mounting space**: Room for Arduino and custom sensors
- **Natural skateboard interaction**: Authentic tilt and weight shifting

**Guitar Hero Controller** provides:
- **Familiar button layout**: Established gaming interface
- **Reliable button matrix**: Proven button detection system
- **Ergonomic design**: Comfortable button placement
- **Cost effective**: Used controllers readily available

**Combined Benefits**:
- **Best of both worlds**: Physical skateboard movement + precise button control
- **Modular design**: Easy to replace or upgrade either component
- **Proven hardware**: Both controllers have established reliability
- **Enhanced functionality**: More control options than either controller alone

## 🤝 Contributing

This is a complex hardware modification project. Contributions welcome for:
- **Improved sensor mounting techniques**
- **Alternative Arduino board recommendations**
- **Enhanced calibration procedures**
- **Additional game compatibility**

## 📄 License

This project is licensed under the MIT License. See LICENSE file for details.

**Note**: This modification voids any warranty on both the Tony Hawk Ride and Guitar Hero controllers.

## 🙏 Acknowledgments

- **Tony Hawk Ride** - Skateboard platform base
- **Guitar Hero Controller** - Button interface base
- **VL53L0X library** by Pololu
- **MPU6050 library** by Electronic Cats  
- **XInput library** for Arduino
- **SoulsLike community** for inspiration and testing

## 📞 Contact

For questions about the modification process or technical issues, please create an issue in this repository.

---

**⚠️ Disclaimer**: This modification permanently alters both the Tony Hawk Ride and Guitar Hero controllers. Attempt at your own risk. The author is not responsible for damaged hardware or personal injury. Test in a safe environment! 🛹🎸⚔️
