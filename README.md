# Arduino-Multi-Function-Robot-Car
Arduino robot car with three control modes: obstacle avoidance, Bluetooth remote control, and voice commands. Features ultrasonic sensor, servo motor, and HC-05 Bluetooth module.

# **Arduino Multi-Function Robot Car**  
*Obstacle Avoiding | Bluetooth Control | Voice Control Robot Car*

A comprehensive Arduino-based robot car featuring three distinct control modes for versatile operation. Perfect for robotics beginners and hobbyists looking to explore autonomous navigation, wireless control, and voice command integration.

## 🚀 **Features**

### **Three Control Modes**
- **🛡️ Obstacle Avoidance** - Autonomous navigation using ultrasonic sensor scanning
- **📱 Bluetooth Control** - Manual remote control via smartphone app
- **🎤 Voice Control** - Voice command operation with distance safety checks

### **Technical Specifications**
- **Controller**: Arduino UNO
- **Motor Driver**: L293D Motor Shield
- **Wireless**: HC-05 Bluetooth Module
- **Sensors**: HC-SR04 Ultrasonic + SG90 Servo for environmental scanning
- **Power**: Dual 18650 Li-ion battery system
- **Chassis**: Customizable foam board/cardboard design

## 📋 **Hardware Requirements**

| Component | Quantity | Purpose |
|-----------|----------|---------|
| Arduino UNO | 1 | Main microcontroller |
| L293D Motor Driver Shield | 1 | 4-motor control |
| HC-SR04 Ultrasonic Sensor | 1 | Distance measurement |
| HC-05 Bluetooth Module | 1 | Wireless communication |
| SG90 Servo Motor | 1 | Sensor positioning |
| DC Gear Motor (100-300 RPM) | 4 | Wheel propulsion |
| Robot Wheels | 4 | Mobility |
| 18650 Battery Holder | 1 | Power management |
| 18650 Li-ion Batteries | 2 | Power source |
| Jumper Wires | 1 pack | Electrical connections |
| Foam Board/Cardboard | 1 sheet | Chassis material |

## 🔌 **Circuit Connections**

### **Pin Configuration**

| Component | Arduino Pin | Connection |
|-----------|------------|------------|
| Ultrasonic Trig | A1 | Digital Output |
| Ultrasonic Echo | A0 | Digital Input |
| Servo Signal | 10 | PWM Output |
| Bluetooth TX | RX (Pin 0) | Serial Receive |
| Bluetooth RX | TX (Pin 1) | Serial Transmit |
| Bluetooth VCC | 5V | Power |
| Bluetooth GND | GND | Ground |

### **Motor Connections (L293D Shield)**
- **M1**: Front Left Motor
- **M2**: Front Right Motor  
- **M3**: Rear Left Motor
- **M4**: Rear Right Motor

## 💻 **Software Setup**

### **1. Install Required Software**
- **Arduino IDE**: Download from [arduino.cc](https://www.arduino.cc/en/software)
- **Required Libraries**: 
cpp
  #include <Servo.h>      // Built-in Arduino library
  #include <AFMotor.h>    // Download from tutorial


### **2. Mobile Applications**
- **Bluetooth Control**: "Arduino RC Bluetooth Car" (Play Store)
- **Voice Control**: "Arduino Bluetooth Control" (Play Store)

## 📝 **Code Configuration**

### **Main Program Setup**
In `robot_car.ino`, select ONE mode by uncommenting in `loop()`:
arduino
void loop() {
  //Obstacle();        // Mode 1: Autonomous
  //Bluetoothcontrol(); // Mode 2: Bluetooth
  //voicecontrol();    // Mode 3: Voice
}


### **Mode-Specific Instructions**

#### **Mode 1: Obstacle Avoidance**
1. Uncomment `Obstacle()` in `loop()`
2. Disconnect Bluetooth TX/RX from Arduino
3. Upload code
4. Reconnect Bluetooth
5. Power on for autonomous navigation

#### **Mode 2: Bluetooth Control**
1. Uncomment `Bluetoothcontrol()` in `loop()`
2. Disconnect Bluetooth TX/RX
3. Upload code
4. Reconnect Bluetooth
5. Pair smartphone (PIN: 1234)
6. Open app → Connect → Control with buttons

#### **Mode 3: Voice Control**
1. Uncomment `voicecontrol()` in `loop()`
2. Disconnect Bluetooth TX/RX
3. Upload code
4. Reconnect Bluetooth
5. Configure app voice commands:
 
   Forward: ^
   Backward: -
   Left: <
   Right: >
   Stop: *

6. Speak commands to control

## 🛠️ **Assembly Guide**

### **Step 1: Chassis Preparation**
1. Cut foam board to 15×20cm
2. Mark motor positions at corners
3. Attach gear motors securely

### **Step 2: Electronics Mounting**
1. Center Arduino with motor shield
2. Mount servo at front center
3. Attach ultrasonic sensor to servo
4. Position Bluetooth module

### **Step 3: Wiring**
1. Connect motors to L293D ports
2. Wire ultrasonic to A0/A1
3. Connect servo to pin 10
4. Connect Bluetooth module
5. Connect battery holder

### **Step 4: Final Assembly**
1. Attach wheels
2. Insert batteries
3. Secure all wires
4. Power on and test

## ⚠️ **Troubleshooting**

| Problem | Solution |
|---------|----------|
| Motors not moving | Check battery and connections |
| Bluetooth not connecting | Verify power and TX/RX wiring |
| Code won't upload | Disconnect Bluetooth during upload |
| Ultrasonic errors | Check sensor connections |
| Servo issues | Confirm pin 10 connection |
| Wrong direction | Swap motor connections on one side |

## 🔧 **Customization**

### **Adjustable Parameters**
arduino
#define Speed 170      // Motor speed (0-255)
#define spoint 103     // Servo center position
if (distance <= 12)    // Obstacle detection threshold
if (L >= 10)           // Minimum turning space


### **Feature Additions**
- LED lighting system
- Buzzer for audio feedback
- Line following sensors
- OLED display for status
- Camera module integration


## 📚 **Learning Resources**
- [Arduino Official Documentation](https://www.arduino.cc/reference/en/)
- [L293D Motor Driver Datasheet](https://www.ti.com/lit/ds/symlink/l293d.pdf)
- [HC-SR04 Ultrasonic Tutorial](https://howtomechatronics.com)
- [HC-05 Bluetooth Guide](https://lastminuteengineers.com)

## 👥 **Contributing**
1. Fork the repository
2. Create feature branch
3. Commit changes
4. Push to branch
5. Open Pull Request

## 📄 **License**
MIT License - See [LICENSE](LICENSE) for details.

## 🙏 **Acknowledgments**
- Original tutorial by [SriTu Hobby](https://srituhobby.com)
- Arduino community support
- Project contributors and testers

---
