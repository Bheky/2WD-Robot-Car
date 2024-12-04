# 2WD Robot Car Project

## 📜 Overview
The 2WD Robot Car project is an Arduino-based robotic platform designed for exploring autonomous navigation and control techniques. The robot features motor control, sensor integration (ultrasonic and IR), and wall-following capabilities. This project leverages Mbed OS for efficient task management and Arduino Nano 33 BLE for processing and communication.

## 🚀 Features
- **Motor Control:** Precise control using PWM and direction pins via TB6612FNG motor drivers.
- **Obstacle Detection:** Uses ultrasonic (HC-SR04) and IR (Sharp GP2Y0E02B) sensors for real-time distance measurements.
- **Wall Following:** Implements proportional-derivative (PD) control for navigating along walls.
- **Autonomous Navigation:** Responds dynamically to obstacles with real-time debugging via serial monitor.

---

## 🛠️ Hardware Requirements
- **Microcontroller:** Arduino Nano 33 BLE
- **Sensors:**
  - 2x HC-SR04 Ultrasonic Sensors (front and back)
  - 4x Sharp GP2Y0E02B IR Sensors (side-mounted)
- **Motor Driver:** TB6612FNG
- **Motors:** 2x DC Motors with encoders (e.g., Pololu motor with 100:1 gear ratio)
- **Power Supply:** 7.4V LiPo battery or equivalent
- **Custom PCB:** ARB (Arduino Robotics Board)

---

## 🔧 Software Setup
### **Dependencies**
- [Arduino IDE](https://www.arduino.cc/en/software) (1.8+ recommended)
- [Mbed OS](https://os.mbed.com/)
- Libraries:
  - `mbed::I2C` for sensor communication
  - `Arduino` for compatibility functions

### **Steps to Run**
1. Clone the repository:
   ```bash
   git clone git@github.com:Bheky/2WD-Robot-Car.git
   cd 2WD-Robot-Car
2. Install required libraries (e.g., via Arduino Library Manager or manual download).
3. Connect your Arduino Nano 33 BLE to the computer and upload the code using the Arduino IDE.
4. Use the serial monitor to debug and verify sensor readings.

## 📁 Directory Structure

2WD-Robot-Car/
├── 2WDRobotInterface/        # Robot interface setup
├── ARB/                      # ARB board schematics and details
├── InterfaceMotorsTest/      # Motor test code
├── InterfaceSensorsTest/     # Sensor test code
├── LeftWallFollowingInterface/ # Wall following logic (left side)
├── MotorsPIDControl/         # Motor control with PD logic
├── WallFollowingInterface-v1/ # Integrated navigation code
└── README.md                 # Project documentation

## 🚧 Notes
  * Sensor Calibration: Ensure the ultrasonic and IR sensors are calibrated before running the robot in a real-world environment.
  * Maze Dimensions: Designed for walls spaced between 29.5 cm and 42.0 cm.
  * Power: Avoid running the motors without a sufficient power supply as it may cause brownouts.

## 🖋️ Contributions
Contributions are welcome! Please fork the repository, make your changes, and submit a pull request. Follow the repository structure for adding new features or fixes.

##  📄 License
This project is licensed under the MIT License. See the LICENSE file for more details.

## 💬 Contact
For questions or feedback, please create an issue in the repository or email telasi.inc@outook.com.

## 🛑 Disclaimer
This project is for educational purposes only. Ensure proper safety measures are in place when testing the robot.

---

### **Add a `.gitignore` File**
To ignore unnecessary files like libraries, create a `.gitignore` file with the following content:

## Ignore Arduino library folders
libraries/ *.cpp.o *.ino.elf *.hex *.bin

## Ignore VS Code settings (if applicable)
.vscode/

## Ignore build artifacts
build/

---

### Steps to Add README and `.gitignore`
1. Create the files locally:
   ```bash
   echo "[Insert README content]" > README.md
   echo "[Insert .gitignore content]" > .gitignore
   
2. Stage and commit the files:

  ```bash
  git add README.md .gitignore
  git commit -m "Added README and .gitignore files"
```

3. Push the changes to GitHub:

```bash
git push
