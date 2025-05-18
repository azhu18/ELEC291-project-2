# ELEC291-project-2
# Coin Picking Robot

A robotic system designed and built as part of ELEC 291/292 at the University of British Columbia to automatically and manually detect, pick up, and track coins using embedded microcontrollers.

# Features

* **Two Modes of Operation**: Manual and automatic coin picking
* **Real-time Wireless Communication**: JDY-40 module for robot-controller communication; HC-05 for computer feedback
* **Multi-sensor Integration**: Metal detector, perimeter detector, ultrasonic obstacle detector
* **Servo Arm + Electromagnet**: For precise coin pickup
* **Bonus Features**: Obstacle avoidance, battery voltage display, PC path tracking using Python

## 🛠 Hardware Overview

### Robot (Slave - EFM8)

* **Microcontroller**: EFM8 (8051 family)
* **Motor Driver**: H-Bridge using P/N-MOSFETs and optocouplers
* **Metal Detection**: Inductor + capacitor frequency shift
* **Perimeter Detection**: LC tank + peak detector (LM358)
* **Distance Sensor**: HC-SR04
* **Actuators**: Servo motor, electromagnet
* **Bluetooth Modules**: JDY-40 (UART2), HC-05 (data to PC)

### Remote Controller (Master - STM32L051)

* **LCD and Buzzer**
* **Joystick**: PS2 joystick to ADC pins
* **Control Buttons**: Mode switching and manual start

## 💻 Software Overview

* **Language**: C (embedded), Python (PC path plotting)
* **Control Logic**: Timer5 ISR manages PWM for motor/servo
* **Communication Protocol**: UART-based JDY-40 and HC-05
* **Mode Management**:

  * Manual: Joystick controls direction/speed
  * Auto: Robot detects coins, updates LCD
  * Obstacle Avoidance: Distance sensor + HC-05 logs
  * Battery Monitor: Displays voltage level

## Repository Structure

```
/robot/
  main.c                  # Robot firmware (EFM8)
  peripherals/            # Motor, servo, ADC modules
/controller/
  main.c                  # Remote firmware (STM32)
  lcd.c, adc.c, uart2.c   # Display, input, communication
/pc/
  diagram.py           # Python script to plot car path
README.md
```

## Setup Instructions

1. **Flash Firmware**: Use Silicon Labs IDE for EFM8, STM32CubeIDE for STM32
2. **Power Supply**: 9V battery + LM7805 to 5V; 4xAA for motors
3. **UART Connection**:

   * JDY-40: Robot → Remote
   * HC-05: Robot → PC
4. **Python Dependencies (optional)**:

```bash
pip install matplotlib pyserial
python pc/track_plot.py
```

## Performance Summary

* Coin pickup success: 19/20 trials
* Trajectory deviation: <10%
* Servo control synchronized via timer interrupts

## Lessons Learned

* Importance of modular hardware/software design
* Debugging multi-threaded ISR and UART systems
* Applied ELEC 201 knowledge (MOSFETs, analog filtering)

## Team & Course Info

* **Course**: ELEC 291/292, UBC
* **Instructor**: Dr. Jesus Calvino-Fraga
* **Team**: Bangyan Jiang, Xuanyu Bao, Pengrui Zhao, Emma Zhu, Harris Liu, Yi Hou


## ⚠️ Academic Integrity Notice

This project is part of a graded university assignment. Any direct copying, reuse, or submission of this work (in whole or in part) without proper citation constitutes academic dishonesty and may violate institutional policies on plagiarism and academic integrity.

---

Made with solder, sleep deprivation, and teamwork at UBC 😊
