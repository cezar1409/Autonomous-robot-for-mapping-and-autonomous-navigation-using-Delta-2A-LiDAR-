# Autonomous-robot-for-mapping-and-autonomous-navigation-using-Delta-2A-LiDAR-


An autonomous mobile robot system featuring LiDAR-based environmental mapping and real-time navigation capabilities.


This project implements a complete autonomous mobile robot system capable of real-time environmental mapping and navigation using LiDAR technology. The robot integrates a Delta-2A LiDAR sensor with triangulation-based distance measurement for 360° environmental scanning across 16 sectors.

 Hardware Components

- **LiDAR Sensor**: Delta-2A with 8-meter range, 360° coverage
- **Main Controller**: Raspberry Pi 3B+ (Debian Linux)
- **Motor Controller**: Arduino Mega 2560 with Moebius shield
- **Locomotion**: Mecanum wheel drive system for omnidirectional movement
- **Communication**: USB 3.0 and UART protocols

 Key Features

- **360° Environmental Scanning**: Real-time LiDAR mapping with 16-sector coverage
- **Autonomous Navigation**: Obstacle avoidance and path planning algorithms
- **Omnidirectional Movement**: Mecanum wheels enable lateral translation and rotation
- **Real-time Processing**: Sub-second response time for navigation decisions
- **Precise Detection**: 5cm minimum obstacle detection accuracy



## 🔧 Installation & Setup

### Prerequisites
- Python 3.8+
- Arduino IDE
- Raspberry Pi with Debian Linux
- Required Python packages (see requirements.txt)

### Hardware Setup
1. Connect Delta-2A LiDAR to Raspberry Pi via USB
2. Wire Arduino Mega 2560 to Raspberry Pi via UART
3. Mount mecanum wheels and connect to Moebius shield
4. Power system and verify connections

LidarDelta2A.py
Handles communication with the Delta-2A LiDAR sensor over serial (230400 baud). Parses proprietary frames divided into 16 sectors (22.5° each), each with up to 52 points (angle, distance, intensity). Enables continuous real-time data streaming.

rover.ino (Arduino Code)
Controls a mecanum-wheel robot via Moebius motor shield. Defines PWM and direction pins for four motors and supports forward, lateral, and rotational motion. Receives single-character movement commands over UART (9600 baud) from a Raspberry Pi. Includes safety stops and speed control.

test_v3.py (Main Navigation)
Implements autonomous navigation using LiDAR data. Uses virtual repulsion for obstacle avoidance, detects corners, and makes smart turns. Prioritizes lateral over backward moves. Includes live matplotlib visualization and separates command/control timing for efficiency.

test1.py (Basic Navigation)
A simple test script for the LiDAR. Establishes serial communication, parses basic distance/power data, and checks sensor functionality. Used for calibration and debugging before running full navigation.

Algorithms Implemented:

- Triangulation-based Distance Measurement: For precise LiDAR ranging
- Obstacle Avoidance: Force-based repulsion from obstacles
- Path Planning: Real-time safe direction calculation
- Corner Detection: 90° and 180° turn strategies
- Motor Control: PWM-based omnidirectional movement


This project was developed as a Bachelor's thesis at University POLITEHNICA of Bucharest, Faculty of Electronics, Telecommunications and Information Technology, Applied Electronics Program.



**Author**: Cezar-Andrei Diaconu  
**University**: POLITEHNICA Bucharest  
**Program**: Applied Electronics

