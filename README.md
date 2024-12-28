# Home Air Monitoring System

**Home Air Monitoring System (HAMS)** is my personal hardware & embedded software development project designed to monitor indoor and outdoor air quality in my home.

## Overview

The system consists of a **central hub** and **six wireless sensor nodes** to measure:

- Temperature
- Relative Humidity
- Atmospheric Pressure
- Particulate Matter
- Volatile Organic Compounds (VOCs)
- NOₓ
- CO₂

### Components

- **Central Hub**
   - **ESP32-based**; mains-powered.
   - Connects to the internet for weather updates.
   - Displays real-time data on a large e-ink screen.
   - Contains sensors for all previously listed quantities.

- **Sensor Nodes**
   - **STM32-based** and battery-powered, optimized for low power consumption.
   - Equipped with temperature and humidity sensors only.
   - Transmit data to the hub every minute using nRF24 radio modules.
   - Outdoor nodes housed in weather-resistant, 3D-printed enclosures.

## Repository Structure

```plaintext
├── docs                         # Documentation for project setup, usage, etc.
├── utilities                    # Various auxiliary scripts, tools etc.
│   └── sht_sensor_calibration   # PlatformIO Arduino-based project for calibrating SHT40 sensors
├── hub
│   ├── firmware                 # Firmware for the ESP32-based central hub
│   └── 3D_models                # 3D models for the hub's enclosure
├── node
│   ├── firmware                 # Firmware for the STM32-based sensor nodes
│   ├── hardware                 # PCB designs and schematics for nodes
│   └── 3D_models                # 3D models for the nodes' enclosures
├── README.md                    # Project overview (you are reading this now!)
└── TODO.md                      # Project task list