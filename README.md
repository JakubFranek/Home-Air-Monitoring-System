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

## System Components

- **Central Hub**
   - **ESP32-based**; mains-powered.
   - Connects to the internet for weather updates and time synchronization via SNTP.
   - Displays real-time measurements on a 7.5" 800x480 e-ink screen with a 1 minute refresh rate.
   - Contains sensors for all previously listed quantities.
   - Screen-sized 3D-printed enclosure (estimated 175 mm x 92.65 mm x 120 mm).

- **Sensor Nodes**
   - **STM32-based** and battery-powered, optimized for low power consumption and long battery life.
   - Equipped with temperature and humidity sensors only, with optional hardware support for one more sensor.
   - Transmit data to the hub every minute using nRF24 radio modules.
   - Minimalistic and visually non-intrusive 3D-printed case with a small form factor (estimated 81 mm x 58 mm x 28 mm).

## Sensors

- **SHT40**
   - Temperature and humidity sensor from Sensirion.
   - Used in the central hub and sensor nodes.
   - Temperature accuracy: ±0.2 °C
   - Humidity accuracy: ±1.8 %RH
- **SCD41**
   - CO2 sensor from Sensirion.
   - Used in the central hub.
   - CO2 accuracy: ±50 ppm ± 5.0 %m.v. (depending on range)
- **SGP41**
   - VOC and NOx gas index sensor from Sensirion.
   - Used in the central hub.
- **SPS30**
   - Particulate matter sensor from Sensirion.
   - Used in the central hub.
   - Accuracy: ±10 %
- **BME280**
   - Temperature, relative humidity and atmospheric pressure sensor from Adafruit.
   - Used in the central hub (for atmospheric pressure only).
   - Atmospheric pressure accuracy: ±1 hPa

## Repository Structure

```plaintext
├── docs                         # Documentation for project setup, usage, etc.
├── utilities                    # Various auxiliary scripts, tools etc.
│   ├── sht_sensor_burn_in       # PlatformIO Arduino project for SHT40 sensor burn-in
│   └── sht_sensor_calibration   # PlatformIO Arduino project for SHT40 sensor calibration
├── hub
│   ├── hardware                 # PCB design and schematic for hub
│   ├── firmware                 # Firmware for the ESP32-based central hub
│   └── 3D_models                # 3D models for the hub's enclosure
├── node
│   ├── firmware                 # Firmware for the STM32-based sensor nodes
│   ├── hardware                 # PCB designs and schematics for nodes
│   └── 3D_models                # 3D models for the nodes' enclosures
├── README.md                    # Project overview (you are reading this now!)
└── TODO.md                      # Project task list