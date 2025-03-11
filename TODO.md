# Task list <!-- omit from toc -->

This is a task list for the HAMS project.

## Contents <!-- omit from toc -->
- [Work in progress](#work-in-progress)
- [To do](#to-do)
- [Backlog](#backlog)
- [Done](#done)


## Work in progress

- [ ] SHT40 sensor burn-in and calibration    
    - [x] [Create a burn-in program](https://github.com/JakubFranek/Home-Air-Monitoring-System/tree/master/utilities/sht_sensor_burn_in)
    - [x] Burn-in SHT40 sensors to remove VOC contaminants
    - [x] [Create a calibration program](https://github.com/JakubFranek/Home-Air-Monitoring-System/tree/master/utilities/sht_sensor_calibration)
    - [ ] Calibrate SHT40 sensor humidity measurements using NaCl and MgCl salt solutions
  
- [ ] Node hardware
    - [x] Node schematic
        - [x] QFP32 breakout board test (test programmability with ST-LINK V3 MINIE)
        - [x] decide whether to use LDO or not (decided to not use LDO, will be created as a separate board if needed)
        - [x] decide whether to support USB power supply as well (decided not to use USB, will be created as a separate board if needed)
        - [x] choose batteries & protection (CR2032 & LiFePO4)
        - [x] bring out SPI, I2C & SWD nodes for programming & debug
        - [x] bring out unused GPIO just in case
        - [x] implement supply current measurement jumpers
        - [x] draw schematic in KiCAD
        - [x] double check all connections
    - [x] Node PCB layout
    - [x] Order PCBs
    - [x] Order components
    - [x] Node 3D enclosure design
        - [x] design
        - [x] test print
        - [x] final print (6x)
    - [x] Node PCB assembly
    - [x] Fix battery holder crimping
    - [ ] Flash nodes
    - [ ] Test nodes
    - [ ] Cover exterior nodes in protective plastic coating

- [ ] Create documentation
    - [x] Document node software incl. flowchart
    - [x] Document hub software incl. FreeRTOS tasks and flowchart
    - [ ] Photos
        - [x] Assembled node
        - [ ] Assembled hub
        - [ ] Hub display
        - [ ] Hub debug display
        - [ ] New calibration setup with batteries
    - [ ] Add HAMS node current measurements
    - [ ] Add enclosure dimension to hardware docs
  

- [ ] Hub firmware
    - [ ] Turn SPS30 off after measurements to limit fan noise
    - [x] Remove T 24h min and PM size from the display
    - [x] Remove "Sever" node, add "Detsky pokoj" node

## To do
  
- [ ] Test SCD41 CO2 measurement in open air (expected cca 400 ppm)
  
- [ ] Hub firmware: SCD41 force ASC via long debug button press

## Backlog

- [ ] Hub firmware: Upload data from hub to Google Sheets or elsewhere online
- [ ] Hub firmware: Replace Adafruit_GFX with C-based graphical library to remove C++ altogether
- [ ] Node firmware: Support for SCD41 (possibly with MCU-driven ASC to minimize consumption)
- [ ] Node firmware: Better error handling (nRF24 transmission of error codes even if FSM steps fail before nRF24 transmit step)
- [ ] Node firmware: Change radio data rate to 250 kbps to increase range
- [ ] Hub firmware: Software calibration of SHT4x measurements
- [ ] Hub firmware: Add pressure correction to SCD41 measurements

## Done

- [x] Node firmware v0.1
    - [x] Test firmware components in separate projects
        - [x] [Create and test nRF24L01+ driver](https://github.com/JakubFranek/STM32/tree/master/NUCLEO-L031K6/nRF24L01p_TX_RX_L031K6_LL)
        - [x] [Create and test SHT40 driver, incl. utilizing dedicated HW for CRC verification](https://github.com/JakubFranek/STM32/tree/master/NUCLEO-L031K6/SHT40_L031K6_LL) 
        - [x] [Test stop mode (incl. current consumption measurement)](https://github.com/JakubFranek/STM32/tree/master/NUCLEO-L031K6/Stop_mode_L031K6_LL)
        - [x] [Test ADC usage for supply voltage measurement](https://github.com/JakubFranek/STM32/tree/master/NUCLEO-L031K6/ADC_LL_L031K6)
        - [x] [Test scheduled wake-up from low power mode using RTC](https://github.com/JakubFranek/STM32/tree/master/NUCLEO-L031K6/RTC_wakeup_L031K6_LL)
    - [x] [Assemble all firmware components into one codebase](https://github.com/JakubFranek/Home-Air-Monitoring-System/tree/master/node/firmware/STM32L031K6)

- [x] Hub firmware v0.1
    - [x] Test firmware components in separate projects
        - [x] [Create and test SGP41 driver](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/I2C_SGP41)
        - [x] [Create and test SPS30 driver](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/I2C_SPS30)
        - [x] [Test nRF24L01+ driver (RX mode), incl. IRQ handling](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/nRF24L01p_RX)
        - [x] [Test SHT40 driver](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/I2C_SHT40)
        - [x] [Create and test SCD41 driver](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/I2C_SCD41)
        - [x] [Create and test BME280 driver](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/I2C_BME280)
        - [x] [Test 5 Volt fan control via GPIO](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/Fan_Switch)
        - [x] [Test WiFi connection](https://github.com/JakubFranek/ESP32/tree/master/Examples/station)
        - [x] [Test RTC time synchronization using SNTP](https://github.com/JakubFranek/ESP32/tree/master/Examples/sntp)
        - [x] [Combine WiFi connection and SNTP time sync into one code](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/WIFI_SNTP)
        - [x] Create e-ink display driver
            - [x] Determine best graphical library to use (Adafruit_GFX)
            - [x] Determine best driver to use ([customized CalEPD](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/Epaper_GDEY029T94_CalEPD_Demo))
            - [x] [Create and test GDEY075T7 driver](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/Epaper_GDEY075T7_CalEPD_Demo)
                - [x] Add support for full partial display refresh
                - [x] Test drawing a bitmap image
        - [x] [Get and parse current name day using HTTPS API](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/API_svatkyapicz)
        - [x] [Get and parse openweathermap.org weather forecast using HTTPS API](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/API_openweathermap)
    - [x] Design display interface
        - [x] Create display interface layout ([implemented in GDEY075T7 test project](https://github.com/JakubFranek/ESP32/tree/master/ESP-ink/Epaper_GDEY075T7_CalEPD_Demo))
        - [x] [Prepare a set of weather icon byte arrays for openweatherapi weather codes](https://github.com/JakubFranek/ESP32/blob/master/ESP-ink/Epaper_GDEY075T7_CalEPD_Demo/main/weather_icons.h)
        - [x] [Prepare a set of small morning / day / eve / night icons for temperatures](https://github.com/JakubFranek/ESP32/blob/master/ESP-ink/Epaper_GDEY075T7_CalEPD_Demo/main/time_of_day_icons.h)
    - [x] [Assemble all firmware components together into one codebase](https://github.com/JakubFranek/Home-Air-Monitoring-System/tree/master/hub/firmware)
        - [x] Implement e-ink display driver
        - [x] Implement WiFi connectivity
        - [x] Implement SNTP
        - [x] Implement svatkyapi.cz requests
        - [x] Implement openweathermap.org requests
        - [x] Implement SHT40 driver
        - [x] Implement SCD41 driver
        - [x] Implement SGP41 driver
        - [x] Implement SPS30 driver
        - [x] Implement BME280 driver
        - [x] Implement fan control
        - [x] Implement nRF24L01+ driver
        - [x] Display sensor data
        - [x] Display weather data
        - [x] Display startup sequence debug info
        - [x] Implement node timestamp checking
        - [x] Implement 24h minimum temperature logic for particular node
        - [x] Implement logic for long holiday names
        - [x] Implement debug display view based on GPIO button toggle
        - [x] Clean up code
        - [x] Show sensor status codes in debug mode
  
- [x] Node firmware
    - [x] Handle fault where nRF24 does not activate IRQ after transmitting
    - [x] Update SHT4x drivers (reuse better hub versions)
    - [x] Update nRF24 drivers (reuse better hub versions)
    - [x] Add comments / function documentation to app.c and important code elsewhere
    - [x] Add define for automatic retransmit delay based on NODE_ID
    - [x] Send UART debug logs only when in debug mode

- [x] Hub hardware
    - [x] Decide which connector to use to provide 5 Volt power to ESPink, SCD41, SPS30 and fan (ESPink hacked to provide 5 Volt power out of VIN pin)
    - [x] Hub sensor board schematic
        - [x] Add SPI & I2C debug headers
    - [x] Order PCBs
    - [x] Order components
    - [x] Hub 3D enclosure
        - [x] determine layout of ESPink + sensors + fan + power connector board
        - [x] determine debug display button location
        - [x] design
        - [x] print
    - [x] Assemble hub PCB
    - [x] Test hub PCB
    - [x] Assemble hub enclosure