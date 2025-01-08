# TODO

This is a task list for HAMS project.

## Work in progress

- [ ] SHT40 sensor burn-in and calibration    
    - [x] [Create a burn-in program](https://github.com/JakubFranek/Home-Air-Monitoring-System/tree/master/utilities/sht_sensor_burn_in)
    - [ ] Burn-in SHT40 sensors to remove VOC contaminants
    - [x] [Create a calibration program](https://github.com/JakubFranek/Home-Air-Monitoring-System/tree/master/utilities/sht_sensor_calibration)
    - [ ] Calibrate SHT40 sensor humidity measurements using NaCl and MgCl salt solutions
  
- [ ] Node hardware
    - [ ] Node schematic
        - [x] QFP32 breakout board test (test programmability with ST-LINK V3 MINIE)
        - [x] decide whether to use LDO or not (decided to not use LDO, will be created as a separate board if needed)
        - [x] decide whether to support USB power supply as well (decided not to use USB, will be created as a separate board if needed)
        - [x] choose batteries & protection (CR2032 & LiFePO4)
        - [x] bring out SPI, I2C & SWD nodes for programming & debug
        - [x] bring out unused GPIO just in case
        - [x] implement supply current measurement jumpers
        - [x] draw schematic in KiCAD
        - [ ] double check all connections
    - [ ] Node PCB layout
    - [ ] Node 3D enclosure design
        - [ ] interior enclosure
            - [ ] design
            - [ ] test print
            - [ ] final print (5x)
        - [ ] exterior enclosure
            - [ ] design
            - [ ] print (1x)
    - [ ] Node assembly
        - [ ] 5x interior nodes
        - [ ] 1x exterior node
    - [ ] Flash nodes
    - [ ] Test nodes
    - [ ] Cover exterior nodes in protective plastic coating

## To do

- [ ] Hub hardware
    - [ ] Decide which connector to use to provide 5 Volt power to ESPink, SCD41, SPS30 and fan (test current capability)
    - [ ] Create hub connector board (perfboard?)
    - [ ] Consider red LED error indicator for errors which prevent use of e-ink display
    - [ ] Hub 3D enclosure
        - [ ] determine layout of ESPink + sensors + fan + power connector board
        - [ ] determine debug display button location
        - [ ] design
        - [ ] print
    - [ ] Assemble hub
  
- [ ] Create documentation
    - [ ] Document node software incl. flowchart
    - [ ] Document hub software incl. FreeRTOS tasks and flowchart
    - [ ] Take photos of assembled nodes and hub incl. display interface

## Backlog

- [ ] Hub firmware: Upload data from hub to Google Sheets or elsewhere online
- [ ] Hub firmware: Replace Adafruit_GFX with C-based graphical library to remove C++ altogether
- [ ] Node firmware: RX/TX node to increase maximum possible amount of nodes above 6
- [ ] Node firmware: Support for SCD41 (possibly with MCU-driven ASC to minimize consumption)

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