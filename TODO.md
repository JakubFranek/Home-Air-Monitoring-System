# TODO

This is a task list for HAMS project.

## Work in progress

- [ ] Hub firmware v0.1
    - [ ] Test firmware components in separate projects
        - [x] Create and test SGP41 driver
        - [x] Create and test SPS30 driver
        - [x] Test nRF24L01+ driver, incl. IRQ handling
        - [x] Test SHT40 driver
        - [ ] Create and test SCD41 driver (driver created, waiting for sensor)
        - [x] Create and test BME280 driver
        - [x] Test 5 Volt fan control via GPIO
        - [x] Test WiFi connection
        - [x] Test RTC time synchronization using SNTP
        - [x] Combine WiFi connection and SNTP time sync into one code
        - [ ] Create e-ink display driver
            - [x] Determine best graphical library to use (Adafruit_GFX)
            - [x] Determine best driver to use (custom GxEPD2 & CalEPD hybrid)
            - [ ] Create and test GDEY075T7 driver (waiting for display to arrive)    
        - [ ] Get and parse current name day using HTTPS API
        - [ ] Get and parse weather forecast using HTTPS API (likely from openweathermap)
    - [ ] Create display interface layout
    - [ ] Assemble all firmware components together into one codebase
    - [ ] Implement debug display view based on GPIO button toggle

## To do


- [ ] Node hardware
    - [ ] Node schematic
        - [ ] QFP32 breakout board test
            - [ ] test programming with ST-LINK V3 MINIE
            - [ ] test RTC quartz
        - [ ] decide whether to use LDO or not
        - [ ] decide whether to support USB power supply as well
        - [ ] choose batteries & protection (possibly LiSOCl2)
        - [ ] bring out SPI, I2C & SWD nodes for programming & debug
        - [ ] bring out unused GPIO just in case
        - [ ] implement supply current measurement jumpers
        - [ ] draw schematic in KiCAD
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
- [ ] Burn-in SHT40 sensors to remove VOC contaminants
- [ ] Calibrate SHT40 sensor humidity measurements using NaCl and MgCl salt solutions
- [ ] Hub hardware
    - [ ] Decide which connector to use to provide 5 Volt power to ESPink, SPS30 and fan
    - [ ] Create hub connector board (perfboard?)
    - [ ] Hub 3D enclosure
        - [ ] determine layout of ESPink + sensors + fan + power connector board
        - [ ] determine debug display button location
        - [ ] design
        - [ ] print
- [ ] Documentation
    - [ ] Document node software incl. flowchart
    - [ ] Document hub software incl. FreeRTOS tasks and flowchart
    - [ ] Take photos of assembled nodes and hub incl. display interface

## Backlog

- [ ] Hub firmware: Upload data from hub to Google Sheets or elsewhere online

## Done

- [x] Node firmware v0.1
    - [x] Test firmware components in separate projects
        - [x] Create and test nRF24L01+ driver
        - [x] Create and test SHT40 driver, incl. utilizing dedicated HW for CRC verification
        - [x] Test stop mode (incl. current consumption measurement)
        - [x] Test ADC usage for supply voltage measurement
        - [x] Test scheduled wake-up from stop mode using RTC
    - [x] Assemble all firmware components into one codebase
