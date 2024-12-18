/*
 * This file is based on source code originally from martinberlin/CalEPD GitHub repository,
 * available at https://github.com/martinberlin/CalEPD.
 *
 * Modifications have been made to the original code by Jakub Franek (https://github.com/JakubFranek),
 * as permitted under the Apache License, Version 2.0.
 */

// GooDisplay product https://www.good-display.com/product/389.html
// Controller:        SSD1680

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include <stdint.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include <epd.h>
#include <Adafruit_GFX.h>
#include <epdspi.h>
#include <gdew_4grays.h>
#include <esp_timer.h>

#define GDEY029T94_X_PIXELS 128
#define GDEY029T94_Y_PIXELS 296
#define GDEY029T94_WIDTH 128
#define GDEY029T94_HEIGHT 296
#define GDEY029T94_VISIBLE_WIDTH 128
#define GDEY029T94_BUFFER_SIZE (uint32_t(GDEY029T94_WIDTH) * uint32_t(GDEY029T94_HEIGHT) / 8)

class Gdey029T94 : public Epd
{
public:
  Gdey029T94(EpdSpi &epd_spi);

  void drawPixel(int16_t x, int16_t y, uint16_t color); // Mandatory override of GFX own drawPixel method

  void initialize(void);
  void fillScreen(uint16_t color);
  void update();

  void updateWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool using_rotation = true);

private:
  static constexpr const char *TAG = "GDEY029T94";
  EpdSpi &epd_spi;
  uint8_t _buffer[GDEY029T94_BUFFER_SIZE];

  void _wakeUp();
  void _sleep();

  void wait_while_busy_(const char *message);
  void _rotate(uint16_t &x, uint16_t &y, uint16_t &w, uint16_t &h);
  void _setRamDataEntryMode(uint8_t em);
  void _SetRamArea(uint8_t Xstart, uint8_t Xend, uint8_t Ystart, uint8_t Ystart1, uint8_t Yend, uint8_t Yend1);
  void _SetRamPointer(uint8_t addrX, uint8_t addrY, uint8_t addrY1);
};
