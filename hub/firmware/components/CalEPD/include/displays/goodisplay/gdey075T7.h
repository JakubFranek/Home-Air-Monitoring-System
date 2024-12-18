/*
 * This file is based on source code originally from martinberlin/CalEPD GitHub repository,
 * available at https://github.com/martinberlin/CalEPD.
 * Modifications have been made to the original code by Jakub Franek (https://github.com/JakubFranek),
 * as permitted under the Apache License, Version 2.0.
 *
 * This file includes code adapted from GxEPD2 by Jean-Marc Zingg.
 * Original source: https://github.com/ZinggJM/GxEPD2
 * Licensed under the GNU General Public License v3.0 (GPLv3).
 * See the LICENSE file or https://www.gnu.org/licenses/gpl-3.0.en.html for details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "epd.h"
#include "epd_colors.h"
#include "Adafruit_GFX.h"
#include "epdspi.h"
#include "esp_timer.h"

#define GDEY075T7_WIDTH 800
#define GDEY075T7_HEIGHT 480
#define GDEY075T7_BUFFER_SIZE (uint32_t(GDEY075T7_WIDTH) * uint32_t(GDEY075T7_HEIGHT) / 8)

class Gdey075T7 : public Epd
{
public:
  Gdey075T7(EpdSpi &epd_spi);

  void drawPixel(int16_t x, int16_t y, uint16_t color); // Override Adafruit_GFX drawPixel method
  void fillScreen(uint16_t color);                      // Override Adafruit_GFX fillScreen method

  void initialize();
  void wake_up();
  void update();
  void clear_screen();

  uint8_t _buffer[GDEY075T7_BUFFER_SIZE];

private:
  EpdSpi &epd_spi;

  void transfer_buffer_(uint8_t command);
  void transfer_single_color_(uint8_t command, uint16_t color);
  void wait_while_busy_(const char *message);
  void rotate_(uint16_t &x, uint16_t &y, uint16_t &w, uint16_t &h);
  void send_refresh_command_();
  void power_on_();
  void power_off_();
  void sleep_();
};