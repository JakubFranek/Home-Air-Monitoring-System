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

#include "displays/goodisplay/gdey029T94.h"

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/task.h"
#include "esp_log.h"

#define GDEY029T94_SPI_FREQUENCY_MHZ 4
#define GDEY029T94_HW_RESET_DELAY_MS 20
#define GDEY029T94_BUSY_TIMEOUT_US 4000000 // 4 s
#define GDEY029T94_UPDATE_SEQUENCE 0xF7
#define GDEY029T94_UPDATE_SEQUENCE_FAST 0xC7
#define GDEY029T94_UPDATE_SEQUENCE_PARTIAL 0xFF
#define GDEY029T94_BORDER_WAVEFORM 0x05 // Follow LUT 1

#define SSD1680_CMD_SET_DRIVER_OUTPUT_CONTROL 0x01
#define SSD1680_CMD_SET_GATE_DRIVING_VOLTAGE 0x03
#define SSD1680_CMD_SET_SOURCE_DRIVING_VOLTAGE 0x04
#define SSD1680_CMD_SET_DEEP_SLEEP_MODE 0x10
#define SSD1680_CMD_SET_DATA_ENTRY_MODE 0x11
#define SSD1680_CMD_SW_RESET 0x12
#define SSD1680_CMD_UPDATE 0x20
#define SSD1680_CMD_SET_UPDATE_RAM 0x21
#define SSD1680_CMD_SET_UPDATE_SEQUENCE 0x22
#define SSD1680_CMD_WRITE_RAM_BW 0x24
#define SSD1680_CMD_WRITE_RAM_RED 0x26
#define SSD1680_CMD_WRITE_VCOM_REGISTER 0x2C
#define SSD1680_CMD_SET_BORDER_WAVEFORM 0x3C
#define SSD1680_CMD_SET_LUT_END_OPTION 0x3F
#define SSD1680_CMD_SELECT_TEMPERATURE_SENSOR 0x18
#define SSD1680_CMD_SET_RAM_X_START_END_ADDRESS 0x44
#define SSD1680_CMD_SET_RAM_Y_START_END_ADDRESS 0x45
#define SSD1680_CMD_SET_RAM_X_COUNTER 0x4E
#define SSD1680_CMD_SET_RAM_Y_COUNTER 0x4F

Gdey029T94::Gdey029T94(EpdSpi &epd_spi) : Adafruit_GFX(GDEY029T94_WIDTH, GDEY029T94_HEIGHT),
                                          Epd(GDEY029T94_WIDTH, GDEY029T94_HEIGHT), epd_spi(epd_spi) {};

void Gdey029T94::initialize(void)
{
  epd_spi.initialize(GDEY029T94_SPI_FREQUENCY_MHZ);

  epd_spi.hardware_reset(GDEY029T94_HW_RESET_DELAY_MS);
  fillScreen(EPD_WHITE);
}

void Gdey029T94::fillScreen(uint16_t color)
{
  uint8_t data = (color == EPD_WHITE) ? EPD_WHITE : EPD_BLACK;
  for (uint16_t x = 0; x < sizeof(_buffer); x++)
  {
    _buffer[x] = data;
  }
  ESP_LOGD(TAG, "fillScreen(%d) _mono_buffer len:%d\n", data, sizeof(_buffer));
}

void Gdey029T94::update()
{
  _using_partial_mode = false;

  uint8_t xLineBytes = GDEY029T94_WIDTH / 8;
  uint8_t x1buf[xLineBytes];

  _wakeUp();

  epd_spi.send_command(SSD1680_CMD_WRITE_RAM_BW);
  for (int y = GDEY029T94_HEIGHT; y >= 0; y--)
  {
    for (uint16_t x = 0; x < xLineBytes; x++)
    {
      uint16_t idx = y * xLineBytes + x;
      x1buf[x] = _buffer[idx];

      if (x == xLineBytes - 1)
        epd_spi.send_data(x1buf, sizeof(x1buf));
    }
  }

  epd_spi.send_command(SSD1680_CMD_SET_UPDATE_SEQUENCE);
  epd_spi.send_data(GDEY029T94_UPDATE_SEQUENCE);
  epd_spi.send_command(SSD1680_CMD_UPDATE);

  wait_while_busy_("update full");
  _sleep();
}

void Gdey029T94::updateWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool using_rotation)
{
  if (!_using_partial_mode)
  {
    _using_partial_mode = true;
    _wakeUp();
  }

  if (using_rotation)
    _rotate(x, y, w, h);

  if (x >= GDEY029T94_WIDTH)
    return;

  if (y >= GDEY029T94_HEIGHT)
    return;

  uint16_t xe = gx_uint16_min(GDEY029T94_WIDTH, x + w) - 1;
  uint16_t ye = gx_uint16_min(GDEY029T94_HEIGHT, y + h) - 1;
  uint16_t xs_d8 = x / 8;
  uint16_t xe_d8 = xe / 8;

  epd_spi.send_command(SSD1680_CMD_SW_RESET);
  wait_while_busy_("Software reset");

  _setRamDataEntryMode(0x03);
  _SetRamArea(xs_d8, xe_d8, y % 256, y / 256, ye % 256, ye / 256);
  _SetRamPointer(xs_d8, y % 256, y / 256);
  wait_while_busy_("updateWindow I");

  epd_spi.send_command(SSD1680_CMD_SET_UPDATE_SEQUENCE);
  epd_spi.send_data(0xFF);

  epd_spi.send_command(SSD1680_CMD_WRITE_RAM_BW); // "current buffer"

  for (int16_t y1 = y; y1 <= ye; y1++)
  {
    for (int16_t x1 = xs_d8; x1 <= xe_d8; x1++)
    {
      uint16_t idx = y1 * (GDEY029T94_WIDTH / 8) + x1;
      uint8_t data = (idx < sizeof(_buffer)) ? _buffer[idx] : 0x00;
      epd_spi.send_data(data);
    }
  }

  epd_spi.send_command(SSD1680_CMD_WRITE_RAM_RED); // "previous buffer"
  for (int16_t y1 = y; y1 <= ye; y1++)
  {
    for (int16_t x1 = xs_d8; x1 <= xe_d8; x1++)
    {
      uint16_t idx = y1 * (GDEY029T94_WIDTH / 8) + x1;
      uint8_t data = (idx < sizeof(_buffer)) ? _buffer[idx] : 0x00;
      epd_spi.send_data(~data);
    }
  }

  epd_spi.send_command(SSD1680_CMD_UPDATE);
  wait_while_busy_("updateWindow II");
  _sleep();
}

void Gdey029T94::wait_while_busy_(const char *message)
{

  ESP_LOGD(TAG, "_wait_while_busy for %s", message);
  int64_t time_since_boot = esp_timer_get_time();

  while (true)
  {
    if (gpio_get_level((gpio_num_t)CONFIG_EINK_BUSY) == 0)
      break;

    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (esp_timer_get_time() - time_since_boot > GDEY029T94_BUSY_TIMEOUT_US)
    {
      ESP_LOGW(TAG, "Busy Timeout");
      break;
    }
  }
}

void Gdey029T94::_sleep()
{
  epd_spi.send_command(SSD1680_CMD_SET_DEEP_SLEEP_MODE); // deep sleep
  epd_spi.send_data(0x01);
}

void Gdey029T94::_rotate(uint16_t &x, uint16_t &y, uint16_t &w, uint16_t &h)
{
  switch (getRotation())
  {
  case 0:
    break;
  case 1:
    swap(x, y);
    swap(w, h);
    x = GDEY029T94_WIDTH - x - w - 1;
    break;
  case 2:
    x = GDEY029T94_WIDTH - x - w - 1;
    y = GDEY029T94_HEIGHT - y - h - 1;
    break;
  case 3:
    swap(x, y);
    swap(w, h);
    y = GDEY029T94_HEIGHT - y - h - 1;
    break;
  }
}

void Gdey029T94::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
    return;

  // check rotation, move pixel around if necessary
  switch (getRotation())
  {
  case 0:
    break;
  case 1:
    swap(x, y);
    x = GDEY029T94_VISIBLE_WIDTH - x - 1;
    break;
  case 2:
    x = GDEY029T94_VISIBLE_WIDTH - x - 1;
    y = GDEY029T94_HEIGHT - y - 1;
    break;
  case 3:
    swap(x, y);
    y = GDEY029T94_HEIGHT - y - 1;
    break;
  }

  uint16_t i = x / 8 + y * GDEY029T94_WIDTH / 8;
  uint8_t mask = 1 << (7 - x % 8);

  if (color == EPD_WHITE)
  {
    _buffer[i] = _buffer[i] | mask;
  }
  else
  {
    _buffer[i] = _buffer[i] & ~mask;
  }
}

void Gdey029T94::_wakeUp()
{
  epd_spi.hardware_reset(GDEY029T94_HW_RESET_DELAY_MS);
  wait_while_busy_("Hardware reset");
  epd_spi.send_command(SSD1680_CMD_SW_RESET);
  wait_while_busy_("Software reset");

  epd_spi.send_command(SSD1680_CMD_SET_DRIVER_OUTPUT_CONTROL);
  epd_spi.send_data(0x27);
  epd_spi.send_data(0x01);
  epd_spi.send_data(0x00);

  epd_spi.send_command(SSD1680_CMD_SET_DATA_ENTRY_MODE);
  epd_spi.send_data(0x01);

  epd_spi.send_command(SSD1680_CMD_SET_BORDER_WAVEFORM);
  epd_spi.send_data(0x05);

  epd_spi.send_command(SSD1680_CMD_SET_UPDATE_RAM);
  epd_spi.send_data(0x00);
  epd_spi.send_data(0x80);

  epd_spi.send_command(SSD1680_CMD_SELECT_TEMPERATURE_SENSOR);
  epd_spi.send_data(0x80);

  epd_spi.send_command(SSD1680_CMD_SET_RAM_X_START_END_ADDRESS);
  epd_spi.send_data(0x00);
  epd_spi.send_data(0x0F);

  epd_spi.send_command(SSD1680_CMD_SET_RAM_Y_START_END_ADDRESS);
  epd_spi.send_data(0x27);
  epd_spi.send_data(0x01);
  epd_spi.send_data(0x00);
  epd_spi.send_data(0x00);
  wait_while_busy_("wakeup CMDs");
}

void Gdey029T94::_SetRamArea(uint8_t Xstart, uint8_t Xend, uint8_t Ystart, uint8_t Ystart1, uint8_t Yend, uint8_t Yend1)
{

  ESP_LOGD(TAG, "_SetRamArea(xS:%d,xE:%d,Ys:%d,Y1s:%d,Ye:%d,Ye1:%d)\n", Xstart, Xend, Ystart, Ystart1, Yend, Yend1);

  epd_spi.send_command(SSD1680_CMD_SET_RAM_X_START_END_ADDRESS);
  epd_spi.send_data(Xstart);
  epd_spi.send_data(Xend);
  epd_spi.send_command(SSD1680_CMD_SET_RAM_Y_START_END_ADDRESS);
  epd_spi.send_data(Ystart);
  epd_spi.send_data(Ystart1);
  epd_spi.send_data(Yend);
  epd_spi.send_data(Yend1);
}

void Gdey029T94::_SetRamPointer(uint8_t addrX, uint8_t addrY, uint8_t addrY1)
{
  ESP_LOGD(TAG, "_SetRamPointer(addrX:%d,addrY:%d,addrY1:%d)\n", addrX, addrY, addrY1);

  epd_spi.send_command(SSD1680_CMD_SET_RAM_X_COUNTER);
  epd_spi.send_data(addrX);
  epd_spi.send_command(SSD1680_CMD_SET_RAM_Y_COUNTER);
  epd_spi.send_data(addrY);
  epd_spi.send_data(addrY1);
}

// We use only 0x03: At the moment this method is not used
// ram_entry_mode = 0x03; // y-increment, x-increment : normal mode
// ram_entry_mode = 0x00; // y-decrement, x-decrement
// ram_entry_mode = 0x01; // y-decrement, x-increment
// ram_entry_mode = 0x02; // y-increment, x-decrement
void Gdey029T94::_setRamDataEntryMode(uint8_t ram_entry_mode)
{
  const uint16_t xPixelsPar = GDEY029T94_X_PIXELS - 1;
  const uint16_t yPixelsPar = GDEY029T94_Y_PIXELS - 1;
  ram_entry_mode = gx_uint16_min(ram_entry_mode, 0x03);
  epd_spi.send_command(SSD1680_CMD_SET_DATA_ENTRY_MODE);
  epd_spi.send_data(ram_entry_mode);

  switch (ram_entry_mode)
  {
  case 0x00:                                                                           // x decrease, y decrease
    _SetRamArea(xPixelsPar / 8, 0x00, yPixelsPar % 256, yPixelsPar / 256, 0x00, 0x00); // X-source area, Y-gate area
    _SetRamPointer(xPixelsPar / 8, yPixelsPar % 256, yPixelsPar / 256);                // set ram
    break;
  case 0x01:                                                                           // x increase, y decrease : as in demo code
    _SetRamArea(0x00, xPixelsPar / 8, yPixelsPar % 256, yPixelsPar / 256, 0x00, 0x00); // X-source area, Y-gate area
    _SetRamPointer(0x00, yPixelsPar % 256, yPixelsPar / 256);                          // set ram
    break;
  case 0x02:                                                                           // x decrease, y increase
    _SetRamArea(xPixelsPar / 8, 0x00, 0x00, 0x00, yPixelsPar % 256, yPixelsPar / 256); // X-source area, Y-gate area
    _SetRamPointer(xPixelsPar / 8, 0x00, 0x00);                                        // set ram
    break;
  case 0x03:                                                                           // x increase, y increase : normal mode
    _SetRamArea(0x00, xPixelsPar / 8, 0x00, 0x00, yPixelsPar % 256, yPixelsPar / 256); // X-source area, Y-gate area
    _SetRamPointer(0x00, 0x00, 0x00);                                                  // set ram
    break;
  }
}
