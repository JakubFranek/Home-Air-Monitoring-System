/*
 * This file is based on source code originally from martinberlin/CalEPD GitHub repository,
 * available at https://github.com/martinberlin/CalEPD.
 * Modifications have been made to the original code by Jakub Franek (https://github.com/JakubFranek),
 * as permitted under the Apache License, Version 2.0.
 */

#include "displays/goodisplay/gdey075T7.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/task.h"
#include "esp_log.h"

#define UC8179_SPI_FREQUENCY_MHZ 20
#define GDEY075T7_BUSY_TIMEOUT_US 10000000 // 10 s

#define UC8179_CMD_PANEL_SETTING 0x00
#define UC8179_CMD_POWER_SETTING 0x01
#define UC8179_CMD_POWER_OFF 0x02
#define UC8179_CMD_POWER_OFF_SEQUENCE 0x03
#define UC8179_CMD_POWER_ON 0x04
#define UC8179_CMD_POWER_ON_MEASURE 0x05
#define UC8179_CMD_BOOSTER_SOFT_START 0x06
#define UC8179_CMD_DEEP_SLEEP 0x07
#define UC8179_CMD_DATA_START_TRANSMISSION_1 0x10
#define UC8179_CMD_DATA_STOP 0x11
#define UC8179_CMD_DISPLAY_REFRESH 0x12
#define UC8179_CMD_DATA_START_TRANSMISSION_2 0x13
#define UC8179_CMD_DUAL_SPI 0x15
#define UC8179_CMD_AUTO_SEQUENCE 0x17
#define UC8179_CMD_LUTC 0x20
#define UC8179_CMD_LUTWW 0x21
#define UC8179_CMD_LUTKW 0x22
#define UC8179_CMD_LUTWK 0x23
#define UC8179_CMD_LUTKK 0x24
#define UC8179_CMD_LUT_BORDER 0x25
#define UC8179_CMD_LUTOPT 0x2A
#define UC8179_CMD_KWOP 0x2B
#define UC8179_CMD_PLL_CONTROL 0x30
#define UC8179_CMD_TEMPERATURE_SENSOR_CALIBRATRION 0x40
#define UC8179_CMD_TEMPERATURE_SENSOR_SELECTION 0x41
#define UC8179_CMD_TEMPERATURE_SENSOR_WRITE 0x42
#define UC8179_CMD_TEMPERATURE_SENSOR_READ 0x43
#define UC8179_CMD_PANEL_BREAK_CHECK 0x44
#define UC8179_CMD_VCOM_AND_DATA_INTERVAL_SETTING 0x50
#define UC8179_CMD_LOWER_POWER_DETECTION 0x51
#define UC8179_CMD_END_VOLTAGE_SETTING 0x52
#define UC8179_CMD_TCON_SETTING 0x60
#define UC8179_CMD_RESOLUTION_SETTING 0x61
#define UC8179_CMD_GATE_SOURCE_START_SETTING 0x65
#define UC8179_CMD_REVISION 0x70
#define UC8179_CMD_GET_STATUS 0x71
#define UC8179_CMD_AUTO_MEASUREMENT_VCOM 0x80
#define UC8179_CMD_READ_VCOM 0x81
#define UC8179_CMD_VCOM_DC_SETTING 0x82
#define UC8179_CMD_PARTIAL_WINDOW 0x90
#define UC8179_CMD_PARTIAL_IN 0x91
#define UC8179_CMD_PARTIAL_OUT 0x92
#define UC8179_CMD_PROGRAM_MODE 0xA0
#define UC8179_CMD_ACTIVE_PROGRAMMING 0xA1
#define UC8179_CMD_READ_OTP 0xA2
#define UC8179_CMD_CASCADE_SETTING 0xE0
#define UC8179_CMD_POWER_SAVING 0xE3
#define UC8179_CMD_LVD_VOLTAGE_SELECT 0xE4
#define UC8179_CMD_FORCE_TEMPERATURE 0xE5
#define UC8179_CMD_TEMPERATURE_BOUNDARY_PHASE_C2 0xE7

#define GDEY075T7_X_LINE_BYTES (GDEY075T7_WIDTH / 8) // Number of bytes per horizontal line in the display buffer

Gdey075T7::Gdey075T7(EpdSpi &epd_spi) : Adafruit_GFX(GDEY075T7_WIDTH, GDEY075T7_HEIGHT),
                                        Epd(GDEY075T7_WIDTH, GDEY075T7_HEIGHT), epd_spi(epd_spi) {};

/**
 * @brief Set up SPI bus, initialize the display and set the framebuffer to white.
 */
void Gdey075T7::initialize()
{
  epd_spi.initialize(UC8179_SPI_FREQUENCY_MHZ);
  fillScreen(EPD_WHITE);
}

/**
 * @brief Wake up the display controller and set the framebuffer to its current
 * content.
 *
 * The framebuffer re-initialization is necessary after deep sleep to ensure the
 * future no-flicker refresh performs correctly and does not create pixel noise.
 *
 * @note Only change the framebuffer content in-between calling this function and
 * calling `update`.
 */
void Gdey075T7::wake_up()
{
  epd_spi.hardware_reset(10);

  power_on_();
  transfer_buffer_(UC8179_CMD_DATA_START_TRANSMISSION_1);
}

/**
 * @brief Fills the framebuffer with a given color.
 *
 * This function overrides `Adafruit_GFX::fillScreen()` for performance reasons.
 *
 * @param color The color to use to fill the framebuffer. This is either `EPD_WHITE` or `EPD_BLACK`.
 */
void Gdey075T7::fillScreen(uint16_t color)
{
  uint8_t data = (color == EPD_BLACK) ? EPD_BLACK : EPD_WHITE;
  for (uint16_t i = 0; i < sizeof(_buffer); i++)
  {
    _buffer[i] = data;
  }
}

/**
 * @brief Transfer the framebuffer to the display controller.
 *
 * @param command The command to send to the display controller.
 */
void Gdey075T7::transfer_buffer_(uint8_t command)
{
  epd_spi.send_command(command);

  uint16_t i = 0;
  uint8_t x1buf[GDEY075T7_X_LINE_BYTES];
  for (uint16_t y = 1; y <= GDEY075T7_HEIGHT; y++)
  {

    for (uint16_t x = 1; x <= GDEY075T7_X_LINE_BYTES; x++)
    {
      x1buf[x - 1] = (i < sizeof(_buffer)) ? _buffer[i] : (uint8_t)EPD_WHITE;

      if (x == GDEY075T7_X_LINE_BYTES)
      {
        epd_spi.send_data(x1buf, sizeof(x1buf));
      }

      i++;
    }
  }
}

/**
 * @brief Transfer a single color to the display controller.
 *
 * This function transfers a single color to the display controller, filling the entire
 * display with that color.
 *
 * @param command The command to send to the display controller.
 * @param color The color to use to fill the display. This is either `EPD_WHITE` or `EPD_BLACK`.
 */
void Gdey075T7::transfer_single_color_(uint8_t command, uint16_t color)
{
  uint16_t i = 0;
  uint8_t x1buf[GDEY075T7_X_LINE_BYTES];

  epd_spi.send_command(command);

  for (uint16_t y = 1; y <= GDEY075T7_HEIGHT; y++)
  {
    for (uint16_t x = 1; x <= GDEY075T7_X_LINE_BYTES; x++)
    {
      x1buf[x - 1] = color;

      if (x == GDEY075T7_X_LINE_BYTES)
      {
        epd_spi.send_data(x1buf, sizeof(x1buf));
      }

      i++;
    }
  }
}

/**
 * @brief Waits until the E-Ink display is no longer busy or a timeout occurs.
 *
 * This function continuously checks the busy status of the E-Ink display by reading
 * the level of the busy GPIO pin. It waits until the pin indicates that the display
 * is no longer busy. If the display remains busy beyond a specified timeout period,
 * a warning message is logged.
 *
 * @param message A message to log in case of a busy timeout.
 */
void Gdey075T7::wait_while_busy_(const char *message)
{
  int64_t time_since_boot = esp_timer_get_time();

  while (1)
  {
    if (gpio_get_level((gpio_num_t)CONFIG_EINK_BUSY) == 1)
      break;

    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (esp_timer_get_time() - time_since_boot > GDEY075T7_BUSY_TIMEOUT_US)
    {
      ESP_LOGW(TAG, "Busy Timeout: %s", message);
      break;
    }
  }
}

/**
 * @brief Applies the current rotation setting to the given coordinates.
 *
 * This internal helper function is used to rotate the coordinates of a rectangular
 * area before sending it to the E-Ink display controller. The rotation is applied
 * in-place, i.e., the input parameters are modified directly. The rotation is
 * performed relative to the top-left corner of the display.
 *
 * @param x The x-coordinate of the top-left corner of the rectangle.
 * @param y The y-coordinate of the top-left corner of the rectangle.
 * @param w The width of the rectangle.
 * @param h The height of the rectangle.
 */
void Gdey075T7::rotate_(uint16_t &x, uint16_t &y, uint16_t &w, uint16_t &h)
{
  ESP_LOGI(TAG, "rotate_, rotation = %d", getRotation());
  switch (getRotation())
  {
  case 1:
    swap(x, y);
    swap(w, h);
    x = GDEY075T7_WIDTH - x - w - 1;
    break;
  case 2:
    x = GDEY075T7_WIDTH - x - w - 1;
    y = GDEY075T7_HEIGHT - y - h - 1;
    break;
  case 3:
    swap(x, y);
    swap(w, h);
    y = GDEY075T7_HEIGHT - y - h - 1;
    break;
  }
}

/**
 * @brief Draws a pixel on the E-Ink display at the specified coordinates with the given color.
 *
 * This function sets a pixel in the display buffer at the specified (x, y) coordinates.
 * It checks the current rotation setting and adjusts the coordinates accordingly.
 * If the coordinates are out of bounds, the function returns without altering the buffer.
 *
 * @param x The x-coordinate of the pixel to draw.
 * @param y The y-coordinate of the pixel to draw.
 * @param color The color of the pixel to draw. Typically `EPD_WHITE` or `EPD_BLACK`.
 */
void Gdey075T7::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
    return;

  switch (getRotation())
  {
  case 0:
    break;
  case 1:
    swap(x, y);
    x = GDEY075T7_WIDTH - x - 1;
    break;
  case 2:
    x = GDEY075T7_WIDTH - x - 1;
    y = GDEY075T7_HEIGHT - y - 1;
    break;
  case 3:
    swap(x, y);
    y = GDEY075T7_HEIGHT - y - 1;
    break;
  }

  uint16_t i = x / 8 + y * GDEY075T7_WIDTH / 8;

  if (color)
  {
    _buffer[i] = (_buffer[i] | (1 << (7 - x % 8)));
  }
  else
  {
    _buffer[i] = (_buffer[i] & (0xFF ^ (1 << (7 - x % 8))));
  }
}

/**
 * @brief Send the display refresh command to the display.
 *
 * This function first sends the display refresh command to the display and then
 * waits for 10 ms before returning. It also calls `wait_while_busy_()` to wait
 * until the display is not busy anymore.
 *
 * This function is used when the display is in deep sleep mode and needs to be
 * refreshed.
 */
void Gdey075T7::send_refresh_command_()
{
  epd_spi.send_command(UC8179_CMD_DISPLAY_REFRESH); // display refresh
  vTaskDelay(10 / portTICK_PERIOD_MS);
  wait_while_busy_("send_refresh_command_");
}

/**
 * @brief Powers on the display.
 *
 * This function sends the power on command to the display and waits until the
 * display is no longer busy. If the display is already powered on, this function
 * does nothing.
 */
void Gdey075T7::power_on_()
{
  epd_spi.send_command(UC8179_CMD_POWER_ON);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  wait_while_busy_("power_on_");
}

/**
 * @brief Powers off the display.
 *
 * This function sends the power off command to the display and waits until the
 * display is no longer busy. If the display is already powered off, this function
 * does nothing.
 *
 * When the display is powered off, the update mode is reset to normal update.
 */
void Gdey075T7::power_off_()
{
  epd_spi.send_command(UC8179_CMD_POWER_OFF);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  wait_while_busy_("power_off_");
}

/**
 * @brief Put the display into deep sleep.
 *
 * This function sends the deep sleep command to the display and waits until the
 * display is no longer busy. The display is then put into deep sleep mode.
 *
 * This function is called automatically by `update()`.
 */
void Gdey075T7::sleep_()
{
  epd_spi.send_command(UC8179_CMD_VCOM_AND_DATA_INTERVAL_SETTING);
  epd_spi.send_data(0xF7); // Border output Hi-Z enabled; Border LUT selection: LUTBD; do not copy new data to old; default data polarity (1 = white) but no same-color transitions

  power_off_();

  epd_spi.send_command(UC8179_CMD_DEEP_SLEEP);
  epd_spi.send_data(0xA5); // Magic number to enter deep sleep as defined in UC8179 datasheet
}

/**
 * @brief Clear the display to white.
 *
 * Wakes up the display and performs a full screen clear to remove ghosting.
 *
 * @note This function should be called after every few calls to `update()` to avoid
 * ghosting.
 */
void Gdey075T7::clear_screen(void)
{
  epd_spi.hardware_reset(10);

  epd_spi.send_command(UC8179_CMD_POWER_SETTING);
  epd_spi.send_data(0x07); // (default) Source LV power selection: internal; Source power selection: internal; Gate power selection: internal
  epd_spi.send_data(0x07); // VGH / VGL selection: VGH = 20 V, VGL = -20 V (default)
  epd_spi.send_data(0x3F); // VDH selection: 15 V
  epd_spi.send_data(0x3F); // VDL selection: -15 V
  epd_spi.send_data(0x09); // VDHR selection: 4.2 V

  epd_spi.send_command(UC8179_CMD_BOOSTER_SOFT_START);
  epd_spi.send_data(0x17); // (default) Soft start phase A: 10 ms; Strength phase A: 3; Min. off time of GDR in phase A: 6.58 us
  epd_spi.send_data(0x17); // (default) Soft start phase B: 10 ms; Strength phase B: 3; Min. off time of GDR in phase B: 6.58 us
  epd_spi.send_data(0x28); // Strength phase C1: 6; Min. off time of GDR in phase C1: 0.27 us
  epd_spi.send_data(0x17); // (default) Booster phase C2: disabled; Strength phase C2: 3; Min. off time of GDR in phase C2: 6.58 us

  power_on_();

  epd_spi.send_command(UC8179_CMD_PANEL_SETTING);
  epd_spi.send_data(0x1F); // LUT selection: OTP; B&W mode; Gate scan direction: up; Source shift direction: right; Booster: on; Soft reset: no effect

  epd_spi.send_command(UC8179_CMD_RESOLUTION_SETTING);
  epd_spi.send_data(WIDTH / 256);
  epd_spi.send_data(WIDTH % 256);
  epd_spi.send_data(HEIGHT / 256);
  epd_spi.send_data(HEIGHT % 256);

  epd_spi.send_command(UC8179_CMD_DUAL_SPI);
  epd_spi.send_data(0x00); // (default) Dual SPI mode: disabled

  epd_spi.send_command(UC8179_CMD_VCOM_AND_DATA_INTERVAL_SETTING);
  epd_spi.send_data(0x10); // Border output Hi-Z disabled; Border LUT selection: LUTR; do not copy new data to old; default data polarity (1 = white) with same-color transitions
  epd_spi.send_data(0x07); // VCOM and data interval: 0d10

  epd_spi.send_command(UC8179_CMD_TCON_SETTING);
  epd_spi.send_data(0x22); // Source to Gate non-overlap: 0d12; Gate to Source non-overlap: 0d12

  transfer_single_color_(UC8179_CMD_DATA_START_TRANSMISSION_1, ~EPD_BLACK);
  transfer_single_color_(UC8179_CMD_DATA_START_TRANSMISSION_2, ~EPD_WHITE);
  send_refresh_command_();
}

/**
 * @brief Perform no-flicker screen update and enter deep sleep.
 *
 * @note Call `clear_screen` after every few `update` calls to avoid ghosting.
 * @note Call `wake_up` before next call to any display method, including this one.
 */
void Gdey075T7::update()
{
  epd_spi.hardware_reset(10);

  epd_spi.send_command(UC8179_CMD_PANEL_SETTING);
  epd_spi.send_data(0x1F); // LUT selection: OTP; B&W mode; Gate scan direction: up; Source shift direction: right; Booster: on; Soft reset: no effect

  power_on_();

  epd_spi.send_command(UC8179_CMD_CASCADE_SETTING);
  epd_spi.send_data(0x02); // Temperature value is defined by TS_SET register
  epd_spi.send_command(UC8179_CMD_FORCE_TEMPERATURE);
  epd_spi.send_data(0x6E); // Write 6E into TS_SET register (110 degrees Celsius)
  epd_spi.send_command(UC8179_CMD_VCOM_AND_DATA_INTERVAL_SETTING);
  epd_spi.send_data(0xA9); // Border output Hi-Z disabled; Border LUT selection: LUTKW, Copy new data to old data enabled; default data polarity (0 = black)
  epd_spi.send_data(0x07); // VCOM and data interval: 0d10

  epd_spi.send_command(UC8179_CMD_PARTIAL_IN);
  epd_spi.send_command(UC8179_CMD_PARTIAL_WINDOW);
  epd_spi.send_data(0); // x-start
  epd_spi.send_data(0);
  epd_spi.send_data((WIDTH - 1) / 256); // x-end
  epd_spi.send_data((WIDTH - 1) % 256 - 1);
  epd_spi.send_data(0); // y-start
  epd_spi.send_data(0);
  epd_spi.send_data((HEIGHT - 1) / 256); // y-end
  epd_spi.send_data((HEIGHT - 1) % 256 - 1);
  epd_spi.send_data(0x01); // Full-screen update

  transfer_buffer_(UC8179_CMD_DATA_START_TRANSMISSION_2); // Transfers frame buffer to "new" RAM
  send_refresh_command_();
  epd_spi.send_command(UC8179_CMD_PARTIAL_OUT);

  sleep_();
}
