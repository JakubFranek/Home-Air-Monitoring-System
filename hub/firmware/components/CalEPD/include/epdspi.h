/*
 * This file is based on source code originally from martinberlin/CalEPD GitHub repository,
 * available at https://github.com/martinberlin/CalEPD.
 *
 * Modifications have been made to the original code by Jakub Franek (https://github.com/JakubFranek),
 * as permitted under the Apache License, Version 2.0.
 */

#ifndef epdspi_h
#define epdspi_h

#include "driver/spi_master.h"
#include "driver/gpio.h"

using namespace std;

class EpdSpi
{
public:
  void send_command(const uint8_t cmd); // Should override if IoInterface is there
  void send_data(uint8_t data);
  void send_data(const uint8_t *data, size_t len);
  void hardware_reset(uint8_t wait_ms);
  void initialize(uint8_t frequency);

private:
  spi_device_handle_t spi;
  static constexpr const char *TAG = "EpdSpi"; // needed for logging
};
#endif