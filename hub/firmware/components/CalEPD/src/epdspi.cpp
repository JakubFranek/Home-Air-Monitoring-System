/*
 * This file is based on source code originally from martinberlin/CalEPD GitHub repository,
 * available at https://github.com/martinberlin/CalEPD.
 *
 * Modifications have been made to the original code by Jakub Franek (https://github.com/JakubFranek),
 * as permitted under the Apache License, Version 2.0.
 */

#include <epdspi.h>
#include <string.h>
#include "freertos/task.h"
#include "esp_log.h"
#include <algorithm>

/**
 * @brief Initializes the SPI interface for the EPD.
 *
 * This function sets up the GPIO pins and SPI bus for communication with the
 * EPD. It configures the chip select (CS), data/command (DC), reset (RST), and
 * busy pins, and initializes the SPI bus with the specified frequency.
 *
 * @param frequency_MHz The SPI clock frequency in megahertz (default is 4 MHz).
 */
void EpdSpi::initialize(uint8_t frequency_MHz = 4)
{
    gpio_set_direction((gpio_num_t)CONFIG_EINK_SPI_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)CONFIG_EINK_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)CONFIG_EINK_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)CONFIG_EINK_BUSY, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)CONFIG_EINK_BUSY, GPIO_PULLUP_ONLY);

    gpio_set_level((gpio_num_t)CONFIG_EINK_SPI_CS, 1);
    gpio_set_level((gpio_num_t)CONFIG_EINK_DC, 1);
    gpio_set_level((gpio_num_t)CONFIG_EINK_RST, 1);

    spi_bus_config_t buscfg = {
        .mosi_io_num = CONFIG_EINK_SPI_MOSI,
        .miso_io_num = -1, // MISO not used
        .sclk_io_num = CONFIG_EINK_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 1000,
    };

    // Config Frequency and SS GPIO
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = frequency_MHz * 1000000,
        .input_delay_ns = 0,
        .spics_io_num = CONFIG_EINK_SPI_CS,
        .flags = (SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE),
        .queue_size = 5,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));

    ESP_LOGI(TAG, "SPI initialized at %d MHz", frequency_MHz);
}

/**
 * @brief Send a single byte command over SPI to the EPD
 *
 * The DC pin is set low to indicate a command is being sent.
 * After the command is sent the DC pin is set high to indicate data is being sent.
 *
 * @note This function does not check if the EPD is busy.
 * The caller should check the EPD busy pin before calling this function.
 *
 * @param command The 8-bit command to send
 */
void EpdSpi::send_command(const uint8_t command)
{
    ESP_LOGD(TAG, "Sending command: 0x%x", command);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = 8;             // Command is always 8 bits
    t.tx_buffer = &command;   // The data is the command itself

    gpio_set_level((gpio_num_t)CONFIG_EINK_DC, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
    gpio_set_level((gpio_num_t)CONFIG_EINK_DC, 1);
}

/**
 * @brief Send a single byte of data over SPI to the EPD
 *
 * @param data The byte of data to send
 */
void EpdSpi::send_data(uint8_t data)
{
    ESP_LOGD(TAG, "Sending data: 0x%x", data);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;

    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

/**
 * @brief Send an array of data to the EPD over SPI.
 *
 * @param data  Pointer to the data to be sent.
 * @param length  The length of the data to be sent, in bytes.
 */
void EpdSpi::send_data(const uint8_t *data, size_t length)
{
    if (length == 0)
        return;

    ESP_LOGD(TAG, "Sending array of data, length: %d", length);
    spi_transaction_t t;

    // Maximum transfer size is SOC_SPI_MAXIMUM_BUFFER_SIZE = 64 bytes
    // Split larger transfers into chunks of SOC_SPI_MAXIMUM_BUFFER_SIZE bytes
    for (size_t i = 0; i < length; i += SOC_SPI_MAXIMUM_BUFFER_SIZE)
    {
        memset(&t, 0, sizeof(t));
        t.length = 8 * std::min(SOC_SPI_MAXIMUM_BUFFER_SIZE, (int)(length - i));
        t.tx_buffer = data + i;
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
    }
}

/**
 * @brief Performs a hardware reset of the EPD
 *
 * The reset line is pulled low for the given amount of time, and then released.
 * The EPD is then given the same amount of time to do its internal reset.
 *
 * @param wait_ms The amount of time to hold the reset line low and then high, in milliseconds.
 */
void EpdSpi::hardware_reset(uint8_t wait_ms = 20)
{
    gpio_set_level((gpio_num_t)CONFIG_EINK_RST, 0);
    vTaskDelay(wait_ms / portTICK_PERIOD_MS);
    gpio_set_level((gpio_num_t)CONFIG_EINK_RST, 1);
    vTaskDelay(wait_ms / portTICK_PERIOD_MS);
}
