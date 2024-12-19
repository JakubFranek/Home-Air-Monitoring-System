#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "sht4x.h"
#include "sgp41.h"

static const char *TAG = "sensor_control";

void setup_sht4x(void);
void measure_sht4x(void);
int8_t sht4x_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length);
int8_t sht4x_i2c_read(uint8_t address, uint8_t *payload, uint8_t length);
int8_t sgp41_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length);
int8_t sgp41_i2c_read(uint8_t address, uint8_t *payload, uint8_t length);

i2c_master_bus_handle_t bus;
i2c_master_bus_config_t i2c_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 21,
    .scl_io_num = 22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 1,
    .trans_queue_depth = 0};

i2c_device_config_t sht4x_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SHT4X_I2C_ADDR_A,
    .scl_speed_hz = 100000,
    .scl_wait_us = 0};
i2c_master_dev_handle_t sht4x_device_handle;

Sht4xDevice sht4x_device = {
    .i2c_write = &sht4x_i2c_write,
    .i2c_read = &sht4x_i2c_read,
    .i2c_address = SHT4X_I2C_ADDR_A,
};
Sht4xStatus sht4x_status;
Sht4xData sht4x_data;
uint32_t sht4x_serial_number;

i2c_device_config_t sgp41_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SGP41_I2C_ADDRESS,
    .scl_speed_hz = 100000,
    .scl_wait_us = 0,
};
i2c_master_dev_handle_t sgp41_device_handle;

Sgp41Device sgp41_device = {
    .i2c_write = &sgp41_i2c_write,
    .i2c_read = &sgp41_i2c_read,
};
uint64_t sgp41_serial_number;
Sgp41Status sgp41_status;
Sgp41Data sgp41_data;

void setup_i2c_bus(void)
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &bus));
}

void setup_sht4x(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &sht4x_config, &sht4x_device_handle));

    sht4x_status = sht4x_request_serial_number(&sht4x_device);
    ESP_LOGI(TAG, "[SHT4X] Request Serial Number, status = %d", sht4x_status);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    sht4x_status = sht4x_read_serial_number(&sht4x_device, &sht4x_serial_number);
    ESP_LOGI(TAG, "[SHT4X] Read Serial Number, serial number = %ld, status = %d", sht4x_serial_number, sht4x_status);
}

void measure_sht4x(void)
{
    sht4x_status = sht4x_start_measurement(&sht4x_device, SHT4X_I2C_CMD_MEAS_HIGH_PREC);
    ESP_LOGI(TAG, "[SHT4X] Start Measurement, status = %d", sht4x_status);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    sht4x_status = sht4x_read_measurement(&sht4x_device, &sht4x_data);
    ESP_LOGI(TAG, "[SHT4X] Read Data, status = %d", sht4x_status);
    ESP_LOGI(TAG, "[SHT4X] Temperature = %f °C, Rel. Humidity = %f %%", sht4x_data.temperature / 1000.0, sht4x_data.humidity / 1000.0);
}

void setup_sgp41(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &sgp41_config, &sgp41_device_handle));

    sgp41_status = sgp41_initialize(&sgp41_device);

    sgp41_status = sgp41_get_serial_number(&sgp41_device, &sgp41_serial_number);
    ESP_LOGI(TAG, "[SGP41] Get Serial Number, status = %d, serial number: %lld",
             sgp41_status, sgp41_serial_number);

    sgp41_status = sgp41_execute_self_test(&sgp41_device);
    ESP_LOGI(TAG, "[SGP41] Execute Self Test, status = %d", sgp41_status);
    vTaskDelay(350 / portTICK_PERIOD_MS);
    sgp41_status = sgp41_evaluate_self_test(&sgp41_device);
    ESP_LOGI(TAG, "[SGP41] Evaluate Self Test, status = %d", sgp41_status);

    sgp41_status = sgp41_execute_conditioning(&sgp41_device);
    ESP_LOGI(TAG, "[SGP41] Execute Conditioning, status = %d", sgp41_status);

    vTaskDelay(10000 / portTICK_PERIOD_MS); // max 10s conditioning*/

    sgp41_status = sgp41_turn_heater_off(&sgp41_device);
    ESP_LOGI(TAG, "[SGP41] Turn Heater Off, status = %d", sgp41_status);
}

void measure_sgp41(void)
{
    sgp41_status = sgp41_measure_raw_signals(&sgp41_device, NULL, NULL);
    ESP_LOGI(TAG, "[SGP41] Measure Raw Signals, status = %d", sgp41_status);
    vTaskDelay(55 / portTICK_PERIOD_MS);
    sgp41_status = sgp41_read_gas_indices(&sgp41_device, &sgp41_data);
    ESP_LOGI(TAG, "[SGP41] Read Gas Indices, status = %d, voc = %d, nox = %d", sgp41_status, (int)sgp41_data.voc_index, (int)sgp41_data.nox_index);
}

int8_t sht4x_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_transmit(sht4x_device_handle, payload, length, 10);
    return err == ESP_OK ? 0 : -1;
}

int8_t sht4x_i2c_read(uint8_t address, uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_receive(sht4x_device_handle, payload, length, 10);
    return err == ESP_OK ? 0 : -1;
}

int8_t sgp41_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length)
{

    (void)address; // Address not necessary
    esp_err_t err = i2c_master_transmit(sgp41_device_handle, payload, length, 10);
    return err == ESP_OK ? 0 : -1;
}

int8_t sgp41_i2c_read(uint8_t address, uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_receive(sgp41_device_handle, payload, length, 10);
    return err == ESP_OK ? 0 : -1;
}
