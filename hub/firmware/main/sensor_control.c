#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "sht4x.h"
#include "sgp41.h"
#include "bme280_i2c.h"
#include "scd4x.h"

static const char *TAG = "sensor_control";

void setup_sht4x(void);
void measure_sht4x(void);
int8_t sht4x_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length);
int8_t sht4x_i2c_read(uint8_t address, uint8_t *payload, uint8_t length);
int8_t sgp41_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length);
int8_t sgp41_i2c_read(uint8_t address, uint8_t *payload, uint8_t length);
int8_t bme280_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length);
int8_t bme280_i2c_read(uint8_t address, uint8_t *payload, uint8_t length);
int8_t bme280_delay_ms(uint16_t ms);
int8_t scd4x_i2c_write(uint8_t address, const uint8_t *payload, size_t length);
int8_t scd4x_i2c_read(uint8_t address, uint8_t *payload, size_t length);
int8_t scd4x_delay_ms(uint16_t ms);

/* ---------- I2C ---------- */

i2c_master_bus_handle_t bus;
i2c_master_bus_config_t i2c_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 21,
    .scl_io_num = 22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 1,
    .trans_queue_depth = 0};

/* ---------- SHT4x ---------- */

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

/* ---------- SGP41 ---------- */

#define SGP41_SAMPLING_INTERVAL_S 60.0f

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
    .sampling_period_s = SGP41_SAMPLING_INTERVAL_S,
};
uint64_t sgp41_serial_number;
Sgp41Status sgp41_status;
Sgp41Data sgp41_data;

/* ---------- BME280 ---------- */

i2c_device_config_t bme280_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = BME280_I2C_ADDRESS_SDO_HIGH,
    .scl_speed_hz = 100000,
    .scl_wait_us = 0};
i2c_master_dev_handle_t bme280_device_handle;

Bme280Device bme280_device = {
    .i2c_write = &bme280_i2c_write,
    .i2c_read = &bme280_i2c_read,
    .address = BME280_I2C_ADDRESS_SDO_HIGH,
    .config = {
        .filter = BME280_FILTER_OFF,
        .spi_3wire_enable = false,
        .standby_time = BME280_STANDBY_TIME_0_5_MS,
        .temperature_oversampling = BME280_OVERSAMPLING_X1,
        .pressure_oversampling = BME280_OVERSAMPLING_X1,
        .humidity_oversampling = BME280_OVERSAMPLING_X1}};
Bme280Status bme280_status;
Bme280Data bme280_data;
bool bme280_measurement_in_progress = false;

/* ---------- SCD4x ---------- */

#define SCD4X_MAX_RETRY_COUNT 10
#define SCD4X_RETRY_PERIOD_MS 100

i2c_device_config_t scd4x_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SCD4X_I2C_ADDRESS,
    .scl_speed_hz = 100000,
    .scl_wait_us = 0,
};
i2c_master_dev_handle_t scd4x_device_handle;

Scd4xDevice scd4x_device = {
    .i2c_write = &scd4x_i2c_write,
    .i2c_read = &scd4x_i2c_read,
    .delay_ms = &scd4x_delay_ms,
};
Scd4xStatus scd4x_status;
Scd4xData scd4x_data;
bool scd4x_data_ready = false;
uint64_t scd4x_serial_number;

void setup_i2c_bus(void)
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &bus));
}

void setup_sht4x(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &sht4x_config, &sht4x_device_handle));

    sht4x_status = sht4x_request_serial_number(&sht4x_device);
    ESP_LOGI(TAG, "[SHT4x] Request Serial Number, status = %d", sht4x_status);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    sht4x_status = sht4x_read_serial_number(&sht4x_device, &sht4x_serial_number);
    ESP_LOGI(TAG, "[SHT4x] Read Serial Number, serial number = %ld, status = %d", sht4x_serial_number, sht4x_status);
}

void measure_sht4x(void)
{
    sht4x_status = sht4x_start_measurement(&sht4x_device, SHT4X_I2C_CMD_MEAS_HIGH_PREC);
    ESP_LOGI(TAG, "[SHT4x] Start Measurement, status = %d", sht4x_status);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    sht4x_status = sht4x_read_measurement(&sht4x_device, &sht4x_data);
    ESP_LOGI(TAG, "[SHT4x] Read Data, status = %d", sht4x_status);
    ESP_LOGI(TAG, "[SHT4x] Temperature = %.2f °C, Rel. humidity = %.2f %%", sht4x_data.temperature / 1000.0, sht4x_data.humidity / 1000.0);
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

void setup_bme280(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &bme280_config, &bme280_device_handle));

    bme280_status = bme280_reset(&bme280_device);
    ESP_LOGI(TAG, "[BME280] Reset, status = %d", bme280_status);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    bme280_status = bme280_init(&bme280_device);
    ESP_LOGI(TAG, "[BME280] Init, status = %d", bme280_status);
}

void measure_bme280(void)
{
    bme280_status = bme280_set_mode(&bme280_device, BME280_MODE_FORCED);
    ESP_LOGI(TAG, "[BME280] Set Forced Mode, status = %d", bme280_status);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    bme280_status = bme280_read_measurement(&bme280_device, &bme280_data);
    ESP_LOGI(TAG, "[BME280] Read Data, Temperature = %.2f °C, Rel. humidity = %.2f %%, Pressure = %.2f Pa, status = %d",
             bme280_data.temperature / 100.0, bme280_data.humidity / 1000.0, bme280_data.pressure / 10.0, bme280_status);
}

void setup_scd4x(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &scd4x_config, &scd4x_device_handle));

    scd4x_status = scd4x_get_serial_number(&scd4x_device, &scd4x_serial_number);
    ESP_LOGI(TAG, "[SCD4x] Read Serial Number, serial number = %lld, status = %d", scd4x_serial_number, scd4x_status);

    scd4x_status = scd4x_perform_self_test(&scd4x_device);
    ESP_LOGI(TAG, "[SCD4x] Perform Self Test, status = %d", scd4x_status);
}

void measure_scd4x(void)
{
    scd4x_status = scd41_measure_single_shot(&scd4x_device);
    ESP_LOGI(TAG, "[SCD4x] Measure Single Shot, status = %d", scd4x_status);

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    uint8_t retry_count = 0;
    while (true)
    {
        scd4x_status = scd4x_get_data_ready_status(&scd4x_device, &scd4x_data_ready);
        ESP_LOGI(TAG, "[SCD4x] Read Data Ready Status, data ready = %d, status = %d", scd4x_data_ready, scd4x_status);
        if (scd4x_data_ready)
        {
            break;
        }
        else if (retry_count > SCD4X_MAX_RETRY_COUNT)
        {
            ESP_LOGE(TAG, "[SCD4x] Data not ready after %d retries", SCD4X_MAX_RETRY_COUNT);
            return;
        }
        retry_count++;
        vTaskDelay(SCD4X_RETRY_PERIOD_MS / portTICK_PERIOD_MS);
    }

    scd4x_status = scd4x_read_measurement(&scd4x_device, &scd4x_data);
    ESP_LOGI(TAG, "[SCD4x] Read Data, CO2 concentration = %d ppm, Temperature = %.2f °C, Rel. humidity = %.2f %% status = %d",
             scd4x_data.co2_ppm, scd4x_data.temperature / 100.0, scd4x_data.relative_humidity / 100.0, scd4x_status);
    scd4x_data_ready = false;
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

int8_t bme280_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_transmit(bme280_device_handle, payload, length, 20);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return err == ESP_OK ? 0 : -1;
}

int8_t bme280_i2c_read(uint8_t address, uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_receive(bme280_device_handle, payload, length, 20);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return err == ESP_OK ? 0 : -1;
}

int8_t bme280_delay_ms(uint16_t ms)
{
    vTaskDelay(ms);
    return 0;
}

int8_t scd4x_i2c_write(uint8_t address, const uint8_t *payload, size_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_transmit(scd4x_device_handle, payload, length, 10);
    return err == ESP_OK ? 0 : -1;
}

int8_t scd4x_i2c_read(uint8_t address, uint8_t *payload, size_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_receive(scd4x_device_handle, payload, length, 10);
    return err == ESP_OK ? 0 : -1;
}

int8_t scd4x_delay_ms(uint16_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
    return 0;
}