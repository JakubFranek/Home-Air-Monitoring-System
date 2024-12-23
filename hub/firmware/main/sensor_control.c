#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "sht4x.h"
#include "sgp41.h"
#include "bme280_i2c.h"
#include "scd4x.h"
#include "sps30_i2c.h"

#include "sensor_control.h"
static const char *TAG = "sensor_control";

#define RETURN_IF_NOT_ZERO(status, data, error_counter) \
    data = status;                                      \
    if (status != 0)                                    \
    {                                                   \
        error_counter++;                                \
        return status;                                  \
    }

/* ---------- Static Prototypes ---------- */

static int8_t sht4x_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length);
static int8_t sht4x_i2c_read(uint8_t address, uint8_t *payload, uint8_t length);
static int8_t sgp41_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length);
static int8_t sgp41_i2c_read(uint8_t address, uint8_t *payload, uint8_t length);
static int8_t sgp41_delay_ms(uint16_t ms);
static int8_t bme280_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length);
static int8_t bme280_i2c_read(uint8_t address, uint8_t *payload, uint8_t length);
static int8_t scd4x_i2c_write(uint8_t address, const uint8_t *payload, size_t length);
static int8_t scd4x_i2c_read(uint8_t address, uint8_t *payload, size_t length);
static int8_t scd4x_delay_ms(uint16_t ms);
static int8_t sps30_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length);
static int8_t sps30_i2c_read(uint8_t address, uint8_t *payload, uint8_t length);
static int8_t sps30_delay_ms(uint16_t ms);

/* ---------- I2C ---------- */

static i2c_master_bus_handle_t bus;
static i2c_master_bus_config_t i2c_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 21,
    .scl_io_num = 22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 1,
    .trans_queue_depth = 0};

/* ---------- SHT4x ---------- */

static i2c_device_config_t sht4x_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SHT4X_I2C_ADDR_A,
    .scl_speed_hz = 100000,
    .scl_wait_us = 0};
static i2c_master_dev_handle_t sht4x_device_handle;

static Sht4xDevice sht4x_device = {
    .i2c_write = &sht4x_i2c_write,
    .i2c_read = &sht4x_i2c_read,
    .i2c_address = SHT4X_I2C_ADDR_A};
static Sht4xStatus sht4x_status;
static Sht4xData sht4x_data;
static uint32_t sht4x_serial_number;

/* ---------- SGP41 ---------- */

#define SGP41_SAMPLING_INTERVAL_S 60.0f

static i2c_device_config_t sgp41_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SGP41_I2C_ADDRESS,
    .scl_speed_hz = 100000,
    .scl_wait_us = 0,
};
static i2c_master_dev_handle_t sgp41_device_handle;

static Sgp41Device sgp41_device = {
    .i2c_write = &sgp41_i2c_write,
    .i2c_read = &sgp41_i2c_read,
    .delay_ms = &sgp41_delay_ms,
    .sampling_period_s = SGP41_SAMPLING_INTERVAL_S};
static uint64_t sgp41_serial_number;
static Sgp41Status sgp41_status;
static Sgp41Data sgp41_data;

/* ---------- BME280 ---------- */

static i2c_device_config_t bme280_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = BME280_I2C_ADDRESS_SDO_HIGH,
    .scl_speed_hz = 100000,
    .scl_wait_us = 0};
static i2c_master_dev_handle_t bme280_device_handle;

static Bme280Device bme280_device = {
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
static Bme280Status bme280_status;
static Bme280Data bme280_data;

/* ---------- SCD4x ---------- */

#define SCD4X_MAX_RETRY_COUNT 10
#define SCD4X_RETRY_PERIOD_MS 100

static i2c_device_config_t scd4x_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SCD4X_I2C_ADDRESS,
    .scl_speed_hz = 100000,
    .scl_wait_us = 0};
static i2c_master_dev_handle_t scd4x_device_handle;

static Scd4xDevice scd4x_device = {
    .i2c_write = &scd4x_i2c_write,
    .i2c_read = &scd4x_i2c_read,
    .delay_ms = &scd4x_delay_ms};
static Scd4xStatus scd4x_status;
static Scd4xData scd4x_data;
static bool scd4x_data_ready = false;
static uint64_t scd4x_serial_number;

/* ---------- SPS30 ---------- */

#define SPS30_MAX_RETRY_COUNT 10
#define SPS30_RETRY_PERIOD_MS 100

static i2c_device_config_t sps30_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SPS30_I2C_ADDRESS,
    .scl_speed_hz = 100000,
    .scl_wait_us = 0};
static i2c_master_dev_handle_t sps30_device_handle;

static Sps30Device sps30_device = {
    .i2c_write = &sps30_i2c_write,
    .i2c_read = &sps30_i2c_read,
    .delay_ms = &sps30_delay_ms};
static Sps30Status sps30_status;
static Sps30FirmwareVersion sps30_version;
static char sps30_product_type[8] = {'\0'};
static char sps30_serial_number[32] = {'\0'};
static Sps30StatusFlags sps30_flags;
static Sps30FloatData sps30_float_data;
static bool sps30_data_ready = false;

/* ---------- Other static variables ---------- */

static struct timeval current_time;

/* ---------- Sensor Data Struct ---------- */

HubSensorData sensor_hub_data;

/* ---------- Functions ---------- */

/*void setup_sensors(void)
{
    setup_i2c_bus();

    setup_sht4x();
    setup_sgp41();
    setup_bme280();
    setup_scd4x();
    setup_sps30();
}*/

int8_t setup_i2c_bus(void)
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &bus));
    return 0;
}

int8_t setup_sht4x(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &sht4x_config, &sht4x_device_handle));

    sht4x_status = sht4x_request_serial_number(&sht4x_device);
    ESP_LOGI(TAG, "[SHT4x] Request Serial Number, status = %d", sht4x_status);
    RETURN_IF_NOT_ZERO((int8_t)sht4x_status, sensor_hub_data.temperature_humidity_status, sensor_hub_data.temperature_humidity_errors);

    vTaskDelay(20 / portTICK_PERIOD_MS);

    sht4x_status = sht4x_read_serial_number(&sht4x_device, &sht4x_serial_number);
    ESP_LOGI(TAG, "[SHT4x] Read Serial Number, serial number = %ld, status = %d", sht4x_serial_number, sht4x_status);
    RETURN_IF_NOT_ZERO((int8_t)sht4x_status, sensor_hub_data.temperature_humidity_status, sensor_hub_data.temperature_humidity_errors);

    return 0;
}

int8_t measure_sht4x(void)
{
    sht4x_status = sht4x_start_measurement(&sht4x_device, SHT4X_I2C_CMD_MEAS_HIGH_PREC);
    ESP_LOGI(TAG, "[SHT4x] Start Measurement, status = %d", sht4x_status);
    RETURN_IF_NOT_ZERO((int8_t)sht4x_status, sensor_hub_data.temperature_humidity_status, sensor_hub_data.temperature_humidity_errors);

    vTaskDelay(20 / portTICK_PERIOD_MS);

    sht4x_status = sht4x_read_measurement(&sht4x_device, &sht4x_data);
    ESP_LOGI(TAG, "[SHT4x] Read Data, Temperature = %.2f °C, Rel. humidity = %.2f %%, status = %d",
             sht4x_data.temperature / 1000.0, sht4x_data.humidity / 1000.0, sht4x_status);
    RETURN_IF_NOT_ZERO((int8_t)sht4x_status, sensor_hub_data.temperature_humidity_status, sensor_hub_data.temperature_humidity_errors);

    sensor_hub_data.temperature = sht4x_data.temperature / 1000.0;
    sensor_hub_data.humidity = sht4x_data.humidity / 1000.0;
    sensor_hub_data.temperature_humidity_status = sht4x_status;
    gettimeofday(&current_time, NULL);
    sensor_hub_data.temperature_humidity_timestamp = current_time;
    sensor_hub_data.temperature_humidity_measurements++;

    return 0;
}

int8_t setup_sgp41(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &sgp41_config, &sgp41_device_handle));

    sgp41_status = sgp41_initialize(&sgp41_device);
    RETURN_IF_NOT_ZERO((int8_t)sgp41_status, sensor_hub_data.gas_index_status, sensor_hub_data.gas_index_errors);

    sgp41_status = sgp41_get_serial_number(&sgp41_device, &sgp41_serial_number);
    ESP_LOGI(TAG, "[SGP41] Get Serial Number, serial number: %lld, status = %d",
             sgp41_serial_number, sgp41_status);
    RETURN_IF_NOT_ZERO((int8_t)sgp41_status, sensor_hub_data.gas_index_status, sensor_hub_data.gas_index_errors);

    sgp41_status = sgp41_execute_self_test(&sgp41_device);
    ESP_LOGI(TAG, "[SGP41] Execute Self Test, status = %d", sgp41_status);
    RETURN_IF_NOT_ZERO((int8_t)sgp41_status, sensor_hub_data.gas_index_status, sensor_hub_data.gas_index_errors);

    sgp41_status = sgp41_execute_conditioning(&sgp41_device);
    ESP_LOGI(TAG, "[SGP41] Execute Conditioning, status = %d", sgp41_status);
    RETURN_IF_NOT_ZERO((int8_t)sgp41_status, sensor_hub_data.gas_index_status, sensor_hub_data.gas_index_errors);

    vTaskDelay(10000 / portTICK_PERIOD_MS); // max 10s conditioning*/

    sgp41_status = sgp41_turn_heater_off(&sgp41_device);
    ESP_LOGI(TAG, "[SGP41] Turn Heater Off, status = %d", sgp41_status);
    RETURN_IF_NOT_ZERO((int8_t)sgp41_status, sensor_hub_data.gas_index_status, sensor_hub_data.gas_index_errors);

    return 0;
}

int8_t measure_sgp41(void)
{
    sgp41_status = sgp41_measure_raw_signals(&sgp41_device, NULL, NULL);
    ESP_LOGI(TAG, "[SGP41] Measure Raw Signals, status = %d", sgp41_status);
    RETURN_IF_NOT_ZERO((int8_t)sgp41_status, sensor_hub_data.gas_index_status, sensor_hub_data.gas_index_errors);

    vTaskDelay(55 / portTICK_PERIOD_MS);

    sgp41_status = sgp41_read_gas_indices(&sgp41_device, &sgp41_data);
    ESP_LOGI(TAG, "[SGP41] Read Gas Indices, voc = %d, nox = %d, status = %d",
             (int)sgp41_data.voc_index, (int)sgp41_data.nox_index, sgp41_status);
    RETURN_IF_NOT_ZERO((int8_t)sgp41_status, sensor_hub_data.gas_index_status, sensor_hub_data.gas_index_errors);

    sensor_hub_data.voc_index = sgp41_data.voc_index;
    sensor_hub_data.nox_index = sgp41_data.nox_index;
    gettimeofday(&current_time, NULL);
    sensor_hub_data.gas_index_timestamp = current_time;
    sensor_hub_data.gas_index_measurements++;

    return 0;
}

int8_t setup_bme280(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &bme280_config, &bme280_device_handle));

    bme280_status = bme280_reset(&bme280_device);
    ESP_LOGI(TAG, "[BME280] Reset, status = %d", bme280_status);
    RETURN_IF_NOT_ZERO((int8_t)bme280_status, sensor_hub_data.pressure_status, sensor_hub_data.pressure_errors);

    vTaskDelay(20 / portTICK_PERIOD_MS);

    bme280_status = bme280_init(&bme280_device);
    ESP_LOGI(TAG, "[BME280] Init, status = %d", bme280_status);
    RETURN_IF_NOT_ZERO((int8_t)bme280_status, sensor_hub_data.pressure_status, sensor_hub_data.pressure_errors);

    return 0;
}

int8_t measure_bme280(void)
{
    bme280_status = bme280_set_mode(&bme280_device, BME280_MODE_FORCED);
    ESP_LOGI(TAG, "[BME280] Set Forced Mode, status = %d", bme280_status);
    RETURN_IF_NOT_ZERO((int8_t)bme280_status, sensor_hub_data.pressure_status, sensor_hub_data.pressure_errors);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    bme280_status = bme280_read_measurement(&bme280_device, &bme280_data);
    ESP_LOGI(TAG, "[BME280] Read Data, Temperature = %.2f °C, Rel. humidity = %.2f %%, Pressure = %.1f hPa, status = %d",
             bme280_data.temperature / 100.0, bme280_data.humidity / 1000.0, bme280_data.pressure / 10.0, bme280_status);
    RETURN_IF_NOT_ZERO((int8_t)bme280_status, sensor_hub_data.pressure_status, sensor_hub_data.pressure_errors);

    sensor_hub_data.pressure_hPa = bme280_data.pressure / 10.0;
    gettimeofday(&current_time, NULL);
    sensor_hub_data.pressure_timestamp = current_time;
    sensor_hub_data.pressure_measurements++;

    return 0;
}

int8_t setup_scd4x(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &scd4x_config, &scd4x_device_handle));

    scd4x_status = scd4x_get_serial_number(&scd4x_device, &scd4x_serial_number);
    ESP_LOGI(TAG, "[SCD4x] Read Serial Number, serial number = %lld, status = %d", scd4x_serial_number, scd4x_status);
    RETURN_IF_NOT_ZERO((int8_t)scd4x_status, sensor_hub_data.co2_status, sensor_hub_data.co2_errors);

    vTaskDelay(20 / portTICK_PERIOD_MS);

    scd4x_status = scd4x_perform_self_test(&scd4x_device);
    ESP_LOGI(TAG, "[SCD4x] Perform Self Test, status = %d", scd4x_status);
    RETURN_IF_NOT_ZERO((int8_t)scd4x_status, sensor_hub_data.co2_status, sensor_hub_data.co2_errors);

    return 0;
}

int8_t measure_scd4x(void)
{
    scd4x_status = scd41_measure_single_shot(&scd4x_device);
    ESP_LOGI(TAG, "[SCD4x] Measure Single Shot, status = %d", scd4x_status);
    RETURN_IF_NOT_ZERO((int8_t)scd4x_status, sensor_hub_data.co2_status, sensor_hub_data.co2_errors);

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    uint8_t retry_count = 0;
    while (true)
    {
        scd4x_status = scd4x_get_data_ready_status(&scd4x_device, &scd4x_data_ready);
        ESP_LOGI(TAG, "[SCD4x] Read Data Ready Status, data ready = %d, status = %d", scd4x_data_ready, scd4x_status);
        RETURN_IF_NOT_ZERO((int8_t)scd4x_status, sensor_hub_data.co2_status, sensor_hub_data.co2_errors);

        if (scd4x_data_ready)
        {
            break;
        }
        else if (retry_count == SCD4X_MAX_RETRY_COUNT)
        {
            ESP_LOGE(TAG, "[SCD4x] Data not ready after %d retries", retry_count);
            sensor_hub_data.co2_errors++;
            return -10;
        }
        retry_count++;
        vTaskDelay(SCD4X_RETRY_PERIOD_MS / portTICK_PERIOD_MS);
    }

    scd4x_data_ready = false;
    scd4x_status = scd4x_read_measurement(&scd4x_device, &scd4x_data);
    ESP_LOGI(TAG, "[SCD4x] Read Data, CO2 concentration = %d ppm, Temperature = %.2f °C, Rel. humidity = %.2f %% status = %d",
             scd4x_data.co2_ppm, scd4x_data.temperature / 100.0, scd4x_data.relative_humidity / 100.0, scd4x_status);
    RETURN_IF_NOT_ZERO((int8_t)scd4x_status, sensor_hub_data.co2_status, sensor_hub_data.co2_errors);

    sensor_hub_data.co2 = scd4x_data.co2_ppm;
    sensor_hub_data.co2_status = scd4x_status;
    gettimeofday(&current_time, NULL);
    sensor_hub_data.co2_timestamp = current_time;
    sensor_hub_data.co2_measurements++;

    return 0;
}

int8_t setup_sps30(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &sps30_config, &sps30_device_handle));

    sps30_status = sps30_read_product_type(&sps30_device, sps30_product_type);
    ESP_LOGI(TAG, "[SPS30] Read Product Type, product type = %s, status = %d", sps30_product_type, sps30_status);
    RETURN_IF_NOT_ZERO((int8_t)sps30_status, sensor_hub_data.pm_status, sensor_hub_data.pm_errors);

    sps30_status = sps30_read_serial_number(&sps30_device, sps30_serial_number);
    ESP_LOGI(TAG, "[SPS30] Read Serial Number, serial number = %s, status = %d", sps30_serial_number, sps30_status);
    RETURN_IF_NOT_ZERO((int8_t)sps30_status, sensor_hub_data.pm_status, sensor_hub_data.pm_errors);

    sps30_status = sps30_read_firmware_version(&sps30_device, &sps30_version);
    ESP_LOGI(TAG, "[SPS30] Read Firmware Version, version = %d.%d, status = %d", sps30_version.major, sps30_version.minor, sps30_status);
    RETURN_IF_NOT_ZERO((int8_t)sps30_status, sensor_hub_data.pm_status, sensor_hub_data.pm_errors);

    sps30_status = sps30_read_device_status_flags(&sps30_device, &sps30_flags);
    ESP_LOGI(TAG, "[SPS30] Read Device Status Flags, speed warning = %d, laser error = %d, fan error = %d, status = %d",
             sps30_flags.speed_warning, sps30_flags.laser_error, sps30_flags.fan_error, sps30_status);
    RETURN_IF_NOT_ZERO((int8_t)sps30_status, sensor_hub_data.pm_status, sensor_hub_data.pm_errors);

    sps30_status = sps30_start_measurement(&sps30_device, SPS30_FLOAT);
    ESP_LOGI(TAG, "[SPS30] Start Measurement (float), status = %d", sps30_status);
    RETURN_IF_NOT_ZERO((int8_t)sps30_status, sensor_hub_data.pm_status, sensor_hub_data.pm_errors);

    return 0;
}

int8_t measure_sps30(void)
{
    uint8_t retry_count = 0;
    while (true)
    {
        sps30_status = sps30_read_data_ready_flag(&sps30_device, &sps30_data_ready);
        ESP_LOGI(TAG, "[SPS30] Read Data Ready Flag, data ready = %d, status = %d", sps30_data_ready, sps30_status);
        RETURN_IF_NOT_ZERO((int8_t)sps30_status, sensor_hub_data.pm_status, sensor_hub_data.pm_errors);

        if (sps30_data_ready)
        {
            break;
        }
        else if (retry_count == SPS30_MAX_RETRY_COUNT)
        {
            ESP_LOGE(TAG, "[SPS30] Data not ready after %d retries", retry_count);
            return -10;
        }
        retry_count++;
        vTaskDelay(SPS30_RETRY_PERIOD_MS / portTICK_PERIOD_MS);
    }

    sps30_status = sps30_read_measured_values_float(&sps30_device, &sps30_float_data);
    ESP_LOGI(TAG, "[SPS30] Read Float Data, PM1.0 = %.2f ug/m^3, PM2.5 = %.2f ug/m^3, "
                  "PM4.0 = %.2f ug/m^3, PM10.0 = %.2f ug/m^3, typ. particle = %.2f nm, status = %d",
             sps30_float_data.mass_concentration_pm1_0,
             sps30_float_data.mass_concentration_pm2_5,
             sps30_float_data.mass_concentration_pm4_0,
             sps30_float_data.mass_concentration_pm10_0,
             sps30_float_data.typical_particle_size, sps30_status);
    sps30_data_ready = false;
    RETURN_IF_NOT_ZERO((int8_t)sps30_status, sensor_hub_data.pm_status, sensor_hub_data.pm_errors);

    sensor_hub_data.pm_1_0 = sps30_float_data.mass_concentration_pm1_0;
    sensor_hub_data.pm_2_5 = sps30_float_data.mass_concentration_pm2_5;
    sensor_hub_data.pm_4_0 = sps30_float_data.mass_concentration_pm4_0;
    sensor_hub_data.pm_10_0 = sps30_float_data.mass_concentration_pm10_0;
    sensor_hub_data.pm_typical_size = sps30_float_data.typical_particle_size;
    gettimeofday(&current_time, NULL);
    sensor_hub_data.pm_timestamp = current_time;
    sensor_hub_data.pm_measurements++;

    return 0;
}

static int8_t sht4x_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_transmit(sht4x_device_handle, payload, length, 20);
    return err == ESP_OK ? 0 : -1;
}

static int8_t sht4x_i2c_read(uint8_t address, uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_receive(sht4x_device_handle, payload, length, 20);
    return err == ESP_OK ? 0 : -1;
}

static int8_t sgp41_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length)
{

    (void)address; // Address not necessary
    esp_err_t err = i2c_master_transmit(sgp41_device_handle, payload, length, 20);
    return err == ESP_OK ? 0 : -1;
}

static int8_t sgp41_i2c_read(uint8_t address, uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_receive(sgp41_device_handle, payload, length, 20);
    return err == ESP_OK ? 0 : -1;
}

static int8_t sgp41_delay_ms(uint16_t ms)
{
    if (ms < 10)
        ms = 10; // Minimum 10 ms delay due to FreeRTOS tick rate being 100 Hz

    vTaskDelay(ms / portTICK_PERIOD_MS);
    return 0;
}

static int8_t bme280_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_transmit(bme280_device_handle, payload, length, 20);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return err == ESP_OK ? 0 : -1;
}

static int8_t bme280_i2c_read(uint8_t address, uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_receive(bme280_device_handle, payload, length, 20);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return err == ESP_OK ? 0 : -1;
}

static int8_t scd4x_i2c_write(uint8_t address, const uint8_t *payload, size_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_transmit(scd4x_device_handle, payload, length, 20);
    return err == ESP_OK ? 0 : -1;
}

static int8_t scd4x_i2c_read(uint8_t address, uint8_t *payload, size_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_receive(scd4x_device_handle, payload, length, 20);
    return err == ESP_OK ? 0 : -1;
}

static int8_t scd4x_delay_ms(uint16_t ms)
{
    if (ms < 10)
        ms = 10; // Minimum 10 ms delay due to FreeRTOS tick rate being 100 Hz

    vTaskDelay(ms / portTICK_PERIOD_MS);
    return 0;
}

static int8_t sps30_i2c_write(uint8_t address, const uint8_t *payload, uint8_t length)
{

    (void)address; // Address not necessary
    esp_err_t err = i2c_master_transmit(sps30_device_handle, payload, length, 20);
    return err == ESP_OK ? 0 : -1;
}

static int8_t sps30_i2c_read(uint8_t address, uint8_t *payload, uint8_t length)
{
    (void)address; // Address not necessary
    esp_err_t err = i2c_master_receive(sps30_device_handle, payload, length, 20);
    return err == ESP_OK ? 0 : -1;
}

static int8_t sps30_delay_ms(uint16_t ms)
{
    if (ms < 10)
        ms = 10; // Minimum 10 ms delay due to FreeRTOS tick rate being 100 Hz

    vTaskDelay(ms / portTICK_PERIOD_MS);
    return 0;
}