#include <stdint.h> // define uint8_t, uint16_t etc.
#include <stddef.h> // define NULL
#include <string.h> // define memcpy

#include "sps30_i2c.h"

/* --- Private constants --- */

#define SPS30_I2C_CMD_LENGTH 2                   // bytes
#define SPS30_I2C_CMD_START_MEASUREMENT_LENGTH 5 // bytes
#define SPS30_I2C_PRODUCT_TYPE_LENGTH 12         // bytes
#define SPS30_I2C_SERIAL_NUMBER_LENGTH 48        // bytes
#define SPS30_I2C_FIRMWARE_LENGTH 3              // bytes
#define SPS30_I2C_STATUS_FLAGS_LENGTH 6          // bytes
#define SPS30_I2C_FLOAT_DATA_LENGTH 60           // bytes
#define SPS30_I2C_UINT16_DATA_LENGTH 30          // bytes
#define SPS30_CRC8_POLYNOMIAL 0x31               // x^8 + x^5 + x^4 + 1, initialization to 0xFF
#define SPS30_PRODUCT_TYPE_LENGTH 8              // chars
#define SPS30_SERIAL_NUMBER_LENGTH 32            // chars

#define SPS30_I2C_FLOAT 0x03, 0x00, 0xAC  // big-endian IEEE754 float
#define SPS30_I2C_UINT16 0x05, 0x00, 0xF6 // big-endian unsigned 16-bit integer

#define SPS30_I2C_CMD_START_MEASUREMENT 0x00, 0x10
#define SPS30_I2C_CMD_START_MEASUREMENT_FLOAT SPS30_I2C_CMD_START_MEASUREMENT, SPS30_I2C_FLOAT
#define SPS30_I2C_CMD_START_MEASUREMENT_UINT16 SPS30_I2C_CMD_START_MEASUREMENT, SPS30_I2C_UINT16
#define SPS30_I2C_CMD_STOP_MEASUREMENT 0x01, 0x04
#define SPS30_I2C_CMD_READ_DATA_READY_FLAG 0x02, 0x02
#define SPS30_I2C_CMD_READ_MEASURED_VALUES 0x03, 0x00
#define SPS30_I2C_CMD_SLEEP 0x10, 0x01  // version >= 2.0
#define SPS30_I2C_CMD_WAKEUP 0x11, 0x03 // version >= 2.0
#define SPS30_I2C_CMD_START_FAN_CLEANING 0x56, 0x07
#define SPS30_I2C_CMD_READ_WRITE_AUTO_CLEANING_INTERVAL 0x80, 0x04
#define SPS30_I2C_CMD_READ_PRODUCT_TYPE 0xD0, 0x02
#define SPS30_I2C_CMD_READ_SERIAL_NUMBER 0xD0, 0x33
#define SPS30_I2C_CMD_READ_VERSION 0xD1, 0x00
#define SPS30_I2C_CMD_READ_DEVICE_STATUS_REGISTER 0xD2, 0x06  // version >= 2.2
#define SPS30_I2C_CMD_CLEAR_DEVICE_STATUS_REGISTER 0xD2, 0x10 // version >= 2.0
#define SPS30_I2C_CMD_RESET 0xD3, 0x04

/* --- Private macros --- */

/**
 * Error-checking macro: if `expr` is not `SPS30_SUCCESS`, this macro returns `expr`,
 * exiting the function where this macro was used immediately.
 */
#define SPS30_CHECK_STATUS(expr)     \
    do                               \
    {                                \
        Sps30Status retval = expr;   \
        if (retval != SPS30_SUCCESS) \
        {                            \
            return retval;           \
        }                            \
    } while (0)

/**
 * Error-checking macro: if `expr` is `NULL`, this macro returns `SPS30_POINTER_NULL`,
 * exiting the function where this macro was used immediately.
 */
#define SPS30_CHECK_NULL(expr)         \
    do                                 \
    {                                  \
        if (expr == NULL)              \
        {                              \
            return SPS30_POINTER_NULL; \
        }                              \
    } while (0)

/* --- Private function prototypes --- */

static Sps30Status sps30_check_checksum(Sps30Device *device, uint8_t data[2], uint8_t checksum);
static Sps30Status sps30_calculate_checksum(Sps30Device *device, uint8_t data[2], uint8_t *checksum);
static uint8_t sps30_calculate_crc8(uint8_t data[2]);
static Sps30Status sps30_check_device(Sps30Device *device);
static float sps30_convert_bytes_to_float(uint8_t bytes[4]);

/* --- Function definitions --- */

/**
 * @brief Starts the measurement on the SPS30 sensor.
 *
 * Depending on the `data_format` parameter, use either `sps30_read_measured_values_float`
 * or `sps30_read_measured_values_uint16` functions to read the measured values.
 *
 * Data will be available after 1 second. Check availability of data using
 * `sps30_read_data_ready_flag` function.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 * @param[in] data_format The data format to use for the measurement. The
 * possible values are `SPS30_FLOAT` and `SPS30_UINT16`.
 *
 * @retval `SPS30_SUCCESS` Command sent successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_POINTER_NULL` `device` pointer is `NULL`.
 */
Sps30Status sps30_start_measurement(Sps30Device *device, Sps30DataFormat data_format)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));

    uint8_t tx_data[SPS30_I2C_CMD_START_MEASUREMENT_LENGTH];
    if (data_format == SPS30_FLOAT)
    {
        uint8_t cmd[] = {SPS30_I2C_CMD_START_MEASUREMENT_FLOAT};
        memcpy(tx_data, cmd, sizeof(cmd));
    }
    else
    {
        uint8_t cmd[] = {SPS30_I2C_CMD_START_MEASUREMENT_UINT16};
        memcpy(tx_data, cmd, sizeof(cmd));
    }

    if (device->i2c_write(SPS30_I2C_ADDRESS, tx_data, SPS30_I2C_CMD_START_MEASUREMENT_LENGTH) != 0)
        return SPS30_I2C_ERROR;
    return SPS30_SUCCESS;
}

/**
 * @brief Stops the measurement on the SPS30 sensor.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 *
 * @retval `SPS30_SUCCESS` Command sent successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_POINTER_NULL` `device` pointer is `NULL`.
 */
Sps30Status sps30_stop_measurement(Sps30Device *device)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_STOP_MEASUREMENT}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;
    return SPS30_SUCCESS;
}

/**
 * @brief Reads the data ready flag from the SPS30 sensor.
 *
 * This function verifies the received checksum to ensure data integrity.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 * @param[out] data_ready A pointer to a boolean where the data ready flag is stored.
 * It will be set to `true` if new data is ready, or `false` otherwise.
 *
 * @retval `SPS30_SUCCESS` Data ready flag read and checksum verified successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_CRC_FAILURE` Checksum verification failed.
 * @retval `SPS30_POINTER_NULL` `device` or `data_ready` pointer is `NULL`.
 */
Sps30Status sps30_read_data_ready_flag(Sps30Device *device, bool *data_ready)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));
    SPS30_CHECK_NULL(data_ready);

    device->delay_ms(1);

    uint8_t rx_data[3];
    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_READ_DATA_READY_FLAG}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    if (device->i2c_read(SPS30_I2C_ADDRESS, rx_data, 3) != 0)
        return SPS30_I2C_ERROR;

    SPS30_CHECK_STATUS(sps30_check_checksum(device, (uint8_t[]){rx_data[0], rx_data[1]}, rx_data[2]));

    *data_ready = (rx_data[1] == 0x01) ? true : false;
    return SPS30_SUCCESS;
}

/**
 * @brief Reads the measured values from the SPS30 sensor in floating-point format.
 *
 * The function checks the returned checksums to ensure data integrity. This function call
 * must be preceded by a call to `sps30_start_measurement` with `SPS30_FLOAT` as the data format.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 * @param[out] data A pointer to a `Sps30FloatData` struct where the measured
 * values are stored.
 *
 * @retval `SPS30_SUCCESS` Measured values read and checksums verified successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_CRC_FAILURE` Checksum verification failed.
 * @retval `SPS30_POINTER_NULL` `device` or `data` pointer is `NULL`.
 */
Sps30Status sps30_read_measured_values_float(Sps30Device *device, Sps30FloatData *data)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));
    SPS30_CHECK_NULL(data);

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_READ_MEASURED_VALUES}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    device->delay_ms(1);

    uint8_t rx_data[SPS30_I2C_FLOAT_DATA_LENGTH];

    if (device->i2c_read(SPS30_I2C_ADDRESS, rx_data, SPS30_I2C_FLOAT_DATA_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    for (uint8_t i = 0; i < SPS30_I2C_FLOAT_DATA_LENGTH; i = i + 3)
    {
        // Every third bytes is a CRC8 checksum of two previous bytes
        SPS30_CHECK_STATUS(sps30_check_checksum(device, (uint8_t[]){rx_data[i], rx_data[i + 1]}, rx_data[i + 2]));
    }

    float parsed_data[10];

    for (uint8_t i = 0; i < 10; i++)
    {
        // Every 6 bytes is a single float (MSB, MSB-1, CRC8, LSB+1, LSB, CRC8)
        parsed_data[i] = sps30_convert_bytes_to_float((uint8_t[]){rx_data[6 * i], rx_data[6 * i + 1],
                                                                  rx_data[6 * i + 3], rx_data[6 * i + 4]});
    }

    data->mass_concentration_pm1_0 = parsed_data[0];
    data->mass_concentration_pm2_5 = parsed_data[1];
    data->mass_concentration_pm4_0 = parsed_data[2];
    data->mass_concentration_pm10_0 = parsed_data[3];
    data->number_concentration_pm0_5 = parsed_data[4];
    data->number_concentration_pm1_0 = parsed_data[5];
    data->number_concentration_pm2_5 = parsed_data[6];
    data->number_concentration_pm4_0 = parsed_data[7];
    data->number_concentration_pm10_0 = parsed_data[8];
    data->typical_particle_size = parsed_data[9];

    return SPS30_SUCCESS;
}

/**
 * @brief Reads the measured values from the SPS30 sensor in unsigned 16-bit integer format.
 *
 * The function checks the returned checksums to ensure data integrity. This function call
 * must be preceded by a call to `sps30_start_measurement` with `SPS30_UINT16` as the data format.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 * @param[out] data A pointer to a `Sps30Uint16Data` struct where the measured
 * values are stored.
 *
 * @retval `SPS30_SUCCESS` Measured values read and checksums verified successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_CRC_FAILURE` Checksum verification failed.
 * @retval `SPS30_POINTER_NULL` `device` or `data` pointer is `NULL`.
 */
Sps30Status sps30_read_measured_values_uint16(Sps30Device *device, Sps30Uint16Data *data)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));
    SPS30_CHECK_NULL(data);

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_READ_MEASURED_VALUES}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    device->delay_ms(1);

    uint8_t rx_data[SPS30_I2C_UINT16_DATA_LENGTH];
    if (device->i2c_read(SPS30_I2C_ADDRESS, rx_data, SPS30_I2C_UINT16_DATA_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    for (uint8_t i = 0; i < SPS30_I2C_UINT16_DATA_LENGTH; i = i + 3)
    {
        // Every third bytes is a CRC8 checksum of two previous bytes
        SPS30_CHECK_STATUS(sps30_check_checksum(device, (uint8_t[]){rx_data[i], rx_data[i + 1]}, rx_data[i + 2]));
    }

    uint16_t parsed_data[10];

    for (uint8_t i = 0; i < 10; i++)
    {
        // Every 3 bytes is a single uint16_t (MSB, LSB, CRC8)
        parsed_data[i] = ((uint16_t)rx_data[3 * i] << 8) | (rx_data[3 * i + 1]);
    }

    data->mass_concentration_pm1_0 = parsed_data[0];
    data->mass_concentration_pm2_5 = parsed_data[1];
    data->mass_concentration_pm4_0 = parsed_data[2];
    data->mass_concentration_pm10_0 = parsed_data[3];
    data->number_concentration_pm0_5 = parsed_data[4];
    data->number_concentration_pm1_0 = parsed_data[5];
    data->number_concentration_pm2_5 = parsed_data[6];
    data->number_concentration_pm4_0 = parsed_data[7];
    data->number_concentration_pm10_0 = parsed_data[8];
    data->typical_particle_size = parsed_data[9];

    return SPS30_SUCCESS;
}

/**
 * @brief Put the SPS30 into sleep mode.
 *
 * This function puts the SPS30 into sleep mode to conserve power. The sensor can be woken up with
 * `sps30_wake_up`.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication functions.
 *
 * @retval `SPS30_SUCCESS` Sleep command sent successfully.
 * @retval `SPS30_POINTER_NULL` `device` pointer is `NULL`.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 */
Sps30Status sps30_sleep(Sps30Device *device)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_SLEEP}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;
    return SPS30_SUCCESS;
}

/**
 * @brief Wake up the SPS30 sensor from sleep mode.
 *
 * The device is normally in sleep mode. The device can be woken up by sending
 * this command. The device will then start to measure particulate matter
 * concentrations.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 *
 * @retval `SPS30_SUCCESS` Command sent successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_POINTER_NULL` `device` pointer is `NULL`.
 */
Sps30Status sps30_wake_up(Sps30Device *device)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));

    // Send wake-up command twice (first command enables I2C interface, but is ignored)
    for (uint8_t i = 0; i < 2; i++)
    {
        if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_WAKEUP}, SPS30_I2C_CMD_LENGTH) != 0)
            return SPS30_I2C_ERROR;
    }
    return SPS30_SUCCESS;
}

/**
 * @brief Initiates the fan cleaning process on the SPS30 sensor.
 *
 * Fan cleaning helps maintain the sensor's performance by removing
 * dust and particles from the internal fan blades. The fan will be accelerated to maximum speed for 10 seconds.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 *
 * @retval `SPS30_SUCCESS` Command sent successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_POINTER_NULL` `device` pointer is `NULL`.
 */
Sps30Status sps30_start_fan_cleaning(Sps30Device *device)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_START_FAN_CLEANING}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;
    return SPS30_SUCCESS;
}

/**
 * @brief Reads the auto cleaning interval from the SPS30 sensor and verifies the checksum.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 * @param[out] interval A pointer to a `uint32_t` where the auto cleaning
 * interval is stored in seconds.
 *
 * @retval `SPS30_SUCCESS` Auto cleaning interval read and checksum verified successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_CRC_FAILURE` Checksum verification failed.
 * @retval `SPS30_POINTER_NULL` `device` or `interval` pointer is `NULL`.
 */
Sps30Status sps30_read_auto_cleaning_interval(Sps30Device *device, uint32_t *interval)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));
    SPS30_CHECK_NULL(interval);

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_READ_WRITE_AUTO_CLEANING_INTERVAL}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    device->delay_ms(1);

    uint8_t rx_data[6];
    if (device->i2c_read(SPS30_I2C_ADDRESS, rx_data, 6) != 0)
        return SPS30_I2C_ERROR;

    SPS30_CHECK_STATUS(sps30_check_checksum(device, (uint8_t[]){rx_data[0], rx_data[1]}, rx_data[2]));
    SPS30_CHECK_STATUS(sps30_check_checksum(device, (uint8_t[]){rx_data[3], rx_data[4]}, rx_data[5]));

    *interval = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[3] << 8) | rx_data[4];

    return SPS30_SUCCESS;
}

/**
 * @brief Sets the auto cleaning interval for the SPS30 sensor.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 * @param[in] interval The auto cleaning interval in seconds.
 *
 * @retval `SPS30_SUCCESS` Auto cleaning interval command sent successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_CRC_FAILURE` Checksum calculation failed.
 * @retval `SPS30_POINTER_NULL` `device` or `interval` pointer is `NULL`.
 */
Sps30Status sps30_set_auto_cleaning_interval(Sps30Device *device, uint32_t interval)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));

    uint8_t tx_data[6];
    tx_data[0] = (interval >> 24) & 0xFF;
    tx_data[1] = (interval >> 16) & 0xFF;
    tx_data[3] = (interval >> 8) & 0xFF;
    tx_data[4] = interval & 0xFF;

    SPS30_CHECK_STATUS(sps30_calculate_checksum(device, (uint8_t[]){tx_data[0], tx_data[1]}, &tx_data[2]));
    SPS30_CHECK_STATUS(sps30_calculate_checksum(device, (uint8_t[]){tx_data[3], tx_data[4]}, &tx_data[5]));

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_READ_WRITE_AUTO_CLEANING_INTERVAL}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    if (device->i2c_write(SPS30_I2C_ADDRESS, tx_data, 6) != 0)
        return SPS30_I2C_ERROR;

    return SPS30_SUCCESS;
}

/**
 * @brief Reads the product type from the SPS30 sensor.
 *
 * The product type is an 8-byte array of ASCII characters. The product type is null-terminated.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 * @param[out] product_type A pointer to a `char` array of size 8 where the
 * product type is stored.
 *
 * @retval `SPS30_SUCCESS` Product type read and checksums verified successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_CRC_FAILURE` Checksum verification failed.
 * @retval `SPS30_POINTER_NULL` `device` or `product_type` pointer is `NULL`.
 */
Sps30Status sps30_read_product_type(Sps30Device *device, char *product_type)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));
    SPS30_CHECK_NULL(product_type);

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_READ_PRODUCT_TYPE}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    device->delay_ms(1);

    uint8_t rx_data[SPS30_I2C_PRODUCT_TYPE_LENGTH];
    if (device->i2c_read(SPS30_I2C_ADDRESS, rx_data, SPS30_I2C_PRODUCT_TYPE_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    for (uint8_t i = 0; i < SPS30_I2C_PRODUCT_TYPE_LENGTH; i = i + 3)
    {
        // Every 3rd byte is a CRC8 checksum of two previous bytes
        SPS30_CHECK_STATUS(sps30_check_checksum(device, (uint8_t[]){rx_data[i], rx_data[i + 1]}, rx_data[i + 2]));
    }

    // Copy the product type to the output buffer, skipping the checksum bytes
    uint8_t input_index = 0, output_index = 0;
    while (output_index < SPS30_PRODUCT_TYPE_LENGTH)
    {
        if (input_index % 3 != 2)
        {
            product_type[output_index] = rx_data[input_index];
            output_index++;
        }
        input_index++;
    }

    return SPS30_SUCCESS;
}

/**
 * @brief Reads the serial number from the SPS30 sensor.
 *
 * The serial number is a 32-byte array of ASCII characters.
 * The function ensures data integrity by verifying the checksums of the received data.
 * The serial number is null-terminated.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 * @param[out] serial_number A pointer to a `char` array of size 32 where the
 * serial number is stored.
 *
 * @retval `SPS30_SUCCESS` Serial number read and checksums verified successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_CRC_FAILURE` Checksum verification failed.
 * @retval `SPS30_POINTER_NULL` `device` or `serial_number` pointer is `NULL`.
 */
Sps30Status sps30_read_serial_number(Sps30Device *device, char *serial_number)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));
    SPS30_CHECK_NULL(serial_number);

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_READ_SERIAL_NUMBER}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    device->delay_ms(1);

    uint8_t rx_data[SPS30_I2C_SERIAL_NUMBER_LENGTH];
    if (device->i2c_read(SPS30_I2C_ADDRESS, rx_data, SPS30_I2C_SERIAL_NUMBER_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    for (uint8_t i = 0; i < SPS30_I2C_SERIAL_NUMBER_LENGTH; i = i + 3)
    {
        // Every 3rd byte is a CRC8 checksum of two previous bytes
        SPS30_CHECK_STATUS(sps30_check_checksum(device, (uint8_t[]){rx_data[i], rx_data[i + 1]}, rx_data[i + 2]));
    }

    // Copy the serial number to the output buffer, skipping the checksum bytes
    uint8_t input_index = 0, output_index = 0;
    while (output_index < SPS30_SERIAL_NUMBER_LENGTH)
    {
        if (input_index % 3 != 2)
        {
            serial_number[output_index] = rx_data[input_index];
            output_index++;
        }
        input_index++;
    }

    return SPS30_SUCCESS;
}

/**
 * @brief Reads the firmware version from the SPS30 device and verifies its checksum.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 * @param[out] version A pointer to a `Sps30FirmwareVersion` struct where the
 * read firmware version is stored.
 *
 * @retval `SPS30_SUCCESS` Firmware version read and verified successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error.
 * @retval `SPS30_CRC_FAILURE` Firmware version checksum verification failed.
 * @retval `SPS30_POINTER_NULL` `device` or `version` pointer is `NULL`.
 */
Sps30Status sps30_read_firmware_version(Sps30Device *device, Sps30FirmwareVersion *version)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));
    SPS30_CHECK_NULL(version);

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_READ_VERSION}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    device->delay_ms(1);

    uint8_t rx_data[SPS30_I2C_FIRMWARE_LENGTH];
    if (device->i2c_read(SPS30_I2C_ADDRESS, rx_data, SPS30_I2C_FIRMWARE_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    SPS30_CHECK_STATUS(sps30_check_checksum(device, (uint8_t[]){rx_data[0], rx_data[1]}, rx_data[2]));

    version->major = rx_data[0];
    version->minor = rx_data[1];

    return SPS30_SUCCESS;
}

/**
 * @brief Reads the device status flags from the SPS30 sensor and verifies its checksum.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 * @param[out] status_flags A pointer to a `Sps30StatusFlags` struct where the
 * read status flags are stored.
 *
 * @retval `SPS30_SUCCESS` Status flags read and checksums verified successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_CRC_FAILURE` Status flags checksum verification failed.
 * @retval `SPS30_POINTER_NULL` `device` or `status_flags` pointer is `NULL`.
 */
Sps30Status sps30_read_device_status_flags(Sps30Device *device, Sps30StatusFlags *status_flags)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));
    SPS30_CHECK_NULL(status_flags);

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_READ_DEVICE_STATUS_REGISTER}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    device->delay_ms(1);

    uint8_t rx_data[SPS30_I2C_STATUS_FLAGS_LENGTH];
    if (device->i2c_read(SPS30_I2C_ADDRESS, rx_data, SPS30_I2C_STATUS_FLAGS_LENGTH) != 0)
        return SPS30_I2C_ERROR;

    for (uint8_t i = 0; i < SPS30_I2C_STATUS_FLAGS_LENGTH; i = i + 3)
    {
        // Every 3rd byte is a CRC8 checksum of two previous bytes
        SPS30_CHECK_STATUS(sps30_check_checksum(device, (uint8_t[]){rx_data[i], rx_data[i + 1]}, rx_data[i + 2]));
    }

    uint32_t status_register = ((uint32_t)rx_data[0] << 24) | ((uint32_t)rx_data[1] << 16) | ((uint32_t)rx_data[3] << 8) | rx_data[4];

    status_flags->fan_error = status_register & (1 << 4);
    status_flags->laser_error = status_register & (1 << 5);
    status_flags->speed_warning = status_register & (1 << 21);

    return SPS30_SUCCESS;
}

/**
 * @brief Clears the device status flags of the SPS30 sensor.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 *
 * @retval `SPS30_SUCCESS` Device status flags cleared successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_POINTER_NULL` `device` pointer is `NULL`.
 */
Sps30Status sps30_clear_device_status_flags(Sps30Device *device)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_CLEAR_DEVICE_STATUS_REGISTER}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;
    return SPS30_SUCCESS;
}

/**
 * @brief Resets the SPS30 device.
 *
 * After calling this command, the module is in the same state as after a power reset.
 *
 * To perform a reset when the sensor is in sleep mode, it is required to send first a wake-up sequence to activate
 * the interface.
 *
 * @param[in] device The `Sps30Device` struct containing the I2C communication
 * functions.
 *
 * @retval `SPS30_SUCCESS` Command sent successfully.
 * @retval `SPS30_I2C_ERROR` I2C communication error occurred.
 * @retval `SPS30_POINTER_NULL` `device` pointer is `NULL`.
 */
Sps30Status sps30_reset(Sps30Device *device)
{
    SPS30_CHECK_STATUS(sps30_check_device(device));

    if (device->i2c_write(SPS30_I2C_ADDRESS, (uint8_t[]){SPS30_I2C_CMD_RESET}, SPS30_I2C_CMD_LENGTH) != 0)
        return SPS30_I2C_ERROR;
    return SPS30_SUCCESS;
}

/**
 * @brief Verifies the checksum for a given data array using the SPS30 device's CRC function.
 *
 * This function calculates the CRC-8 checksum of the provided 2-byte data array and compares it
 * with the given checksum. If the SPS30 device has a custom CRC calculation function defined, it uses that function.
 * Otherwise, it falls back to the default software CRC-8 algorithm.
 *
 * @param[in] device The `Sps30Device` struct containing the CRC calculation function.
 * @param[in] data A 2-byte array for which the checksum is to be verified.
 * @param[in] checksum A `uint8_t` containing the expected checksum.
 *
 * @retval `SPS30_SUCCESS` Checksum verified successfully.
 * @retval `SPS30_CRC_FAILURE` CRC verification using the device's function failed.
 */
static Sps30Status sps30_check_checksum(Sps30Device *device, uint8_t data[2], uint8_t checksum)
{
    uint8_t calculated_checksum = 0;
    SPS30_CHECK_STATUS(sps30_calculate_checksum(device, data, &calculated_checksum));

    if (calculated_checksum != checksum)
        return SPS30_CRC_FAILURE;

    return SPS30_SUCCESS;
}

/**
 * @brief Calculates the checksum for a given data array using the SPS30 device's CRC function.
 *
 * This function calculates the CRC-8 checksum of the provided 2-byte data array.
 * If the SPS30 device has a custom CRC calculation function defined, it uses that function.
 * Otherwise, it falls back to the default software CRC-8 algorithm.
 *
 * @param[in] device The `Sps30Device` struct containing the CRC calculation function.
 * @param[in] data A 2-byte array for which the checksum is to be calculated.
 * @param[out] checksum A pointer to a `uint8_t` where the calculated checksum is stored.
 *
 * @retval `SPS30_SUCCESS` Checksum calculated successfully.
 * @retval `SPS30_CRC_FAILURE` CRC calculation using the device's function failed.
 */
static Sps30Status sps30_calculate_checksum(Sps30Device *device, uint8_t data[2], uint8_t *checksum)
{
    if (device->calculate_crc != NULL)
    {
        if (device->calculate_crc(data, 2, SPS30_CRC8_POLYNOMIAL, checksum) != 0)
            return SPS30_CRC_FAILURE;
    }
    else
        *checksum = sps30_calculate_crc8(data);

    return SPS30_SUCCESS;
}

/**
 * @brief Calculates the CRC-8 checksum of two bytes of data.
 *
 * This function implements the CRC-8 algorithm as specified in the SPS30
 * datasheet. It takes two bytes of data and returns the calculated CRC-8
 * checksum.
 *
 * @param[in] data Two bytes of data to calculate the CRC-8 checksum for.
 *
 * @return The calculated CRC-8 checksum.
 */
static uint8_t sps30_calculate_crc8(uint8_t data[2])
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ SPS30_CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Checks whether the provided `Sps30Device` struct contains valid pointers.
 *
 * Returns -1 if the `device`pointer is NULL, or if the `i2c_write` or `i2c_read`
 * function pointers are `NULL`.
 *
 * WARNING: This function does not check whether the function pointers are
 * pointing to valid functions.
 *
 * @param[in] device The `Sps30Device` struct to be checked.
 *
 * @retval `SPS30_SUCCESS` The `Sps30Device` struct pointers are valid.
 * @retval `SPS30_POINTER_NULL` One of the pointers is `NULL`.
 */
static Sps30Status sps30_check_device(Sps30Device *device)
{
    if (device == NULL)
        return SPS30_POINTER_NULL;
    if (device->i2c_write == NULL || device->i2c_read == NULL)
        return SPS30_POINTER_NULL;
    return SPS30_SUCCESS;
}

/**
 * @brief Converts a 4-byte array to a floating-point number.
 *
 * This function takes a 4-byte array interpreted as a big-endian
 * IEEE 754 floating-point number and converts it to a `float`.
 *
 * @param[in] bytes A 4-byte array representing a float in big-endian order.
 *
 * @return The converted floating-point value.
 */
static float sps30_convert_bytes_to_float(uint8_t bytes[4])
{
    uint32_t value = bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3];
    union
    {
        uint32_t i;
        float f;
    } converter;
    converter.i = value;
    return converter.f;
}