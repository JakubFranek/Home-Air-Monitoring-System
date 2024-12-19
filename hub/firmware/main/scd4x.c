#include <stdint.h>  // define uint8_t, uint16_t etc.
#include <stddef.h>  // define NULL
#include <string.h>  // define memcpy
#include <stdbool.h> // define bool

#include "scd4x.h"

/* --- Private constants --- */

#define SCD4X_I2C_CMD_LENGTH 2
#define SCD4X_CRC8_POLYNOMIAL 0x31
#define SCD4X_CRC8_INITIAL_VALUE 0xFF
#define SCD4X_MEASUREMENT_LENGTH 9
#define SCD4X_SERIAL_NUMBER_LENGTH 9
#define SCD4X_SETTING_LENGTH 3

/* --- SCD4x commands --- */

#define SCD4X_CMD_START_PERIODIC_MEASUREMENT 0x21, 0xB1
#define SCD4X_CMD_READ_MEASUREMENT 0xEC, 0x05
#define SCD4X_CMD_STOP_PERIODIC_MEASUREMENT 0x3F, 0x86
#define SCD4X_CMD_SET_TEMPERATURE_OFFSET 0x24, 0x1D
#define SCD4X_CMD_GET_TEMPERATURE_OFFSET 0x23, 0x18
#define SCD4X_CMD_SET_SENSOR_ALTITUDE 0x24, 0x27
#define SCD4X_CMD_GET_SENSOR_ALTITUDE 0x23, 0x22
#define SCD4X_CMD_GET_SET_AMBIENT_PRESSURE 0xE0, 0x00
#define SCD4X_CMD_PERFORM_FORCED_RECALIBRATION 0x36, 0x2F
#define SCD4X_CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED 0x24, 0x16
#define SCD4X_CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED 0x23, 0x13
#define SCD4X_CMD_SET_AUTOMATIC_SELF_CALIBRATION_TARGET 0x24, 0x3A
#define SCD4X_CMD_GET_AUTOMATIC_SELF_CALIBRATION_TARGET 0x23, 0x3F
#define SCD4X_CMD_START_LOW_POWER_PERIODIC_MEASUREMENT 0x21, 0xAC
#define SCD4X_CMD_GET_DATA_READY_STATUS 0xE4, 0xB8
#define SCD4X_CMD_PERSIST_SETTINGS 0x36, 0x15
#define SCD4X_CMD_GET_SERIAL_NUMBER 0x36, 0x82
#define SCD4X_CMD_PERFORM_SELF_TEST 0x36, 0x39
#define SCD4X_CMD_PERFORM_FACTORY_RESET 0x36, 0x32
#define SCD4X_CMD_REINIT 0x36, 0x46
#define SCD4X_CMD_GET_SENSOR_VARIANT 0x20, 0x2F

/* --- SCD41-only commands --- */

#define SCD4X_CMD_MEASURE_SINGLE_SHOT 0x21, 0x9D
#define SCD4X_CMD_MEASURE_SINGLE_SHOT_RHT_ONLY 0x21, 0x96
#define SCD4X_CMD_POWER_DOWN 0x36, 0xE0
#define SCD4X_CMD_WAKE_UP 0x36, 0xF6
#define SCD4X_CMD_SET_AUTOMATIC_SELF_CALIBRATION_INITIAL_PERIOD 0x24, 0x45
#define SCD4X_CMD_GET_AUTOMATIC_SELF_CALIBRATION_INITIAL_PERIOD 0x23, 0x40
#define SCD4X_CMD_SET_AUTOMATIC_SELF_CALIBRATION_STANDARD_PERIOD 0x24, 0x4E
#define SCD4X_CMD_GET_AUTOMATIC_SELF_CALIBRATION_STANDARD_PERIOD 0x23, 0x4B

/* --- Private macros --- */

/**
 * Error-checking macro: if `expr` is not `SCD4X_SUCCESS`, this macro returns `expr`,
 * exiting the function where this macro was used immediately.
 */
#define SCD4X_CHECK_STATUS(expr)     \
    do                               \
    {                                \
        Scd4xStatus retval = expr;   \
        if (retval != SCD4X_SUCCESS) \
        {                            \
            return retval;           \
        }                            \
    } while (0)

/**
 * Error-checking macro: if `expr` is `NULL`, this macro returns `SCD4X_POINTER_NULL`,
 * exiting the function where this macro was used immediately.
 */
#define SCD4X_CHECK_NULL(expr)         \
    do                                 \
    {                                  \
        if (expr == NULL)              \
        {                              \
            return SCD4X_POINTER_NULL; \
        }                              \
    } while (0)

/* --- Private function prototypes --- */

static Scd4xStatus scd4x_check_checksum(Scd4xDevice *device, uint8_t data[2], uint8_t checksum);
static Scd4xStatus scd4x_calculate_checksum(Scd4xDevice *device, uint8_t data[2], uint8_t *checksum);
static uint8_t scd4x_calculate_checksum_default(uint8_t data[2]);
static Scd4xStatus scd4x_check_device(Scd4xDevice *device);
static Scd4xStatus scd4x_send_i2c_command(Scd4xDevice *device, uint8_t *command);
static Scd4xStatus scd4x_receive_i2c_data(Scd4xDevice *device, uint8_t *data, uint8_t data_length);

/* --- Function definitions --- */

/**
 * @brief Starts the periodic measurement mode.
 *
 * In this mode, the sensor will continuously take measurements with a 5 second period.
 * The data ready status can be checked using the `get_data_ready_status` function.
 * The measurement data can be read using the `request_measurement` and `read_measurement` functions.
 * The periodic measurement mode can be stopped using the `stop_periodic_measurement` function.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Scd4xStatus scd4x_start_periodic_measurement(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    return scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_START_PERIODIC_MEASUREMENT});
}

/**
 * @brief Reads a measurement from the sensor.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed.
 */
Scd4xStatus scd4x_read_measurement(Scd4xDevice *device, Scd4xData *data)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(data);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_READ_MEASUREMENT}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_MEASUREMENT_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_MEASUREMENT_LENGTH));

    for (int i = 0; i < SCD4X_MEASUREMENT_LENGTH; i = i + 3)
        SCD4X_CHECK_STATUS(scd4x_check_checksum(device, (uint8_t[]){rx_data[i], rx_data[i + 1]}, rx_data[i + 2]));

    data->co2_ppm = rx_data[0] << 8 | rx_data[1];
    data->temperature = ((4375 * (int32_t)(rx_data[3] << 8 | rx_data[4])) >> 14) - 4500;
    data->relative_humidity = ((2500 * (uint32_t)(rx_data[6] << 8 | rx_data[7])) >> 14);

    return SCD4X_SUCCESS;
}

/**
 * @brief Stops the periodic measurement mode.
 *
 * WARNING: Do not issue any commands after calling this function for 500 milliseconds.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Scd4xStatus scd4x_stop_periodic_measurement(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    return scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_STOP_PERIODIC_MEASUREMENT});
}

/**
 * @brief Sets the temperature offset.
 *
 * The temperature offset is added to the internal temperature measurement.
 * Recommended values are between 0 and 20 degrees Celsius. Default value is 4 degrees Celsius.
 * To save the setting to the non-volatile EEPROM, call `scd4x_persist_settings`.
 *
 * WARNING: Do not issue any commands after calling this function for 1 millisecond.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[in] offset The temperature offset to set, in degrees Celsius, of type `float`.
 *
 * @retval `SCD4X_SUCCESS` The offset was set successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation failed.
 */
Scd4xStatus scd4x_set_temperature_offset(Scd4xDevice *device, float offset_degC)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));

    int16_t offset_int = offset_degC * 374; // (2**16 - 1) / 175 = 374.4857
    uint8_t tx_data[SCD4X_SETTING_LENGTH];
    tx_data[0] = offset_int >> 8;
    tx_data[1] = offset_int & 0x00FF;
    SCD4X_CHECK_STATUS(scd4x_calculate_checksum(device, tx_data, &tx_data[2]));

    if (device->i2c_write(SCD4X_I2C_ADDRESS, (uint8_t[]){SCD4X_CMD_SET_TEMPERATURE_OFFSET, tx_data[0], tx_data[1], tx_data[2]},
                          SCD4X_I2C_CMD_LENGTH + SCD4X_SETTING_LENGTH) != 0)
        return SCD4X_I2C_ERROR;
    return SCD4X_SUCCESS;
}

/**
 * @brief Gets the current temperature offset.
 *
 * The temperature offset is added to the internal temperature measurement.
 * The default value is 4 degrees Celsius.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[out] offset The current temperature offset, in degrees Celsius, of type `float`.
 *
 * @retval `SCD4X_SUCCESS` The offset was read successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation failed.
 */
Scd4xStatus scd4x_get_temperature_offset(Scd4xDevice *device, float *offset_degC)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(offset_degC);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_GET_TEMPERATURE_OFFSET}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    int16_t offset_int = rx_data[0] << 8 | rx_data[1];
    *offset_degC = offset_int / 374.4857; // (2**16 - 1) / 175 = 374.4857

    return SCD4X_SUCCESS;
}

/**
 * @brief Sets the current sensor altitude.
 *
 * The altitude is used to correct the measured CO2 concentration. To save the setting to
 * the non-volatile EEPROM, call `scd4x_persist_settings`.
 *
 * WARNING: Do not issue any commands after calling this function for 1 millisecond.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[in] altitude The sensor altitude in meters, of type `uint16_t`.
 * Range is 0 to 3000 meters. Default value is 0 meters.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation failed.
 */
Scd4xStatus scd4x_set_sensor_altitude(Scd4xDevice *device, uint16_t altitude_m)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));

    uint8_t tx_data[SCD4X_SETTING_LENGTH];
    tx_data[0] = altitude_m >> 8;
    tx_data[1] = altitude_m & 0xFF;
    SCD4X_CHECK_STATUS(scd4x_calculate_checksum(device, tx_data, &tx_data[2]));

    if (device->i2c_write(SCD4X_I2C_ADDRESS, (uint8_t[]){SCD4X_CMD_SET_SENSOR_ALTITUDE, tx_data[0], tx_data[1], tx_data[2]},
                          SCD4X_I2C_CMD_LENGTH + SCD4X_SETTING_LENGTH) != 0)
        return SCD4X_I2C_ERROR;

    return SCD4X_SUCCESS;
}

/**
 * @brief Retrieves the current sensor altitude setting.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[out] altitude Pointer to a `uint16_t` where the sensor altitude in meters will be stored.
 *
 * @retval `SCD4X_SUCCESS` The altitude was read successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` or `altitude` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed.
 */
Scd4xStatus scd4x_get_sensor_altitude(Scd4xDevice *device, uint16_t *altitude_m)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(altitude_m);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_GET_SENSOR_ALTITUDE}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    *altitude_m = rx_data[0] << 8 | rx_data[1];
    return SCD4X_SUCCESS;
}

/**
 * @brief Sets the current ambient pressure.
 *
 * The ambient pressure is used to correct the measured CO2 concentration. To save the setting to
 * the non-volatile EEPROM, call `scd4x_persist_settings`.
 *
 * WARNING: Do not issue any commands after calling this function for 1 millisecond.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[in] pressure_hPa The ambient pressure in hecto Pascals, of type `uint16_t`.
 * Range is 700 to 1200 hPa. Default value is 1013 hPa.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation failed.
 */
Scd4xStatus scd4x_set_ambient_pressure(Scd4xDevice *device, uint16_t pressure_hPa)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));

    uint8_t tx_data[SCD4X_SETTING_LENGTH];
    tx_data[0] = pressure_hPa >> 8;
    tx_data[1] = pressure_hPa & 0xFF;
    SCD4X_CHECK_STATUS(scd4x_calculate_checksum(device, tx_data, &tx_data[2]));

    if (device->i2c_write(SCD4X_I2C_ADDRESS, (uint8_t[]){SCD4X_CMD_GET_SET_AMBIENT_PRESSURE, tx_data[0], tx_data[1], tx_data[2]},
                          SCD4X_I2C_CMD_LENGTH + SCD4X_SETTING_LENGTH) != 0)
        return SCD4X_I2C_ERROR;

    return SCD4X_SUCCESS;
}

/**
 * @brief Retrieves the current ambient pressure setting.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[out] pressure_hPa Pointer to a `uint16_t` where the ambient pressure in hecto Pascals will be stored.
 *
 * @retval `SCD4X_SUCCESS` The ambient pressure was read successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` or `pressure_hPa` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed.
 */
Scd4xStatus scd4x_get_ambient_pressure(Scd4xDevice *device, uint16_t *pressure_hPa)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(pressure_hPa);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_GET_SET_AMBIENT_PRESSURE}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    *pressure_hPa = rx_data[0] << 8 | rx_data[1];
    return SCD4X_SUCCESS;
}

/**
 * @brief Perform forced recalibration on the sensor.
 *
 * To successfully conduct an accurate FRC, the following steps need to be carried out:
 *
 * 1. Operate the SCD4x in the operation mode later used for normal sensor operation (e.g. periodic measurement) for at least
 * 3 minutes in an environment with a homogenous and constant CO2 concentration. The sensor must be operated at the
 * voltage desired for the application when performing the FRC sequence.
 *
 * 2. Call the `scd4x_stop_periodic_measurement` function. Wait 500 ms for the command to complete.
 *
 * 3. Call `scd4x_perform_forced_recalibration` function. Note that this function contains 400 ms wait time before reading out
 * the FRC correction.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[in] target_co2_ppm The desired CO2 concentration in parts per million, of type `uint16_t`.
 * @param[out] frc_correction The calculated FRC correction in ppm, of type `int16_t`.
 *
 * @retval `SCD4X_SUCCESS` The forced recalibration was successful.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` or `frc_correction` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed.
 * @retval `SCD4X_FRC_ERROR` Forced recalibration failed (sensor not operated before FRC procedure).
 */
Scd4xStatus scd4x_perform_forced_recalibration(Scd4xDevice *device, uint16_t target_co2_ppm, int16_t *frc_correction)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(frc_correction);

    uint8_t tx_data[SCD4X_SETTING_LENGTH];
    tx_data[0] = target_co2_ppm >> 8;
    tx_data[1] = target_co2_ppm & 0xFF;
    SCD4X_CHECK_STATUS(scd4x_calculate_checksum(device, tx_data, &tx_data[2]));

    if (device->i2c_write(SCD4X_I2C_ADDRESS, (uint8_t[]){SCD4X_CMD_PERFORM_FORCED_RECALIBRATION, tx_data[0], tx_data[1], tx_data[2]},
                          SCD4X_I2C_CMD_LENGTH) != 0)
        return SCD4X_I2C_ERROR;

    device->delay_ms(400);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    if (rx_data[0] == 0xFF && rx_data[1] == 0xFF)
        return SCD4X_FRC_ERROR;

    *frc_correction = (rx_data[0] << 8 | rx_data[1]) - 0x8000; // see datasheet section 3.8.1

    return SCD4X_SUCCESS;
}

/**
 * @brief Enables or disables automatic self-calibration (ASC).
 *
 * To save the setting to the non-volatile EEPROM, call `scd4x_persist_settings`.
 *
 * WARNING: Do not issue any commands after calling this function for 1 millisecond.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[in] enabled Set to `true` to enable ASC, or `false` to disable ASC.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation failed.
 */
Scd4xStatus scd4x_set_automatic_self_calibration_enabled(Scd4xDevice *device, bool enabled)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));

    uint8_t tx_data[SCD4X_SETTING_LENGTH];
    tx_data[0] = enabled ? 0x01 : 0x00;
    tx_data[1] = 0x00;
    SCD4X_CHECK_STATUS(scd4x_calculate_checksum(device, tx_data, &tx_data[2]));

    if (device->i2c_write(SCD4X_I2C_ADDRESS, (uint8_t[]){SCD4X_CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED, tx_data[0], tx_data[1], tx_data[2]},
                          SCD4X_I2C_CMD_LENGTH) != 0)
        return SCD4X_I2C_ERROR;

    return SCD4X_SUCCESS;
}

/**
 * @brief Retrieves the current status of automatic self-calibration (ASC).
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[out] enabled Set to `true` if ASC is enabled, `false` otherwise.
 *
 * @retval `SCD4X_SUCCESS` The data was read from the device successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation failed.
 */
Scd4xStatus scd4x_get_automatic_self_calibration_enabled(Scd4xDevice *device, bool *enabled)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(enabled);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    *enabled = rx_data[0] == 0x01;
    return SCD4X_SUCCESS;
}

/**
 * @brief Sets the target CO2 concentration for automatic self-calibration (ASC).
 *
 * The target CO2 concentration is used as a reference point for ASC. The
 * device will automatically adjust its CO2 measurement to match this target
 * value.
 *
 * To save the setting to the non-volatile EEPROM, call `scd4x_persist_settings`.
 *
 * WARNING: Do not issue any commands after calling this function for 1 millisecond.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[in] target_co2_ppm The target CO2 concentration in parts per million,
 *     of type `uint16_t`. Default value is 400 ppm.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation failed.
 */
Scd4xStatus scd4x_set_automatic_self_calibration_target(Scd4xDevice *device, uint16_t target_co2_ppm)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));

    uint8_t tx_data[SCD4X_SETTING_LENGTH];
    tx_data[0] = target_co2_ppm >> 8;
    tx_data[1] = target_co2_ppm & 0xFF;
    SCD4X_CHECK_STATUS(scd4x_calculate_checksum(device, tx_data, &tx_data[2]));

    if (device->i2c_write(SCD4X_I2C_ADDRESS, (uint8_t[]){SCD4X_CMD_SET_AUTOMATIC_SELF_CALIBRATION_TARGET, tx_data[0], tx_data[1], tx_data[2]},
                          SCD4X_I2C_CMD_LENGTH) != 0)
        return SCD4X_I2C_ERROR;

    return SCD4X_SUCCESS;
}

/**
 * @brief Retrieves the current target CO2 concentration for automatic self-calibration (ASC).
 *
 * This function sends a command to the SCD4x device to read the baseline CO2 concentration
 * used as a reference for ASC. The device will adjust its CO2 measurements based on this target.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[out] target_co2_ppm Pointer to a `uint16_t` where the target CO2 concentration in
 *     parts per million (ppm) will be stored.
 *
 * @retval `SCD4X_SUCCESS` The target CO2 concentration was read successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` or `target_co2_ppm` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed.
 */
Scd4xStatus scd4x_get_automatic_self_calibration_target(Scd4xDevice *device, uint16_t *target_co2_ppm)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(target_co2_ppm);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_GET_AUTOMATIC_SELF_CALIBRATION_TARGET}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    *target_co2_ppm = rx_data[0] << 8 | rx_data[1];

    return SCD4X_SUCCESS;
}

/**
 * @brief Starts low power periodic measurement mode.
 *
 * The measurement interval for low power periodic measurements is approximately 30 seconds.
 * To check whether a new measurement result is available, call `scd41_get_data_ready_status`.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation failed.
 */
Scd4xStatus scd4x_start_low_power_periodic_measurement(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    return scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_START_LOW_POWER_PERIODIC_MEASUREMENT});
}

/**
 * @brief Checks if new measurement data is ready to be read from the sensor.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[out] ready Set to `true` if new data is ready, `false` otherwise.
 *
 * @retval `SCD4X_SUCCESS` The data ready status was read successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` or `ready` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed.
 */
Scd4xStatus scd4x_get_data_ready_status(Scd4xDevice *device, bool *ready)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(ready);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_GET_DATA_READY_STATUS}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    // Data is ready if 11 least significant bits are not zero
    *ready = ((rx_data[0] << 8 | rx_data[1]) & 0x07FF) ? true : false;

    return SCD4X_SUCCESS;
}

/**
 * @brief Persists the current settings to the non-volatile memory of the sensor.
 *
 * Call this function after configuring all settings that need to be retained after a power cycle.
 *
 * WARNING: Do not issue any commands after calling this function for 800 milliseconds.
 *
 * WARNING: The EEPROM is guaranteed to withstand only 2000 write cycles.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Scd4xStatus scd4x_persist_settings(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    return scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_PERSIST_SETTINGS});
}

/**
 * @brief Retrieves the serial number of the SCD4x sensor and verifies its checksum.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[out] serial_number Pointer to a `uint64_t` where the 48-bit serial number will be stored.
 *
 * @retval `SCD4X_SUCCESS` The serial number was read successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` or `serial_number` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed for one or more segments.
 */
Scd4xStatus scd4x_get_serial_number(Scd4xDevice *device, uint64_t *serial_number)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(serial_number);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_GET_SERIAL_NUMBER}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_SERIAL_NUMBER_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SERIAL_NUMBER_LENGTH));

    for (uint8_t i = 0; i < SCD4X_SERIAL_NUMBER_LENGTH; i = i + 3)
        SCD4X_CHECK_STATUS(scd4x_check_checksum(device, (uint8_t[]){rx_data[i], rx_data[i + 1]}, rx_data[i + 2]));

    // Copy the serial number to the output buffer, skipping the checksum bytes
    uint8_t input_index = 0, output_index = 0;
    uint8_t serial_number_bytes[SCD4X_SERIAL_NUMBER_LENGTH / 3 * 2];
    while (output_index < SCD4X_SERIAL_NUMBER_LENGTH)
    {
        if (input_index % 3 != 2)
        {
            serial_number_bytes[output_index] = rx_data[input_index];
            output_index++;
        }
        input_index++;
    }

    *serial_number = (uint64_t)serial_number_bytes[0] << 40 | (uint64_t)serial_number_bytes[1] << 32 |
                     (uint32_t)serial_number_bytes[2] << 24 | (uint32_t)serial_number_bytes[3] << 16 |
                     (uint32_t)serial_number_bytes[4] << 8 | (uint32_t)serial_number_bytes[5];

    return SCD4X_SUCCESS;
}

/**
 * @brief Perform a self-test on the sensor.
 *
 * WARNING: This function contains a wait time of 10 seconds!
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The self-test was completed. The result .
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` or `test_result` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed.
 * @retval `SCD4X_SELF_TEST_FAILURE` The self-test failed.
 */
Scd4xStatus scd4x_perform_self_test(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_PERFORM_SELF_TEST}));

    device->delay_ms(10000);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    if ((rx_data[0] << 8 | rx_data[1]) == 0x00)
        return SCD4X_SUCCESS;

    return SCD4X_SELF_TEST_FAILURE;
}

/**
 * @brief Resets the sensor to its factory settings.
 *
 * This function resets all configuration settings stored in the EEPROM and erases
 * the FRC and ASC algorithm history.
 *
 * WARNING: Do not issue any commands after calling this function for 1200 milliseconds.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Scd4xStatus scd4x_perform_factory_reset(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    return scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_PERFORM_FACTORY_RESET});
}

/**
 * @brief Reinitializes the sensor user settings from EEPROM.
 *
 * WARNING: Do not issue any commands after calling this function for 30 milliseconds.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Scd4xStatus scd4x_reinit(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    return scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_REINIT});
}

/**
 * @brief Retrieves the variant of the SCD4x sensor.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[out] variant Pointer to a `Scd4xSensorVariant` enum where the sensor variant will be stored.
 *
 * @retval `SCD4X_SUCCESS` The variant was read successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` or `variant` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed.
 * @retval `SCD4X_INVALID_VALUE` The returned variant is not a valid SCD4x sensor variant.
 */
Scd4xStatus scd4x_get_sensor_variant(Scd4xDevice *device, Scd4xSensorVariant *variant)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(variant);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_GET_SENSOR_VARIANT}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    if ((rx_data[0] & 0xF0) == 0x10) // response must be 0x1???
        *variant = SCD4X_SENSOR_VARIANT_SCD41;
    else if ((rx_data[0] | 0x0F) == 0x0F) // response must be 0x0???
        *variant = SCD4X_SENSOR_VARIANT_SCD40;
    else
        return SCD4X_INVALID_VALUE;

    return SCD4X_SUCCESS;
}

/**
 * @brief Trigger a single CO2 + temperature + relative humidity measurement.
 *
 * The measurement result is not immediately available. Call `scd4x_get_data_ready_status` to check when the measurement
 * result is available. The sampling interval is approximately 5 seconds.
 * The measurement result can then be retrieved using the `scd41_get_measurement` function.
 *
 * WARNING: Do not issue any commands after calling this function for 5 seconds.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Scd4xStatus scd41_measure_single_shot(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    return scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_MEASURE_SINGLE_SHOT});
}

/**
 * @brief Trigger a single temperature + relative humidity measurement.
 *
 * The measurement result is not immediately available. Call `scd4x_get_data_ready_status` to check when the measurement
 * result is available. The sampling interval is approximately 50 milliseconds.
 * The measurement result can then be retrieved using the `scd41_get_measurement` function.
 * CO2 output is returned as 0 ppm.
 *
 * WARNING: Do not issue any commands after calling this function for 50 milliseconds.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Scd4xStatus scd41_measure_single_shot_rht_only(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    return scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_MEASURE_SINGLE_SHOT_RHT_ONLY});
}

/**
 * @brief Put the sensor from idle to sleep to reduce current consumption.
 *
 * To wake up the device from power down mode, call `scd41_wake_up`.
 *
 * WARNING: Do not issue any commands after calling this function for 1 millisecond.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Scd4xStatus scd41_power_down(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    return scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_POWER_DOWN});
}

/**
 * @brief Wakes the sensor from power down mode.
 *
 * To verify that the sensor is awake, call `scd41_get_serial_number`.
 *
 * WARNING: Do not issue any commands after calling this function for 30 millisecond.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Scd4xStatus scd41_wake_up(Scd4xDevice *device)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));

    // The sensor does not acknowledge the wake up command, send it blind
    device->i2c_write(SCD4X_I2C_ADDRESS, (uint8_t[]){SCD4X_CMD_WAKE_UP}, SCD4X_I2C_CMD_LENGTH);

    return SCD4X_SUCCESS;
}

/**
 * @brief Sets the initial period for automatic self-calibration (ASC) on the SCD41.
 *
 * To save the setting to the non-volatile EEPROM, call `scd4x_persist_settings`.
 *
 * Note: For single shot operation, this parameter always assumes a measurement interval
 * of 5 minutes, counting the number of
 * single shots to calculate elapsed time. If single shot measurements are taken more / less
 * frequently than once every 5 minutes,
 * this parameter must be scaled accordingly to achieve the intended period in hours
 * (e.g. for a 10-minute measurement interval,
 * the scaled parameter value is obtained by multiplying the intended period in hours by 0.5).
 *
 * WARNING: Do not issue any commands after calling this function for 1 millisecond.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[in] period The initial period in multiples of 4 hours, of type `uint16_t`.
 * Default value is 44 hours. A value of 0 results in an immediate ASC correction.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation failed.
 */
Scd4xStatus scd41_set_automatic_self_calibration_initial_period(Scd4xDevice *device, uint16_t period_4hrs)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));

    uint8_t tx_data[SCD4X_SETTING_LENGTH];
    tx_data[0] = period_4hrs >> 8;
    tx_data[1] = period_4hrs & 0xFF;
    SCD4X_CHECK_STATUS(scd4x_calculate_checksum(device, tx_data, &tx_data[2]));

    if (device->i2c_write(SCD4X_I2C_ADDRESS, (uint8_t[]){SCD4X_CMD_SET_AUTOMATIC_SELF_CALIBRATION_INITIAL_PERIOD, tx_data[0], tx_data[1], tx_data[2]},
                          SCD4X_I2C_CMD_LENGTH) != 0)
        return SCD4X_I2C_ERROR;

    return SCD4X_SUCCESS;
}

/**
 * @brief Retrieves the current initial period for automatic self-calibration (ASC) on the SCD41.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[out] period_4hrs Pointer to a `uint16_t` where the initial period in multiples of 4 hours will be stored.
 *
 * @retval `SCD4X_SUCCESS` The initial period was read successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` or `period_4hrs` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed.
 */
Scd4xStatus scd41_get_automatic_self_calibration_initial_period(Scd4xDevice *device, uint16_t *period_4hrs)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(period_4hrs);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_GET_AUTOMATIC_SELF_CALIBRATION_INITIAL_PERIOD}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    *period_4hrs = rx_data[0] << 8 | rx_data[1];

    return SCD4X_SUCCESS;
}

/**
 * @brief Sets the standard period for automatic self-calibration (ASC) on the SCD41.
 *
 * To save the setting to the non-volatile EEPROM, call `scd4x_persist_settings`.
 *
 * WARNING: Do not issue any commands after calling this function for 1 millisecond.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[in] period_4hrs The standard period in multiples of 4 hours, of type `uint16_t`.
 * Default value is 156 hours.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation failed.
 */
Scd4xStatus scd41_set_automatic_self_calibration_standard_period(Scd4xDevice *device, uint16_t period_4hrs)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));

    uint8_t tx_data[SCD4X_SETTING_LENGTH];
    tx_data[0] = period_4hrs >> 8;
    tx_data[1] = period_4hrs & 0xFF;
    SCD4X_CHECK_STATUS(scd4x_calculate_checksum(device, tx_data, &tx_data[2]));

    if (device->i2c_write(SCD4X_I2C_ADDRESS, (uint8_t[]){SCD4X_CMD_SET_AUTOMATIC_SELF_CALIBRATION_STANDARD_PERIOD, tx_data[0], tx_data[1], tx_data[2]},
                          SCD4X_I2C_CMD_LENGTH) != 0)
        return SCD4X_I2C_ERROR;

    return SCD4X_SUCCESS;
}

/**
 * @brief Retrieves the current standard period for automatic self-calibration (ASC) on the SCD41.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[out] period_4hrs Pointer to a `uint16_t` where the standard period in multiples of 4 hours will be stored.
 *
 * @retval `SCD4X_SUCCESS` The standard period was read successfully.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` or `period_4hrs` pointer is `NULL`.
 * @retval `SCD4X_CRC_FAILURE` CRC verification failed.
 */
Scd4xStatus scd41_get_automatic_self_calibration_standard_period(Scd4xDevice *device, uint16_t *period_4hrs)
{
    SCD4X_CHECK_STATUS(scd4x_check_device(device));
    SCD4X_CHECK_NULL(period_4hrs);

    SCD4X_CHECK_STATUS(scd4x_send_i2c_command(device, (uint8_t[]){SCD4X_CMD_GET_AUTOMATIC_SELF_CALIBRATION_STANDARD_PERIOD}));

    device->delay_ms(1);

    uint8_t rx_data[SCD4X_SETTING_LENGTH];
    SCD4X_CHECK_STATUS(scd4x_receive_i2c_data(device, rx_data, SCD4X_SETTING_LENGTH));

    SCD4X_CHECK_STATUS(scd4x_check_checksum(device, rx_data, rx_data[2]));

    *period_4hrs = rx_data[0] << 8 | rx_data[1];

    return SCD4X_SUCCESS;
}

/**
 * @brief Verifies the checksum for a given data array using the SCD4x device's CRC function.
 *
 * This function calculates the CRC-8 checksum of the provided 2-byte data array and compares it
 * with the given checksum. If the SCD4x device has a custom CRC calculation function defined, it uses that function.
 * Otherwise, it falls back to the default software CRC-8 algorithm.
 *
 * @param[in] device The `Scd4xDevice` struct containing the CRC calculation function.
 * @param[in] data A 2-byte array for which the checksum is to be verified.
 * @param[in] checksum A `uint8_t` containing the expected checksum.
 *
 * @retval `SCD4X_SUCCESS` Checksum verified successfully.
 * @retval `SCD4X_CRC_FAILURE` CRC verification using the device's function failed.
 */
static Scd4xStatus scd4x_check_checksum(Scd4xDevice *device, uint8_t data[2], uint8_t checksum)
{
    uint8_t calculated_checksum = 0;
    SCD4X_CHECK_STATUS(scd4x_calculate_checksum(device, data, &calculated_checksum));

    if (calculated_checksum != checksum)
        return SCD4X_CRC_FAILURE;

    return SCD4X_SUCCESS;
}

/**
 * @brief Calculates the checksum for a given data array using the SCD4x device's CRC function.
 *
 * This function calculates the CRC-8 checksum of the provided 2-byte data array.
 * If the SCD4x device has a custom CRC calculation function defined, it uses that function.
 * Otherwise, it falls back to the default software CRC-8 algorithm.
 *
 * @param[in] device The `Scd4xDevice` struct containing the CRC calculation function.
 * @param[in] data A 2-byte array for which the checksum is to be calculated.
 * @param[out] checksum A pointer to a `uint8_t` where the calculated checksum is stored.
 *
 * @retval `SCD4X_SUCCESS` Checksum calculated successfully.
 * @retval `SCD4X_CRC_FAILURE` CRC calculation using the device's function failed.
 */
static Scd4xStatus scd4x_calculate_checksum(Scd4xDevice *device, uint8_t data[2], uint8_t *checksum)
{
    if (device->calculate_crc != NULL)
    {
        if (device->calculate_crc(data, 2, SCD4X_CRC8_POLYNOMIAL, checksum) != 0)
            return SCD4X_CRC_FAILURE;
    }
    else
        *checksum = scd4x_calculate_checksum_default(data);

    return SCD4X_SUCCESS;
}

/**
 * @brief Calculates the CRC-8 checksum of two bytes of data.
 *
 * This function implements the CRC-8 algorithm as specified in the SCD4x
 * datasheet. It takes two bytes of data and returns the calculated CRC-8
 * checksum.
 *
 * @param[in] data Two bytes of data to calculate the CRC-8 checksum for.
 *
 * @return The calculated CRC-8 checksum.
 */
static uint8_t scd4x_calculate_checksum_default(uint8_t data[2])
{
    uint8_t crc = SCD4X_CRC8_INITIAL_VALUE;
    for (int i = 0; i < 2; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ SCD4X_CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Checks whether the provided `Scd4xDevice` struct contains valid pointers.
 *
 * Returns -1 if the `device`pointer is NULL, or if the `i2c_write`, `i2c_read` or
 * `delay_ms` function pointers are `NULL`.
 *
 * WARNING: This function does not check whether the function pointers are
 * pointing to valid functions.
 *
 * @param[in] device The `Scd4xDevice` struct to be checked.
 *
 * @retval `SCD4X_SUCCESS` The `Scd4xDevice` struct pointers are valid.
 * @retval `SCD4X_POINTER_NULL` One of the pointers is `NULL`.
 */
static Scd4xStatus scd4x_check_device(Scd4xDevice *device)
{
    if (device == NULL)
        return SCD4X_POINTER_NULL;
    if (device->i2c_write == NULL || device->i2c_read == NULL || device->delay_ms == NULL)
        return SCD4X_POINTER_NULL;
    return SCD4X_SUCCESS;
}

/**
 * @brief Sends an I2C command to the sensor.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C functions.
 * @param[in] command The command byte array to be sent.
 *
 * @retval `SCD4X_SUCCESS` The command was sent to the device.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SCD4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
static Scd4xStatus scd4x_send_i2c_command(Scd4xDevice *device, uint8_t *command)
{
    if (device->i2c_write(SCD4X_I2C_ADDRESS, command, (size_t)SCD4X_I2C_CMD_LENGTH) != 0)
        return SCD4X_I2C_ERROR;

    return SCD4X_SUCCESS;
}

/**
 * @brief Receives data from the sensor via I2C.
 *
 * @param[in] device The `Scd4xDevice` struct containing the I2C read function.
 * @param[out] data A pointer to the buffer where the received data will be stored.
 * @param[in] data_length The number of bytes to read from the sensor.
 *
 * @retval `SCD4X_SUCCESS` The data was successfully received.
 * @retval `SCD4X_I2C_ERROR` An I2C communication error occurred.
 */
static Scd4xStatus scd4x_receive_i2c_data(Scd4xDevice *device, uint8_t *data, uint8_t data_length)
{
    if (device->i2c_read(SCD4X_I2C_ADDRESS, data, data_length) != 0)
        return SCD4X_I2C_ERROR;
    return SCD4X_SUCCESS;
}