/**
 * @file scd4x.h
 * @author Jakub Franek (https://github.com/JakubFranek)
 * @brief SCD4x I2C driver
 *
 * How to use this driver:
 * 1. Include this header file in your code.
 * 2. Create an instance of `Scd4xDevice` and initialize it with the required function pointers.
 * 3. Call the driver functions as desired (typical first call is `scd4x_start_periodic_measurement`).
 */

#ifndef __SCD4X_I2C_H__
#define __SCD4X_I2C_H__

#include <stdint.h> // define uint8_t, uint16_t etc.

/* --- Public constants --- */

#define SCD4X_I2C_ADDRESS 0x62

/* --- Function pointers --- */
// Target functions must return int8_t error code, 0 is the only accepted success value

typedef int8_t (*scd4x_i2c_write_t)(uint8_t address, const uint8_t *payload, size_t length);
typedef int8_t (*scd4x_i2c_read_t)(uint8_t address, uint8_t *payload, size_t length);
typedef int8_t (*scd4x_delay_ms_t)(uint16_t ms);
typedef int8_t (*scd4x_calculate_crc_t)(const uint8_t *data, size_t length, uint8_t polynomial, uint8_t *result);

/* --- Types --- */

typedef enum Scd4xStatus
{
    SCD4X_SUCCESS = 0,
    SCD4X_I2C_ERROR = -1,
    SCD4X_INVALID_VALUE = -2,
    SCD4X_POINTER_NULL = -3,
    SCD4X_CRC_FAILURE = -4,
    SCD4X_FRC_ERROR = -5,
    SCD4X_SELF_TEST_FAILURE = -6
} Scd4xStatus;

typedef enum Scd4xSensorVariant
{
    SCD4X_SENSOR_VARIANT_SCD40 = 0,
    SCD4X_SENSOR_VARIANT_SCD41 = 1
} Scd4xSensorVariant;

typedef struct Scd4xData
{
    uint16_t co2_ppm;           // ppm
    int16_t temperature;        // 100 * °C
    uint16_t relative_humidity; // 100 * %
} Scd4xData;

typedef struct Scd4xDevice
{
    scd4x_i2c_write_t i2c_write;
    scd4x_i2c_read_t i2c_read;
    scd4x_delay_ms_t delay_ms;
    scd4x_calculate_crc_t calculate_crc; // Optional: If `NULL`, internal SW CRC algorithm will be used
} Scd4xDevice;

/* --- SCD4x functions --- */

Scd4xStatus scd4x_start_periodic_measurement(Scd4xDevice *device);
Scd4xStatus scd4x_read_measurement(Scd4xDevice *device, Scd4xData *data);
Scd4xStatus scd4x_stop_periodic_measurement(Scd4xDevice *device);
Scd4xStatus scd4x_set_temperature_offset(Scd4xDevice *device, float offset_degC);
Scd4xStatus scd4x_get_temperature_offset(Scd4xDevice *device, float *offset_degC);
Scd4xStatus scd4x_set_sensor_altitude(Scd4xDevice *device, uint16_t altitude_m);
Scd4xStatus scd4x_get_sensor_altitude(Scd4xDevice *device, uint16_t *altitude_m);
Scd4xStatus scd4x_set_ambient_pressure(Scd4xDevice *device, uint16_t pressure_hPa);
Scd4xStatus scd4x_get_ambient_pressure(Scd4xDevice *device, uint16_t *pressure_hPa);
Scd4xStatus scd4x_perform_forced_recalibration(Scd4xDevice *device, uint16_t target_co2_ppm, int16_t *frc_correction);
Scd4xStatus scd4x_set_automatic_self_calibration_enabled(Scd4xDevice *device, bool enabled);
Scd4xStatus scd4x_get_automatic_self_calibration_enabled(Scd4xDevice *device, bool *enabled);
Scd4xStatus scd4x_set_automatic_self_calibration_target(Scd4xDevice *device, uint16_t target_co2_ppm);
Scd4xStatus scd4x_get_automatic_self_calibration_target(Scd4xDevice *device, uint16_t *target_co2_ppm);
Scd4xStatus scd4x_start_low_power_periodic_measurement(Scd4xDevice *device);
Scd4xStatus scd4x_get_data_ready_status(Scd4xDevice *device, bool *ready);
Scd4xStatus scd4x_persist_settings(Scd4xDevice *device);
Scd4xStatus scd4x_get_serial_number(Scd4xDevice *device, uint64_t *serial_number);
Scd4xStatus scd4x_perform_self_test(Scd4xDevice *device);
Scd4xStatus scd4x_perform_factory_reset(Scd4xDevice *device);
Scd4xStatus scd4x_reinit(Scd4xDevice *device);
Scd4xStatus scd4x_get_sensor_variant(Scd4xDevice *device, Scd4xSensorVariant *variant);

/* --- SCD41-only functions --- */

Scd4xStatus scd41_measure_single_shot(Scd4xDevice *device);
Scd4xStatus scd41_measure_single_shot_rht_only(Scd4xDevice *device);
Scd4xStatus scd41_power_down(Scd4xDevice *device);
Scd4xStatus scd41_wake_up(Scd4xDevice *device);
Scd4xStatus scd41_set_automatic_self_calibration_initial_period(Scd4xDevice *device, uint16_t period_4hrs);
Scd4xStatus scd41_get_automatic_self_calibration_initial_period(Scd4xDevice *device, uint16_t *period_4hrs);
Scd4xStatus scd41_set_automatic_self_calibration_standard_period(Scd4xDevice *device, uint16_t period_4hrs);
Scd4xStatus scd41_get_automatic_self_calibration_standard_period(Scd4xDevice *device, uint16_t *period_4hrs);

#endif /* __SCD4X_I2C_H__ */