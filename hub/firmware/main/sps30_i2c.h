/**
 * @file sps30_i2c.h
 * @author Jakub Franek (https://github.com/JakubFranek)
 * @brief SPS30 I2C driver
 *
 * How to use this driver:
 * 1. Include this header file in your code.
 * 2. Create an instance of `Sps30Device` and initialize it with the required function pointers.
 * 3. Call the driver functions as desired (typical first call is `sps30_start_measurement`).
 */

#ifndef __SPS30_I2C_H__
#define __SPS30_I2C_H__

#include <stdint.h>  // define uint8_t
#include <stdbool.h> // define bool

/* --- Constants --- */

#define SPS30_I2C_ADDRESS 0x69

/* --- Function pointers --- */
// Target functions must return int8_t error code, 0 is the only accepted success value

typedef int8_t (*sps30_i2c_write_t)(uint8_t address, const uint8_t *payload, uint8_t length);
typedef int8_t (*sps30_i2c_read_t)(uint8_t address, uint8_t *payload, uint8_t length);
typedef int8_t (*sps30_calculate_crc_t)(const uint8_t *data, uint8_t length, uint8_t polynomial, uint8_t *result);
typedef int8_t (*sps30_delay_ms_t)(uint16_t ms);

/* --- Types --- */

typedef enum Sps30Status
{
    SPS30_SUCCESS = 0,
    SPS30_I2C_ERROR = -1,
    SPS30_INVALID_VALUE = -2,
    SPS30_POINTER_NULL = -3,
    SPS30_CRC_FAILURE = -4
} Sps30Status;

typedef struct Sps30FirmwareVersion
{
    uint8_t major;
    uint8_t minor;
} Sps30FirmwareVersion;

typedef enum Sps30DataFormat
{
    SPS30_FLOAT = 0,
    SPS30_UINT16 = -1
} Sps30DataFormat;

typedef struct Sps30StatusFlags
{
    bool speed_warning;
    bool laser_error;
    bool fan_error;
} Sps30StatusFlags;

/**
 * @brief A structure containing the `float` data read from the SPS30 sensor.
 *
 * Mass concentration unit is `ug/m^3`, number concentration unit is `#/cm^3`
 * and typical particle size unit is `nm`.
 */
typedef struct Sps30FloatData
{
    float mass_concentration_pm1_0;
    float mass_concentration_pm2_5;
    float mass_concentration_pm4_0;
    float mass_concentration_pm10_0;
    float number_concentration_pm0_5;
    float number_concentration_pm1_0;
    float number_concentration_pm2_5;
    float number_concentration_pm4_0;
    float number_concentration_pm10_0;
    float typical_particle_size;
} Sps30FloatData;

/**
 * @brief A structure containing the `uint16_t` data read from the SPS30 sensor.
 *
 * Mass concentration unit is `ug/m^3`, number concentration unit is `#/cm^3`
 * and typical particle size unit is `nm`.
 */
typedef struct Sps30Uint16Data
{
    uint16_t mass_concentration_pm1_0;
    uint16_t mass_concentration_pm2_5;
    uint16_t mass_concentration_pm4_0;
    uint16_t mass_concentration_pm10_0;
    uint16_t number_concentration_pm0_5;
    uint16_t number_concentration_pm1_0;
    uint16_t number_concentration_pm2_5;
    uint16_t number_concentration_pm4_0;
    uint16_t number_concentration_pm10_0;
    uint16_t typical_particle_size;
} Sps30Uint16Data;

typedef struct Sps30Device
{
    sps30_i2c_write_t i2c_write;
    sps30_i2c_read_t i2c_read;
    sps30_delay_ms_t delay_ms;
    sps30_calculate_crc_t calculate_crc; // Optional: If `NULL`, internal SW CRC algorithm will be used
} Sps30Device;

/* --- Function Prototypes --- */

Sps30Status sps30_start_measurement(Sps30Device *device, Sps30DataFormat data_format);
Sps30Status sps30_stop_measurement(Sps30Device *device);
Sps30Status sps30_read_data_ready_flag(Sps30Device *device, bool *data_ready);
Sps30Status sps30_read_measured_values_float(Sps30Device *device, Sps30FloatData *data);
Sps30Status sps30_read_measured_values_uint16(Sps30Device *device, Sps30Uint16Data *data);
Sps30Status sps30_sleep(Sps30Device *device);
Sps30Status sps30_wake_up(Sps30Device *device);
Sps30Status sps30_start_fan_cleaning(Sps30Device *device);
Sps30Status sps30_read_auto_cleaning_interval(Sps30Device *device, uint32_t *interval);
Sps30Status sps30_set_auto_cleaning_interval(Sps30Device *device, uint32_t interval);
Sps30Status sps30_read_product_type(Sps30Device *device, char *product_type);
Sps30Status sps30_read_serial_number(Sps30Device *device, char *serial_number);
Sps30Status sps30_read_firmware_version(Sps30Device *device, Sps30FirmwareVersion *version);
Sps30Status sps30_read_device_status_flags(Sps30Device *device, Sps30StatusFlags *status_flags);
Sps30Status sps30_clear_device_status_flags(Sps30Device *device);
Sps30Status sps30_reset(Sps30Device *device);

#endif /* __SPS30_I2C_H__ */