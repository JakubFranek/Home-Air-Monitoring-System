/**
 * @file bme280_i2c.c
 * @author Jakub Franek (https://github.com/JakubFranek)
 * @brief BME280 I2C driver
 *
 * How to use this driver:
 * // TODO: add how to - guide
 */

#include <stdint.h>
#include <stdbool.h>

/* --- Function pointers --- */
// Target functions must return int8_t error code, 0 is the only accepted success value

typedef int8_t (*bme280_i2c_write_t)(uint8_t address, const uint8_t *payload, uint8_t length);
typedef int8_t (*bme280_i2c_read_t)(uint8_t address, uint8_t *payload, uint8_t length);

typedef enum Bme280Status
{
    BME280_SUCCESS = 0,
    BME280_I2C_ERROR = -1,
    BME280_INVALID_VALUE = -2,
    BME280_INVALID_OPERATION = -3,
    BME280_POINTER_NULL = -4
} Bme280Status;

typedef enum Bme280I2cAddress
{
    BME280_I2C_ADDRESS_SDO_LOW = 0x76,
    BME280_I2C_ADDRESS_SDO_HIGH = 0x77
} Bme280I2cAddress;

typedef enum Bme280Oversampling
{
    BME280_OVERSAMPLING_NONE = 0,
    BME280_OVERSAMPLING_X1 = 1,
    BME280_OVERSAMPLING_X2 = 2,
    BME280_OVERSAMPLING_X4 = 3,
    BME280_OVERSAMPLING_X8 = 4,
    BME280_OVERSAMPLING_X16 = 5
} Bme280Oversampling;

typedef enum Bme280Mode
{
    BME280_MODE_NORMAL = 0,
    BME280_MODE_FORCED = 1,
    BME280_MODE_SLEEP = 3
} Bme280Mode;

typedef enum Bme280StandbyTime
{
    BME280_STANDBY_TIME_0_5_MS = 0,
    BME280_STANDBY_TIME_62_5_MS = 1,
    BME280_STANDBY_TIME_125_MS = 2,
    BME280_STANDBY_TIME_250_MS = 3,
    BME280_STANDBY_TIME_500_MS = 4,
    BME280_STANDBY_TIME_1000_MS = 5,
    BME280_STANDBY_TIME_10_MS = 6,
    BME280_STANDBY_TIME_20_MS = 7
} Bme280StandbyTime;

typedef enum Bme280Filter
{
    BME280_FILTER_OFF = 0,
    BME280_FILTER_X2 = 1,
    BME280_FILTER_X4 = 2,
    BME280_FILTER_X8 = 3,
    BME280_FILTER_X16 = 4
} Bme280Filter;

typedef struct Bme280Calibration
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
    int32_t t_fine;
} Bme280Calibration;

typedef struct Bme280Config
{
    Bme280Oversampling temperature_oversampling;
    Bme280Oversampling pressure_oversampling;
    Bme280Oversampling humidity_oversampling;
    Bme280StandbyTime standby_time;
    Bme280Filter filter;
    bool spi_3wire_enable;
} Bme280Config;

typedef struct Bme280Data
{
    int32_t temperature; // 100 * degC
    uint32_t pressure;   // 10 * Pa
    uint32_t humidity;   // 1000 * %RH
} Bme280Data;

typedef struct Bme280Device
{
    bme280_i2c_write_t i2c_write;
    bme280_i2c_read_t i2c_read;
    Bme280I2cAddress address;
    Bme280Config config;
    Bme280Calibration calibration; // Leave as `NULL`: this struct is initialized in `bme280_init`
} Bme280Device;

Bme280Status bme280_init(Bme280Device *device);
Bme280Status bme280_reset(Bme280Device *device);
Bme280Status bme280_set_mode(Bme280Device *device, Bme280Mode mode);
Bme280Status bme280_set_config(Bme280Device *device, Bme280Config *config);
Bme280Status bme280_is_measuring(Bme280Device *device, bool *measuring);
Bme280Status bme280_read_measurement(Bme280Device *device, Bme280Data *data);
