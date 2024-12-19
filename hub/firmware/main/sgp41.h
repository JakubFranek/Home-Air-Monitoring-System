/**
 * @file sgp41.h
 * @author Jakub Franek (https://github.com/JakubFranek)
 * @brief SGP41 I2C driver
 *
 * How to use this driver:
 * 1. Include this header file in your code.
 * 2. Create an instance of `Sgp41Device` and initialize it with the required function pointers.
 * 3. Initialize the VOC and NOx gas index algorithm parameters by calling `sgp41_initialize` function.
 * 4. Call the driver functions as desired (it is beneficial to run `sgp41_execute_conditioning` before measurements).
 */

#ifndef INC_SGP41_H_
#define INC_SGP41_H_

#include <stdint.h>
#include "sensirion_gas_index_algorithm.h"

/* --- Constants --- */

#define SGP41_I2C_ADDRESS 0x59

/* --- Function pointers --- */
// Target functions must return int8_t error code, 0 is the only accepted success value

typedef int8_t (*sgp41_i2c_write_t)(uint8_t address, const uint8_t *payload, uint8_t length);
typedef int8_t (*sgp41_i2c_read_t)(uint8_t address, uint8_t *payload, uint8_t length);
typedef int8_t (*sgp41_calculate_crc_t)(const uint8_t *data, uint8_t length, uint8_t polynomial, uint8_t *result);

/* --- Types --- */

typedef enum Sgp41Status
{
    SGP41_SUCCESS = 0,
    SGP41_I2C_ERROR = -1,
    SGP41_INVALID_VALUE = -2,
    SGP41_INVALID_OPERATION = -3,
    SGP41_POINTER_NULL = -4,
    SGP41_CRC_FAILURE = -5,
    SGP41_NOT_IMPLEMENTED = -6,
    SGP41_SELF_TEST_FAILURE = -7
} Sgp41Status;

typedef struct Sgp41Data
{
    int32_t voc_index; // 0..500, average value: 100
    int32_t nox_index; // 1..500, average value: 1
} Sgp41Data;

typedef struct Sgp41Device
{
    sgp41_i2c_write_t i2c_write;
    sgp41_i2c_read_t i2c_read;
    sgp41_calculate_crc_t calculate_crc; // Optional: If `NULL`, internal SW CRC algorithm will be used
    GasIndexAlgorithmParams gia_voc;     // Do not initialize, leave as `NULL`
    GasIndexAlgorithmParams gia_nox;     // Do not initialize, leave as `NULL`
} Sgp41Device;

/* --- Public functions --- */

Sgp41Status sgp41_initialize(Sgp41Device *device);
Sgp41Status sgp41_execute_conditioning(Sgp41Device *device);
Sgp41Status sgp41_measure_raw_signals(Sgp41Device *device, float *temp_celsius, float *rel_hum_pct);
Sgp41Status sgp41_read_gas_indices(Sgp41Device *device, Sgp41Data *data);
Sgp41Status sgp41_get_serial_number(Sgp41Device *device, uint64_t *serial_number);
Sgp41Status sgp41_turn_heater_off(Sgp41Device *device);
Sgp41Status sgp41_execute_self_test(Sgp41Device *device);
Sgp41Status sgp41_evaluate_self_test(Sgp41Device *device);

#endif /* INC_SGP41_H_ */