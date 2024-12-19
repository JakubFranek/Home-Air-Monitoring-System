/**
 * @file sht4x.h
 * @author Jakub Franek (https://github.com/JakubFranek)
 * @brief SHT4x I2C driver
 *
 * How to use this driver:
 * 1. Include this header file in your code.
 * 2. Create an instance of `Sht4xDevice` and initialize it with the required function pointers.
 * 3. Call the driver functions as desired.
 */

#ifndef INC_SHT4X_H_
#define INC_SHT4X_H_

#include <stdint.h> // definition of uint8_t etc

/* --- Constants --- */

#define SHT4X_MEAS_HIGH_PREC_PERIOD_US 8200 // in microseconds
#define SHT4X_MEAS_MED_PREC_PERIOD_US 4500	// in microseconds
#define SHT4X_MEAS_LOW_PREC_PERIOD_US 1700	// in microseconds
#define SHT4X_SERIAL_NUMBER_PERIOD_US 1000	// in microseconds

/* --- Function pointers --- */
// Target functions must return int8_t error code, 0 is the only accepted success value

typedef int8_t (*sht4x_i2c_write_t)(uint8_t address, const uint8_t *payload, uint8_t length);
typedef int8_t (*sht4x_i2c_read_t)(uint8_t address, uint8_t *payload, uint8_t length);
typedef int8_t (*sht4x_calculate_crc_t)(const uint8_t *data, uint8_t length, uint8_t polynomial, uint8_t *result);

/* --- Types --- */

typedef enum Sht4xI2cAddress
{
	SHT4X_I2C_ADDR_A = 0x44,
	SHT4X_I2C_ADDR_B = 0x45
} Sht4xI2cAddress; // 8 bit number

typedef enum Sht4xMeasurement
{
	SHT4X_I2C_CMD_MEAS_HIGH_PREC = 0xFD,  // measurement duration up to 8.2 ms
	SHT4X_I2C_CMD_MEAS_MED_PREC = 0xF6,	  // measurement duration up to 4.5 ms
	SHT4X_I2C_CMD_MEAS_LOW_PREC = 0xE0,	  // measurement duration up to 1.7 ms
	SHT4X_I2C_CMD_HEAT_200MW_1S = 0x39,	  // automatically starts high precision measurement
	SHT4X_I2C_CMD_HEAT_200MW_0P1S = 0x32, // automatically starts high precision measurement
	SHT4X_I2C_CMD_HEAT_110MW_1S = 0x2F,	  // automatically starts high precision measurement
	SHT4X_I2C_CMD_HEAT_110MW_0P1S = 0x24, // automatically starts high precision measurement
	SHT4X_I2C_CMD_HEAT_20MW_1S = 0x1E,	  // automatically starts high precision measurement
	SHT4X_I2C_CMD_HEAT_20MW_0P1S = 0x15	  // automatically starts high precision measurement
} Sht4xMeasurement;

typedef enum Sht4xStatus
{
	SHT4X_SUCCESS = 0,
	SHT4X_I2C_ERROR = -1,
	SHT4X_INVALID_VALUE = -2,
	SHT4X_INVALID_OPERATION = -3,
	SHT4X_POINTER_NULL = -4,
	SHT4X_CRC_FAILURE = -5
} Sht4xStatus;

typedef struct Sht4xData
{
	int32_t temperature; // convert to degrees Celsius via division by 1000
	int32_t humidity;	 // convert to % RH via division by 1000
} Sht4xData;

typedef struct Sht4xDevice
{
	Sht4xI2cAddress i2c_address;
	sht4x_i2c_write_t i2c_write;
	sht4x_i2c_read_t i2c_read;
	sht4x_calculate_crc_t calculate_crc; // Optional: If `NULL`, internal SW CRC algorithm will be used
} Sht4xDevice;

/* --- Function Prototypes --- */

Sht4xStatus sht4x_start_measurement(Sht4xDevice *device, Sht4xMeasurement command);
Sht4xStatus sht4x_read_measurement(Sht4xDevice *device, Sht4xData *data);
Sht4xStatus sht4x_request_serial_number(Sht4xDevice *device);
Sht4xStatus sht4x_read_serial_number(Sht4xDevice *device, uint32_t *serial_number);
Sht4xStatus sht4x_soft_reset(Sht4xDevice *device);

#endif /* INC_SHT4X_H_ */
