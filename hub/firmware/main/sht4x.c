#include <stddef.h> // definition of NULL
#include "sht4x.h"

/* --- Private definitions --- */

#define SHT4X_I2C_CMD_READ_SERIAL_NUMBER 0x89
#define SHT4X_I2C_CMD_SOFT_RESET 0x94
#define SHT4X_I2C_CMD_LENGHT 1				 // bytes
#define SHT4X_I2C_CMD_MEASUREMENT_LENGHT 6	 // bytes
#define SHT4X_I2C_CMD_SERIAL_NUMBER_LENGHT 6 // bytes
#define SHT4X_CRC8_POLYNOMIAL 0x31			 // x^8 + x^5 + x^4 + 1, initialization to 0xFF
#define SHT4X_CRC8_INIT 0xFF

/* --- Private macros --- */

/**
 * Error-checking macro: if `expr` is not `SHT4X_SUCCESS`, this macro returns `expr`,
 * exiting the function where this macro was used immediately.
 */
#define SHT4X_CHECK_STATUS(expr)     \
	do                               \
	{                                \
		Sht4xStatus retval = expr;   \
		if (retval != SHT4X_SUCCESS) \
		{                            \
			return retval;           \
		}                            \
	} while (0)

/**
 * Error-checking macro: if `expr` is `NULL`, this macro returns `SHT4X_POINTER_NULL`,
 * exiting the function where this macro was used immediately.
 */
#define SHT4X_CHECK_NULL(expr)         \
	do                                 \
	{                                  \
		if (expr == NULL)              \
		{                              \
			return SHT4X_POINTER_NULL; \
		}                              \
	} while (0)

/* --- Private function prototypes --- */

static Sht4xStatus sht4x_check_checksum(Sht4xDevice *device, uint8_t data[2], uint8_t checksum);
static Sht4xStatus sht4x_calculate_checksum(Sht4xDevice *device, uint8_t data[2], uint8_t *checksum);
static uint8_t sht4x_calculate_checksum_default(uint8_t data[2]);
static Sht4xStatus sht4x_check_device(Sht4xDevice *device);

/* --- Function definitions --- */

/**
 * @brief Starts a measurement on the SHT4x sensor.
 *
 * Read the measured data using `sht4x_read_measurement` function. Wait for
 * a specified time as defined in the driver header file, depending on the
 * measurement type.
 *
 * @param[in] device The `Sht4xDevice` struct containing the I2C address and
 * communication functions.
 * @param[in] command One of the `Sht4xMeasurement` enum values, specifying the
 * measurement type.
 *
 * @retval `SHT4X_SUCCESS` The measurement was started successfully.
 * @retval `SHT4X_I2C_ERROR` I2C communication error occurred.
 * @retval `SHT4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Sht4xStatus sht4x_start_measurement(Sht4xDevice *device, Sht4xMeasurement command)
{
	SHT4X_CHECK_STATUS(sht4x_check_device(device));

	if (device->i2c_write(device->i2c_address, (uint8_t[]){command}, SHT4X_I2C_CMD_LENGHT) != 0)
		return SHT4X_I2C_ERROR;
	return SHT4X_SUCCESS;
}

/**
 * @brief Reads the measurement data from the SHT4x sensor.
 *
 * @param[in] device The `Sht4xDevice` struct containing the I2C address and
 * communication functions.
 * @param[out] data A pointer to a `Sht4xData` struct where the measurement data
 * will be stored.
 *
 * @retval `SHT4X_SUCCESS` The measurement data was read successfully.
 * @retval `SHT4X_I2C_ERROR` I2C communication error occurred.
 * @retval `SHT4X_POINTER_NULL` The `device` or `data` pointer is `NULL`.
 * @retval `SHT4X_CRC_FAILURE` CRC verification failed.
 */
Sht4xStatus sht4x_read_measurement(Sht4xDevice *device, Sht4xData *data)
{
	SHT4X_CHECK_STATUS(sht4x_check_device(device));
	SHT4X_CHECK_NULL(data);

	uint8_t rx_data[SHT4X_I2C_CMD_MEASUREMENT_LENGHT];

	if (device->i2c_read(device->i2c_address, rx_data, SHT4X_I2C_CMD_MEASUREMENT_LENGHT) != 0)
		return SHT4X_I2C_ERROR;

	SHT4X_CHECK_STATUS(sht4x_check_checksum(device, (uint8_t[]){rx_data[0], rx_data[1]}, rx_data[2]));
	SHT4X_CHECK_STATUS(sht4x_check_checksum(device, (uint8_t[]){rx_data[3], rx_data[4]}, rx_data[5]));

	uint32_t temperature_merged = (rx_data[0] << 8) | rx_data[1];
	uint32_t humidity_merged = (rx_data[3] << 8) | rx_data[4];

	data->temperature = ((21875 * temperature_merged) >> 13) - 45000;
	data->humidity = ((15625 * humidity_merged) >> 13) - 6000;

	return SHT4X_SUCCESS;
}

/**
 * @brief Requests the serial number from the SHT4x sensor.
 *
 * This function sends the serial number request command to the SHT4x sensor.
 * The serial number can then be read using `sht4x_read_serial_number` after
 * `SHT4X_SERIAL_NUMBER_PERIOD_US` delay.
 *
 * @param[in] device The `Sht4xDevice` struct containing the I2C address and
 * communication functions.
 *
 * @retval `SHT4X_SUCCESS` The serial number request was sent successfully.
 * @retval `SHT4X_I2C_ERROR` I2C communication error occurred.
 * @retval `SHT4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Sht4xStatus sht4x_request_serial_number(Sht4xDevice *device)
{
	SHT4X_CHECK_STATUS(sht4x_check_device(device));

	if (device->i2c_write(device->i2c_address, (uint8_t[]){SHT4X_I2C_CMD_READ_SERIAL_NUMBER}, SHT4X_I2C_CMD_LENGHT) != 0)
		return SHT4X_I2C_ERROR;

	return SHT4X_SUCCESS;
}

/**
 * @brief Reads the serial number from the SHT4x sensor.
 *
 * This function call must be preceded by a call to `sht4x_request_serial_number`
 * with a minimum of `SHT4X_SERIAL_NUMBER_PERIOD_US` delay.
 *
 * @param[in] device The `Sht4xDevice` struct containing the I2C address and
 * communication functions.
 * @param[out] serial_number Pointer to a `uint32_t` where the serial number
 * will be stored.
 *
 * @retval `SHT4X_SUCCESS` The serial number was read successfully.
 * @retval `SHT4X_I2C_ERROR` An I2C communication error occurred.
 * @retval `SHT4X_POINTER_NULL` The `device` or `serial_number` pointer is `NULL`.
 * @retval `SHT4X_CRC_FAILURE` CRC verification failed.
 */
Sht4xStatus sht4x_read_serial_number(Sht4xDevice *device, uint32_t *serial_number)
{
	SHT4X_CHECK_STATUS(sht4x_check_device(device));
	SHT4X_CHECK_NULL(serial_number);

	uint8_t data[SHT4X_I2C_CMD_SERIAL_NUMBER_LENGHT];
	if (device->i2c_read(device->i2c_address, data, SHT4X_I2C_CMD_SERIAL_NUMBER_LENGHT) != 0)
		return SHT4X_I2C_ERROR;

	uint32_t serial_number_ = (data[0] << 24) | (data[1] << 16) | (data[3] << 8) | data[4];

	SHT4X_CHECK_STATUS(sht4x_check_checksum(device, (uint8_t[]){data[0], data[1]}, data[2]));
	SHT4X_CHECK_STATUS(sht4x_check_checksum(device, (uint8_t[]){data[3], data[4]}, data[5]));

	*serial_number = serial_number_;

	return SHT4X_SUCCESS;
}

/**
 * @brief Resets the SHT4x device via the soft reset command.
 *
 * @param[in] device The `Sht4xDevice` struct containing the I2C address and
 * communication functions.
 *
 * @retval `SHT4X_SUCCESS` The soft reset command was sent successfully.
 * @retval `SHT4X_I2C_ERROR` I2C communication error occurred.
 * @retval `SHT4X_POINTER_NULL` The `device` pointer is `NULL`.
 */
Sht4xStatus sht4x_soft_reset(Sht4xDevice *device)
{
	SHT4X_CHECK_STATUS(sht4x_check_device(device));

	if (device->i2c_write(device->i2c_address, (uint8_t[]){SHT4X_I2C_CMD_SOFT_RESET}, SHT4X_I2C_CMD_LENGHT) != 0)
		return SHT4X_I2C_ERROR;
	return SHT4X_SUCCESS;
}

/**
 * @brief Verifies the checksum for a given data array using the SHT4X device's CRC function.
 *
 * This function calculates the CRC-8 checksum of the provided 2-byte data array and compares it
 * with the given checksum. If the SHT4X device has a custom CRC calculation function defined, it uses that function.
 * Otherwise, it falls back to the default software CRC-8 algorithm.
 *
 * @param[in] device The `Sht4xDevice` struct containing the CRC calculation function.
 * @param[in] data A 2-byte array for which the checksum is to be verified.
 * @param[in] checksum A `uint8_t` containing the expected checksum.
 *
 * @retval `SHT4X_SUCCESS` Checksum verified successfully.
 * @retval `SHT4X_CRC_FAILURE` CRC verification using the device's function failed.
 */
static Sht4xStatus sht4x_check_checksum(Sht4xDevice *device, uint8_t data[2], uint8_t checksum)
{
	uint8_t calculated_checksum = 0;
	SHT4X_CHECK_STATUS(sht4x_calculate_checksum(device, data, &calculated_checksum));

	if (calculated_checksum != checksum)
		return SHT4X_CRC_FAILURE;

	return SHT4X_SUCCESS;
}

/**
 * @brief Calculates the checksum for a given data array using the SHT4X device's CRC function.
 *
 * This function calculates the CRC-8 checksum of the provided 2-byte data array.
 * If the SHT4X device has a custom CRC calculation function defined, it uses that function.
 * Otherwise, it falls back to the default software CRC-8 algorithm.
 *
 * @param[in] device The `Sht4xDevice` struct containing the CRC calculation function.
 * @param[in] data A 2-byte array for which the checksum is to be calculated.
 * @param[out] checksum A pointer to a `uint8_t` where the calculated checksum is stored.
 *
 * @retval `SHT4X_SUCCESS` Checksum calculated successfully.
 * @retval `SHT4X_CRC_FAILURE` CRC calculation using the device's function failed.
 */
static Sht4xStatus sht4x_calculate_checksum(Sht4xDevice *device, uint8_t data[2], uint8_t *checksum)
{
	if (device->calculate_crc != NULL)
	{
		if (device->calculate_crc(data, 2, SHT4X_CRC8_POLYNOMIAL, checksum) != 0)
			return SHT4X_CRC_FAILURE;
	}
	else
		*checksum = sht4x_calculate_checksum_default(data);

	return SHT4X_SUCCESS;
}

/**
 * @brief Calculates the CRC-8 checksum of two bytes of data.
 *
 * This function implements the CRC-8 algorithm as specified in the SHT4x
 * datasheet. It takes two bytes of data and returns the calculated CRC-8
 * checksum.
 *
 * @param[in] data Two bytes of data to calculate the CRC-8 checksum for.
 *
 * @return The calculated CRC-8 checksum.
 */
static uint8_t sht4x_calculate_checksum_default(uint8_t data[2])
{
	uint8_t crc = SHT4X_CRC8_INIT;
	for (int i = 0; i < 2; i++)
	{
		crc ^= data[i];
		for (uint8_t bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ SHT4X_CRC8_POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}
	return crc;
}

/**
 * @brief Checks whether the provided `Sht4xDevice` struct contains valid pointers.
 *
 * Returns -1 if the `device`pointer is NULL, or if the `i2c_write`, `i2c_read` or
 * `delay_ms` function pointers are `NULL`.
 *
 * WARNING: This function does not check whether the function pointers are
 * pointing to valid functions.
 *
 * @param[in] device The `Sht4xDevice` struct to be checked.
 *
 * @retval `SHT4X_SUCCESS` The `Sht4xDevice` struct pointers are valid.
 * @retval `SHT4X_POINTER_NULL` One of the pointers is `NULL`.
 */
static Sht4xStatus sht4x_check_device(Sht4xDevice *device)
{
	if (device == NULL)
		return SHT4X_POINTER_NULL;
	if (device->i2c_write == NULL || device->i2c_read == NULL)
		return SHT4X_POINTER_NULL;
	return SHT4X_SUCCESS;
}