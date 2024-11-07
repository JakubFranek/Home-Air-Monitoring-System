#include <stdint.h>

#include "sht4x.h"
#include "main.h"

#define SHT4X_CHECK_STATUS(expr)     \
	do                               \
	{                                \
		Sht4xStatus retval = expr;   \
		if (retval != SHT4X_SUCCESS) \
		{                            \
			return retval;           \
		}                            \
	} while (0)

Sht4xStatus sht4x_send_command(Sht4xDevice *device, uint8_t command)
{
	if (device->i2c_write(device->i2c_address, command) != 0)
		return SHT4X_I2C_ERROR;
	return SHT4X_SUCCESS;
}

Sht4xStatus sht4x_read_raw_measurement(Sht4xDevice *device, Sht4xRawData *raw_data)
{
	uint8_t data[6];

	if (device->i2c_read(device->i2c_address, data, 6) != 0)
		return SHT4X_I2C_ERROR;

	raw_data->temperature_raw[0] = data[0];
	raw_data->temperature_raw[1] = data[1];
	raw_data->temperature_crc = data[2];
	raw_data->humidity_raw[0] = data[3];
	raw_data->humidity_raw[1] = data[4];
	raw_data->humidity_crc = data[5];

	return SHT4X_SUCCESS;
}

Sht4xStatus sht4x_convert_raw_data(Sht4xDevice *device, Sht4xRawData *raw_data, Sht4xData *data)
{
	uint32_t temperature_merged = (raw_data->temperature_raw[0] << 8) | raw_data->temperature_raw[1];
	uint32_t humidity_merged = (raw_data->humidity_raw[0] << 8) | raw_data->humidity_raw[1];

	data->temperature = ((21875 * temperature_merged) >> 13) - 45000;
	data->humidity = ((15625 * humidity_merged) >> 13) - 6000;

	return SHT4X_SUCCESS;
}

Sht4xStatus sht4x_read_and_check_measurement(Sht4xDevice *device, Sht4xData *data)
{
	Sht4xRawData raw_data;

	SHT4X_CHECK_STATUS(sht4x_read_raw_measurement(device, &raw_data));
	SHT4X_CHECK_STATUS(sht4x_check_crc(device, &raw_data));

	return sht4x_convert_raw_data(device, &raw_data, data);
}

Sht4xStatus sht4x_check_crc(Sht4xDevice *device, Sht4xRawData *raw_data)
{
	uint8_t temperature_crc;
	uint8_t humidity_crc;

	device->calculate_crc(raw_data->temperature_raw, 2, SHT4X_CRC8_POLYNOMIAL, &temperature_crc);
	device->calculate_crc(raw_data->humidity_raw, 2, SHT4X_CRC8_POLYNOMIAL, &humidity_crc);

	if (temperature_crc != raw_data->temperature_crc || humidity_crc != raw_data->humidity_crc)
		return SHT4X_CRC_FAILURE;

	return SHT4X_SUCCESS;
}

Sht4xStatus sht4x_read_serial_number(Sht4xDevice *device, Sht4xSerialNumber *serial_number)
{
	uint8_t data[6];

	if (device->i2c_read(device->i2c_address, data, 6) != 0)
		return SHT4X_I2C_ERROR;

	serial_number->serial_msb = (data[0] << 8) + data[1];
	serial_number->msb_crc = data[2];
	serial_number->serial_lsb = (data[3] << 8) + data[4];
	serial_number->lsb_crc = data[5];

	return SHT4X_SUCCESS;
}
