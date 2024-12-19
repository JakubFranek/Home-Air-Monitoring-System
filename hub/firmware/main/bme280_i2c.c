#include <stddef.h> // definitions of NULL, size_t
#include "bme280_i2c.h"

/* --- Private defines --- */

#define BME280_REG_HUM_LSB_ADDRESS 0xFE
#define BME280_REG_HUM_MSB_ADDRESS 0xFD
#define BME280_REG_TEMP_XLSB_ADDRESS 0xFC
#define BME280_REG_TEMP_LSB_ADDRESS 0xFB
#define BME280_REG_TEMP_MSB_ADDRESS 0xFA
#define BME280_REG_PRESS_XLSB_ADDRESS 0xF9
#define BME280_REG_PRESS_LSB_ADDRESS 0xF8
#define BME280_REG_PRESS_MSB_ADDRESS 0xF7
#define BME280_REG_CONFIG_ADDRESS 0xF5
#define BME280_REG_CTRL_MEAS_ADDRESS 0xF4
#define BME280_REG_STATUS_ADDRESS 0xF3
#define BME280_REG_CTRL_HUM_ADDRESS 0xF2
#define BME280_REG_RESET_ADDRESS 0xE0
#define BME280_REG_ID_ADDRESS 0xD0
#define BME280_REG_CALIB_00_ADDRESS 0x88 // 0x88 - 0xA1
#define BME280_REG_CALIB_26_ADDRESS 0xE1 // 0xE1 - 0xF0

#define BME280_ID 0x60 // expected content of BME280_REG_ID
#define BME280_RESET_VALUE 0xB6

/* Set to '1' when NVM data is being copied to image registers (after power-on-reset
and before every conversion)*/
#define BME280_REG_STATUS_IM_UPDATE (1U << 0)
/* Set to '1' when a measurement is in progress*/
#define BME280_REG_STATUS_MEASURING (1U << 3)

#define BME280_REG_CTRL_HUM_OSRS_H_POS 0
#define BME280_REG_CTRL_HUM_OSRS_H_MASK (0b111 << 0)
#define BME280_REG_CTRL_MEAS_MODE_POS 0
#define BME280_REG_CTRL_MEAS_MODE_MASK (0b11 << 0)
#define BME280_REG_CTRL_MEAS_OSRS_P_POS 2
#define BME280_REG_CTRL_MEAS_OSRS_P_MASK (0b111 << 2)
#define BME280_REG_CTRL_MEAS_OSRS_T_POS 5
#define BME280_REG_CTRL_MEAS_OSRS_T_MASK (0b111 << 5)

/* Enables the 3-wire SPI interface*/
#define BME280_REG_CONFIG_SPI3W_EN (1U << 0)

#define BME280_REG_CONFIG_FILTER_POS 2
#define BME280_REG_CONFIG_FILTER_MASK (0b111 << 2)
#define BME280_REG_CONFIG_T_SB_POS 5
#define BME280_REG_CONFIG_T_SB_MASK (0b111 << 5)
#define BME280_REG_PRESS_XLSB_POS 4
#define BME280_REG_PRESS_XLSB_MASK (0b1111 << 4)
#define BME280_REG_TEMP_XLSB_POS 4
#define BME280_REG_TEMP_XLSB_MASK (0b1111 << 4)

/* --- Private macros --- */

#ifndef SET_BIT
#define SET_BIT(reg, bit) ((reg) |= (bit))
#endif

#ifndef CLEAR_BIT
#define CLEAR_BIT(reg, bit) ((reg) &= ~(bit))
#endif

#ifndef READ_BIT
#define READ_BIT(reg, bit) ((reg) & (bit))
#endif

#ifndef CLEAR_FIELD
#define CLEAR_FIELD(field, mask) ((field) &= ~(mask))
#endif

#ifndef SET_FIELD
#define SET_FIELD(field, new_val, mask, pos) ((field) |= (((new_val) << (pos)) & (mask)))
#endif

#ifndef READ_FIELD
#define READ_FIELD(field, mask) ((field) & (mask))
#endif

/**
 * Error-checking macro: if `expr` is not `BME280_SUCCESS`, this macro returns `expr`,
 * exiting the function where this macro was used immediately.
 */
#define BME280_CHECK_STATUS(expr)     \
    do                                \
    {                                 \
        Bme280Status retval = expr;   \
        if (retval != BME280_SUCCESS) \
        {                             \
            return retval;            \
        }                             \
    } while (0)

/**
 * Error-checking macro: if `expr` is `NULL`, this macro returns `BME280_POINTER_NULL`,
 * exiting the function where this macro was used immediately.
 */
#define BME280_CHECK_NULL(expr)         \
    do                                  \
    {                                   \
        if (expr == NULL)               \
        {                               \
            return BME280_POINTER_NULL; \
        }                               \
    } while (0)

/* --- Private function prototypes --- */

static Bme280Status bme280_check_device(Bme280Device *device);
static Bme280Status bme280_write_i2c_register(Bme280Device *device, uint8_t address, uint8_t value);
static Bme280Status bme280_read_i2c_register(Bme280Device *device, uint8_t address, uint8_t *data, uint8_t data_length);
static Bme280Status bme280_read_calibration_data(Bme280Device *device);
static int32_t bme280_compensate_T_int32(Bme280Device *device, int32_t adc_T);
static uint32_t bme280_compensate_P_int64(Bme280Device *device, int32_t adc_P);
static uint32_t bme280_compensate_H_int32(Bme280Device *device, int32_t adc_H);

/* --- Function definitions --- */

/**
 * @brief Initializes the BME280 sensor.
 *
 * This function reads and stores the calibration data and sets
 * the configuration of the sensor as specified in `device->config`.
 *
 * @param[in] device The `Bme280Device` struct containing the sensor's I2C
 *                   functions and address.
 *
 * @retval `BME280_SUCCESS` The initialization was successful.
 * @retval `BME280_I2C_ERROR` An I2C communication error occurred.
 * @retval `BME280_POINTER_NULL` The `device` pointer is `NULL`.
 */
Bme280Status bme280_init(Bme280Device *device)
{
    BME280_CHECK_STATUS(bme280_check_device(device));

    BME280_CHECK_STATUS(bme280_read_calibration_data(device));
    BME280_CHECK_STATUS(bme280_set_config(device, &device->config));

    return BME280_SUCCESS;
}

/**
 * @brief Resets the BME280 sensor.
 *
 * This function sends a reset command to the BME280 sensor via I2C,
 * which performs a soft reset of the sensor.
 *
 * @param[in] device The `Bme280Device` struct containing the sensor's
 *                   I2C functions and address.
 *
 * @retval `BME280_SUCCESS` The sensor was successfully reset.
 * @retval `BME280_I2C_ERROR` An I2C communication error occurred.
 * @retval `BME280_POINTER_NULL` The `device` pointer is `NULL`.
 */
Bme280Status bme280_reset(Bme280Device *device)
{
    BME280_CHECK_STATUS(bme280_check_device(device));
    BME280_CHECK_STATUS(bme280_write_i2c_register(device, BME280_REG_RESET_ADDRESS, BME280_RESET_VALUE));
    return BME280_SUCCESS;
}

/**
 * @brief Sets the configuration of the BME280 sensor.
 *
 * This function sets the temperature, pressure, and humidity
 * oversampling rates, standby time, filter, and SPI 3-wire interface mode of the
 * BME280 sensor.
 *
 * @param[in] device The `Bme280Device` struct containing the sensor's I2C
 *                   functions and address.
 * @param[in] config The `Bme280Config` struct containing the desired
 *                   configuration of the sensor.
 *
 * @retval `BME280_SUCCESS` The configuration was successfully set.
 * @retval `BME280_I2C_ERROR` An I2C communication error occurred.
 * @retval `BME280_POINTER_NULL` The `device` or `config` pointer is `NULL`.
 */
Bme280Status bme280_set_config(Bme280Device *device, Bme280Config *config)
{
    BME280_CHECK_STATUS(bme280_check_device(device));

    uint8_t config_value = 0;
    if (config->spi_3wire_enable)
        SET_BIT(config_value, BME280_REG_CONFIG_SPI3W_EN);
    SET_FIELD(config_value, config->filter, BME280_REG_CONFIG_FILTER_MASK, BME280_REG_CONFIG_FILTER_POS);
    SET_FIELD(config_value, config->standby_time, BME280_REG_CONFIG_T_SB_MASK, BME280_REG_CONFIG_T_SB_POS);
    BME280_CHECK_STATUS(bme280_write_i2c_register(device, BME280_REG_CONFIG_ADDRESS, config_value));

    uint8_t ctrl_meas_value = 0;
    SET_FIELD(ctrl_meas_value, config->pressure_oversampling, BME280_REG_CTRL_MEAS_OSRS_P_MASK,
              BME280_REG_CTRL_MEAS_OSRS_P_POS);
    SET_FIELD(ctrl_meas_value, config->temperature_oversampling, BME280_REG_CTRL_MEAS_OSRS_T_MASK,
              BME280_REG_CTRL_MEAS_OSRS_T_POS);
    BME280_CHECK_STATUS(bme280_write_i2c_register(device, BME280_REG_CTRL_MEAS_ADDRESS, ctrl_meas_value));

    uint8_t ctrl_hum = 0;
    SET_FIELD(ctrl_hum, config->humidity_oversampling, BME280_REG_CTRL_HUM_OSRS_H_MASK,
              BME280_REG_CTRL_HUM_OSRS_H_POS);
    BME280_CHECK_STATUS(bme280_write_i2c_register(device, BME280_REG_CTRL_HUM_ADDRESS, ctrl_hum));

    return BME280_SUCCESS;
}

/**
 * @brief Sets the operation mode of the BME280 sensor.
 *
 * @param[in] device The `Bme280Device` struct containing the sensor's I2C
 *                   functions and address.
 * @param[in] mode The desired operation mode of the sensor, specified as a
 *                 `Bme280Mode` enum value.
 *
 * @retval `BME280_SUCCESS` The mode was successfully set.
 * @retval `BME280_I2C_ERROR` An I2C communication error occurred.
 * @retval `BME280_POINTER_NULL` The `device` pointer is `NULL`.
 */
Bme280Status bme280_set_mode(Bme280Device *device, Bme280Mode mode)
{
    BME280_CHECK_STATUS(bme280_check_device(device));

    uint8_t ctrl_meas_value = 0;
    BME280_CHECK_STATUS(bme280_read_i2c_register(device, BME280_REG_CTRL_MEAS_ADDRESS, &ctrl_meas_value, 1));
    SET_FIELD(ctrl_meas_value, mode, BME280_REG_CTRL_MEAS_MODE_MASK, BME280_REG_CTRL_MEAS_MODE_POS);
    BME280_CHECK_STATUS(bme280_write_i2c_register(device, BME280_REG_CTRL_MEAS_ADDRESS, ctrl_meas_value));

    return BME280_SUCCESS;
}

/**
 * @brief Queries the sensor if a measurement is in progress.
 *
 * @param[in] device The `Bme280Device` struct containing the I2C functions and
 *                    the address of the sensor.
 * @param[out] measuring A boolean indicating whether a measurement is in progress.
 *
 * @retval `BME280_SUCCESS` The status was successfully read.
 * @retval `BME280_I2C_ERROR` An I2C communication error occurred.
 * @retval `BME280_POINTER_NULL` The `device` or `measuring` pointer is `NULL`.
 */
Bme280Status bme280_is_measuring(Bme280Device *device, bool *is_measuring)
{
    BME280_CHECK_STATUS(bme280_check_device(device));
    uint8_t status = 0;
    BME280_CHECK_STATUS(bme280_read_i2c_register(device, BME280_REG_STATUS_ADDRESS, &status, 1));

    *is_measuring = (status & BME280_REG_STATUS_MEASURING) != 0;

    return BME280_SUCCESS;
}

/**
 * @brief Reads raw sensor data and converts it to human-readable measurements.
 *
 * This function retrieves the raw temperature, pressure, and humidity data from the
 * BME280 sensor via I2C and compensates these values using the calibration data
 * stored in the `Bme280Device` struct. The compensated values are stored in the
 * `Bme280Data` struct.
 *
 * @param[in] device The `Bme280Device` struct containing the sensor configuration
 *                   and calibration data.
 * @param[out] data The `Bme280Data` struct where the compensated temperature,
 *                  pressure, and humidity data will be stored.
 *
 * @retval `BME280_SUCCESS` The data was successfully read and compensated.
 * @retval `BME280_I2C_ERROR` An I2C communication error occurred.
 * @retval `BME280_POINTER_NULL` The `device` or `data` pointer is `NULL`.
 */
Bme280Status bme280_read_measurement(Bme280Device *device, Bme280Data *data)
{
    BME280_CHECK_STATUS(bme280_check_device(device));

    int32_t adc_T, adc_P, adc_H;
    uint8_t rx_data[8];
    BME280_CHECK_STATUS(bme280_read_i2c_register(device, BME280_REG_PRESS_MSB_ADDRESS, rx_data, 8));

    adc_P = rx_data[0] << 12 | rx_data[1] << 4 | rx_data[2] >> 4;
    adc_T = rx_data[3] << 12 | rx_data[4] << 4 | rx_data[5] >> 4;
    adc_H = rx_data[6] << 8 | rx_data[7];

    data->temperature = bme280_compensate_T_int32(device, adc_T); // must be calculated first to update `t_fine`
    uint32_t pressure_256 = bme280_compensate_P_int64(device, adc_P);
    uint32_t humidity_1024 = bme280_compensate_H_int32(device, adc_H);

    data->pressure = (10 * pressure_256) / 256;
    data->humidity = (1000 * humidity_1024) / 1024;

    return BME280_SUCCESS;
}

/**
 * @brief Compensates the raw temperature readings from the sensor.
 *
 * This function uses the calibration data stored in the `Bme280Device` struct
 * to convert the raw temperature data (`adc_T`) into a compensated temperature value.
 * The result is returned as an unsigned 32-bit integer.
 *
 * @param[in] device The `Bme280Device` struct containing the calibration data.
 * @param[in] adc_T The raw temperature data read from the sensor.
 *
 * @return The compensated temperature value as an unsigned 32-bit integer. To convert
 * to DegC, divide the result by 100.
 */
static int32_t bme280_compensate_T_int32(Bme280Device *device, int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)device->calibration.dig_T1 << 1))) * ((int32_t)device->calibration.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)device->calibration.dig_T1)) * ((adc_T >> 4) - ((int32_t)device->calibration.dig_T1))) >> 12) *
            ((int32_t)device->calibration.dig_T3)) >>
           14;
    device->calibration.t_fine = var1 + var2;
    T = (device->calibration.t_fine * 5 + 128) >> 8;
    return T;
}

/**
 * @brief Compensates the raw pressure readings from the sensor.
 *
 * This function uses the calibration data stored in the `Bme280Device` struct
 * to convert the raw pressure data (`adc_P`) into a compensated pressure value.
 * The result is returned as an unsigned 32-bit integer.
 *
 * @param[in] device The `Bme280Device` struct containing the calibration data.
 * @param[in] adc_P The raw pressure data read from the sensor.
 *
 * @return The compensated pressure value as an unsigned 32-bit integer. To convert to
 * Pa, divide the result by 256.
 */
static uint32_t bme280_compensate_P_int64(Bme280Device *device, int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)device->calibration.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)device->calibration.dig_P6;
    var2 = var2 + ((var1 * (int64_t)device->calibration.dig_P5) << 17);
    var2 = var2 + (((int64_t)device->calibration.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)device->calibration.dig_P3) >> 8) + ((var1 * (int64_t)device->calibration.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)device->calibration.dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)device->calibration.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)device->calibration.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)device->calibration.dig_P7) << 4);
    return (uint32_t)p;
}

/**
 * @brief Compensates the raw humidity readings from the sensor.
 *
 * This function uses the calibration data stored in the `Bme280Device` struct
 * to convert the raw humidity data (`adc_H`) into a compensated humidity value.
 * The result is returned as an unsigned 32-bit integer.
 *
 * @param[in] device The `Bme280Device` struct containing the calibration data.
 * @param[in] adc_H The raw humidity data read from the sensor.
 *
 * @return The compensated humidity value as an unsigned 32-bit integer. To convert
 * to %RH, divide the result by 1024.
 */
static uint32_t bme280_compensate_H_int32(Bme280Device *device, int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (device->calibration.t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)device->calibration.dig_H4) << 20) - (((int32_t)device->calibration.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >>
                  15) *
                 (((((((v_x1_u32r *
                        ((int32_t)device->calibration.dig_H6)) >>
                       10) *
                      (((v_x1_u32r * ((int32_t)device->calibration.dig_H3)) >> 11) +
                       ((int32_t)32768))) >>
                     10) +
                    ((int32_t)2097152)) *
                       ((int32_t)device->calibration.dig_H2) +
                   8192) >>
                  14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)device->calibration.dig_H1)) >>
                              4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

/**
 * @brief Checks whether the provided `Bme280Device` struct contains valid pointers.
 *
 * Returns -1 if the `device`pointer is NULL, or if the `i2c_write` or `i2c_read`
 * function pointers are `NULL`.
 *
 * WARNING: This function does not check whether the function pointers are
 * pointing to valid functions.
 *
 * @param[in] device The `Bme280Device` struct to be checked.
 *
 * @retval `BME280_SUCCESS` The `Bme280Device` struct pointers are valid.
 * @retval `BME280_POINTER_NULL` One of the pointers is `NULL`.
 */
static Bme280Status bme280_check_device(Bme280Device *device)
{
    if (device == NULL)
        return BME280_POINTER_NULL;
    if (device->i2c_write == NULL || device->i2c_read == NULL)
        return BME280_POINTER_NULL;
    return BME280_SUCCESS;
}

/**
 * @brief Sends an I2C command to the sensor.
 *
 * @param[in] device The `Bme280Device` struct containing the I2C functions.
 * @param[in] command The command byte array to be sent.
 *
 * @retval `BME280_SUCCESS` The command was sent to the device.
 * @retval `BME280_I2C_ERROR` An I2C communication error occurred.
 * @retval `BME280_POINTER_NULL` The `device` pointer is `NULL`.
 */
static Bme280Status bme280_write_i2c_register(Bme280Device *device, uint8_t address, uint8_t value)
{
    if (device->i2c_write(device->address, (uint8_t[]){address, value}, 2) != 0)
        return BME280_I2C_ERROR;
    return BME280_SUCCESS;
}

/**
 * @brief Receives data from the sensor via I2C.
 *
 * @param[in] device The `Bme280Device` struct containing the I2C read function.
 * @param[out] data A pointer to the buffer where the received data will be stored.
 * @param[in] data_length The number of bytes to read from the sensor.
 *
 * @retval `BME280_SUCCESS` The data was successfully received.
 * @retval `BME280_I2C_ERROR` An I2C communication error occurred.
 */
static Bme280Status bme280_read_i2c_register(Bme280Device *device, uint8_t initial_address, uint8_t *data, uint8_t data_length)
{
    if (device->i2c_write(device->address, (uint8_t[]){initial_address}, 1) != 0)
        return BME280_I2C_ERROR;
    if (device->i2c_read(device->address, data, data_length) != 0)
        return BME280_I2C_ERROR;
    return BME280_SUCCESS;
}

/**
 * @brief Reads and stores the calibration data from the sensor.
 *
 * @param[in] device The `Bme280Device` struct containing the I2C functions and
 *                    the address of the sensor.
 *
 * @retval `BME280_SUCCESS` The calibration data was successfully read.
 * @retval `BME280_I2C_ERROR` An I2C communication error occurred.
 * @retval `BME280_POINTER_NULL` The `device` pointer is `NULL`.
 */
static Bme280Status bme280_read_calibration_data(Bme280Device *device)
{
    uint8_t calibration_data_1[25];
    uint8_t calibration_data_2[7];

    BME280_CHECK_STATUS(bme280_read_i2c_register(device, BME280_REG_CALIB_00_ADDRESS, calibration_data_1, 25));
    BME280_CHECK_STATUS(bme280_read_i2c_register(device, BME280_REG_CALIB_26_ADDRESS, calibration_data_2, 7));

    device->calibration.dig_T1 = (calibration_data_1[1] << 8) | calibration_data_1[0];
    device->calibration.dig_T2 = (calibration_data_1[3] << 8) | calibration_data_1[2];
    device->calibration.dig_T3 = (calibration_data_1[5] << 8) | calibration_data_1[4];
    device->calibration.dig_P1 = (calibration_data_1[7] << 8) | calibration_data_1[6];
    device->calibration.dig_P2 = (calibration_data_1[9] << 8) | calibration_data_1[8];
    device->calibration.dig_P3 = (calibration_data_1[11] << 8) | calibration_data_1[10];
    device->calibration.dig_P4 = (calibration_data_1[13] << 8) | calibration_data_1[12];
    device->calibration.dig_P5 = (calibration_data_1[15] << 8) | calibration_data_1[14];
    device->calibration.dig_P6 = (calibration_data_1[17] << 8) | calibration_data_1[16];
    device->calibration.dig_P7 = (calibration_data_1[19] << 8) | calibration_data_1[18];
    device->calibration.dig_P8 = (calibration_data_1[21] << 8) | calibration_data_1[20];
    device->calibration.dig_P9 = (calibration_data_1[23] << 8) | calibration_data_1[22];
    device->calibration.dig_H1 = calibration_data_2[24];
    device->calibration.dig_H2 = (calibration_data_2[1] << 8) | calibration_data_2[0];
    device->calibration.dig_H3 = calibration_data_2[2];
    device->calibration.dig_H4 = (calibration_data_2[3] << 4) | (calibration_data_2[4] & 0x0F);
    device->calibration.dig_H5 = ((calibration_data_2[5]) << 4) | (calibration_data_2[4] & 0xF0);
    device->calibration.dig_H6 = calibration_data_2[6];

    return BME280_SUCCESS;
}