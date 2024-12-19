#include <stddef.h> // define NULL

#include "sgp41.h"

/* --- Private defines --- */

#define SGP41_CRC8_POLYNOMIAL 0x31
#define SGP41_CMD_LEN 2              // bytes
#define SGP41_SERIAL_NUMBER_LENGTH 6 // bytes
#define SGP41_TEMP_RH_DATA_LEN 6     // bytes
#define SGP41_DEF_TEMP 0x66, 0x66, 0x93
#define SGP41_DEF_RH 0x80, 0x00, 0xA2

#define SGP41_CMD_EXEC_COND 0x26, 0x12
#define SGP41_CMD_MEAS_RAW 0x26, 0x19
#define SGP41_CMD_SELF_TEST 0x28, 0x0E
#define SGP41_CMD_HEATER_OFF 0x36, 0x15
#define SGP41_CMD_SERIAL_NO 0x36, 0x82

#define SGP41_RSP_LEN_COND 3       // bytes
#define SGP41_RSP_LEN_MEAS_RAW 6   // bytes
#define SGP41_RSP_LEN_SELF_TEST 3  // bytes
#define SGP41_RSP_LEN_HEATER_OFF 0 // bytes
#define SGP41_RSP_LEN_SERIAL_NO 9  // bytes

/* --- Private macros --- */

/**
 * Error-checking macro: if `expr` is not `SGP41_SUCCESS`, this macro returns `expr`,
 * exiting the function where this macro was used immediately.
 */
#define SGP41_CHECK_STATUS(expr)     \
    do                               \
    {                                \
        Sgp41Status retval = expr;   \
        if (retval != SGP41_SUCCESS) \
        {                            \
            return retval;           \
        }                            \
    } while (0)

/**
 * Error-checking macro: if `expr` is `NULL`, this macro returns `SGP41_POINTER_NULL`,
 * exiting the function where this macro was used immediately.
 */
#define SGP41_CHECK_NULL(expr)         \
    do                                 \
    {                                  \
        if (expr == NULL)              \
        {                              \
            return SGP41_POINTER_NULL; \
        }                              \
    } while (0)

/* --- Private function prototypes --- */

static Sgp41Status sgp41_check_checksum(Sgp41Device *device, uint8_t data[2], uint8_t checksum);
static Sgp41Status sgp41_calculate_checksum(Sgp41Device *device, uint8_t data[2], uint8_t *checksum);
static uint8_t sgp41_calculate_checksum_default(uint8_t data[2]);
static Sgp41Status sgp41_check_device(Sgp41Device *device);

/* --- Function definitions --- */

/**
 * @brief Initializes an `Sgp41Device` by setting up the VOC and NOx gas index
 *        algorithm parameters.
 *
 * Call once before using any other SGP41 driver function.
 *
 * @param[in] device Pointer to the `Sgp41Device` structure to be initialized.
 *
 * @retval `SGP41_SUCCESS` Initialization successful.
 */
Sgp41Status sgp41_initialize(Sgp41Device *device)
{
    SGP41_CHECK_STATUS(sgp41_check_device(device));

    GasIndexAlgorithm_init(&device->gia_voc, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    GasIndexAlgorithm_init(&device->gia_nox, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);
    return SGP41_SUCCESS;
}

/**
 * @brief Reads raw sensor data, verifies the CRC, and converts it to gas index
 *        values using the VOC and NOx gas index algorithms.
 *
 * @param[in] device Pointer to the `Sgp41Device` structure.
 * @param[out] data Pointer to the `Sgp41Data` structure to store the converted data.
 *
 * @retval `SGP41_SUCCESS` Command successful.
 * @retval `SGP41_I2C_ERROR` I2C error occured.
 * @retval `SGP41_CRC_FAILURE` CRC check failed.
 */
Sgp41Status sgp41_read_gas_indices(Sgp41Device *device, Sgp41Data *data)
{
    SGP41_CHECK_STATUS(sgp41_check_device(device));
    SGP41_CHECK_NULL(data);

    uint8_t rx_data[6];

    if (device->i2c_read(SGP41_I2C_ADDRESS, rx_data, SGP41_RSP_LEN_MEAS_RAW) != 0)
        return SGP41_I2C_ERROR;

    uint8_t sraw_voc[3];
    uint8_t sraw_nox[3];

    sraw_voc[0] = rx_data[0];
    sraw_voc[1] = rx_data[1];
    sraw_voc[2] = rx_data[2];
    sraw_nox[0] = rx_data[3];
    sraw_nox[1] = rx_data[4];
    sraw_nox[2] = rx_data[5];

    SGP41_CHECK_STATUS(sgp41_check_checksum(device, sraw_voc, sraw_voc[2]));
    SGP41_CHECK_STATUS(sgp41_check_checksum(device, sraw_nox, sraw_nox[2]));

    int32_t sraw_voc_32bit = (sraw_voc[0] << 8) + sraw_voc[1];
    int32_t sraw_nox_32bit = (sraw_nox[0] << 8) + sraw_nox[1];

    GasIndexAlgorithm_process(&device->gia_voc, sraw_voc_32bit, &data->voc_index);
    GasIndexAlgorithm_process(&device->gia_nox, sraw_nox_32bit, &data->nox_index);

    return SGP41_SUCCESS;
}

/**
 * @brief Retrieves the serial number from the SGP41 sensor.
 *
 * @param[in] device Pointer to the `Sgp41Device` structure.
 * @param[out] serial_number Pointer to the `uint64_t` variable to store the 48-bit serial number in.
 *
 * @retval `SGP41_SUCCESS` Command successful.
 * @retval `SGP41_I2C_ERROR` I2C error occured.
 * @retval `SGP41_CRC_FAILURE` CRC check failed.
 */
Sgp41Status sgp41_get_serial_number(Sgp41Device *device, uint64_t *serial_number)
{
    SGP41_CHECK_STATUS(sgp41_check_device(device));
    SGP41_CHECK_NULL(serial_number);

    if (device->i2c_write(SGP41_I2C_ADDRESS, (uint8_t[]){SGP41_CMD_SERIAL_NO}, SGP41_CMD_LEN) != 0)
        return SGP41_I2C_ERROR;

    uint8_t rx_data[9];

    if (device->i2c_read(SGP41_I2C_ADDRESS, rx_data, SGP41_RSP_LEN_SERIAL_NO) != 0)
        return SGP41_I2C_ERROR;

    for (uint8_t i = 0; i < SGP41_RSP_LEN_SERIAL_NO; i = i + 3)
        SGP41_CHECK_STATUS(sgp41_check_checksum(device, (uint8_t[]){rx_data[i], rx_data[i + 1]}, rx_data[i + 2]));

    uint64_t serial_number_ = 0;
    uint8_t input_index = 0, output_index = 0;
    while (output_index < SGP41_SERIAL_NUMBER_LENGTH)
    {
        if (input_index % 3 != 2)
        {
            serial_number_ |= rx_data[input_index] << (output_index * 8);
            output_index++;
        }
        input_index++;
    }
    *serial_number = serial_number_;

    return SGP41_SUCCESS;
}

/**
 * @brief Executes the conditioning command on the SGP41 sensor.
 *
 * WARNING: This function must be followed up by a measurement command or heater off command
 * within 10 seconds to prevent damage to the sensor.
 *
 * @param[in] device Pointer to the `Sgp41Device` structure.
 *
 * @retval `SGP41_SUCCESS` Command successful.
 * @retval `SGP41_I2C_ERROR` I2C error occured.
 */
Sgp41Status sgp41_execute_conditioning(Sgp41Device *device)
{
    SGP41_CHECK_STATUS(sgp41_check_device(device));

    uint8_t tx_data[8] = {SGP41_CMD_EXEC_COND, SGP41_DEF_RH, SGP41_DEF_TEMP};

    if (device->i2c_write(SGP41_I2C_ADDRESS, tx_data, SGP41_CMD_LEN) != 0)
        return SGP41_I2C_ERROR;
    return SGP41_SUCCESS;
}

/**
 * @brief Writes the measurement command and optional temperature and humidity
 *        values to the SGP41 sensor.
 *
 * To read the raw sensor data, wait for 50 ms and call `sgp41_read_raw_signals`.
 *
 * @param[in] device Pointer to the `Sgp41Device` structure.
 * @param[in] temp_celsius Optional pointer to the temperature value in Celsius degrees.
 * @param[in] rel_hum_pct Optional pointer to the relative humidity value in percent.
 *
 * @retval `SGP41_SUCCESS` Command successful.
 * @retval `SGP41_I2C_ERROR` I2C error occured.
 */
Sgp41Status sgp41_measure_raw_signals(Sgp41Device *device, float *temp_celsius, float *rel_hum_pct)
{
    SGP41_CHECK_STATUS(sgp41_check_device(device));

    uint8_t tx_data[8] = {SGP41_CMD_MEAS_RAW, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rel_hum_data[3] = {SGP41_DEF_RH};
    uint8_t temp_data[3] = {SGP41_DEF_TEMP};

    uint16_t temp_ticks;
    uint16_t rel_hum_ticks;

    if (temp_celsius != NULL)
    {
        temp_ticks = (uint16_t)((*temp_celsius + 45) * 65535 / 175);
        temp_data[0] = (uint8_t)((temp_ticks >> 8) & 0xFF);
        temp_data[1] = (uint8_t)(temp_ticks & 0xFF);
        temp_data[2] = sgp41_calculate_checksum_default((uint8_t[]){temp_data[0], temp_data[1]});
    }

    if (rel_hum_pct != NULL)
    {
        rel_hum_ticks = (uint16_t)(*rel_hum_pct * 65535 / 100);
        rel_hum_data[0] = (uint8_t)((rel_hum_ticks >> 8) & 0xFF);
        rel_hum_data[1] = (uint8_t)(rel_hum_ticks & 0xFF);
        rel_hum_data[2] = sgp41_calculate_checksum_default((uint8_t[]){rel_hum_data[0], rel_hum_data[1]});
    }

    tx_data[2] = rel_hum_data[0];
    tx_data[3] = rel_hum_data[1];
    tx_data[4] = rel_hum_data[2];
    tx_data[5] = temp_data[0];
    tx_data[6] = temp_data[1];
    tx_data[7] = temp_data[2];

    if (device->i2c_write(SGP41_I2C_ADDRESS, tx_data, SGP41_CMD_LEN + SGP41_TEMP_RH_DATA_LEN) != 0)
        return SGP41_I2C_ERROR;
    return SGP41_SUCCESS;
}

/**
 * @brief Turns the SGP41 sensor's heater off.
 *
 * This function sends the command to turn the heater off to the SGP41 sensor.
 * The sensor enters idle mode.
 *
 * @param[in] device Pointer to the `Sgp41Device` structure.
 *
 * @retval `SGP41_SUCCESS` Command successful.
 * @retval `SGP41_I2C_ERROR` I2C error occured.
 */
Sgp41Status sgp41_turn_heater_off(Sgp41Device *device)
{
    SGP41_CHECK_STATUS(sgp41_check_device(device));

    if (device->i2c_write(SGP41_I2C_ADDRESS, (uint8_t[]){SGP41_CMD_HEATER_OFF}, SGP41_CMD_LEN) != 0)
        return SGP41_I2C_ERROR;
    return SGP41_SUCCESS;
}

/**
 * @brief Executes the self-test command on the SGP41 sensor.
 *
 * This function sends the self-test command to the SGP41 sensor. The result
 * of the self-test can be evaluated using the `sgp41_evaluate_self_test
 * function`, after a minimum interval of 320 ms.
 *
 * @param[in] device Pointer to the Sgp41Device structure.
 *
 * @retval `SGP41_SUCCESS` Command successful.
 * @retval `SGP41_I2C_ERROR` I2C error occured.
 */
Sgp41Status sgp41_execute_self_test(Sgp41Device *device)
{
    SGP41_CHECK_STATUS(sgp41_check_device(device));

    if (device->i2c_write(SGP41_I2C_ADDRESS, (uint8_t[]){SGP41_CMD_SELF_TEST}, SGP41_CMD_LEN) != 0)
        return SGP41_I2C_ERROR;
    return SGP41_SUCCESS;
}

/**
 * @brief Evaluate the result of the SGP41 sensor's self-test.
 *
 * Reads the self-test result from the SGP41 sensor and verifies the integrity of
 * the data using CRC.
 *
 * Call this function >320 ms after a call to `sgp41_execute_self_test`.
 *
 * @param[in] device Pointer to an `Sgp41Device` structure representing the sensor instance.
 *
 * @retval `SGP41_SUCCESS` Self-test completed successfully.
 * @retval `SGP41_I2C_ERROR` I2C communication error occurred.
 * @retval `SGP41_CRC_FAILURE` CRC verification of the received data failed.
 * @retval `SGP41_SELF_TEST_FAILURE` The self-test indicated a sensor failure.
 */
Sgp41Status sgp41_evaluate_self_test(Sgp41Device *device)
{
    SGP41_CHECK_STATUS(sgp41_check_device(device));

    uint8_t rx_data[3];
    uint8_t crc_check;

    if (device->i2c_read(SGP41_I2C_ADDRESS, rx_data, SGP41_RSP_LEN_SELF_TEST) != 0)
        return SGP41_I2C_ERROR;

    if (device->calculate_crc != NULL)
        device->calculate_crc((uint8_t[]){rx_data[0], rx_data[1]}, 2, SGP41_CRC8_POLYNOMIAL, &crc_check);
    else
        crc_check = sgp41_calculate_checksum_default((uint8_t[]){rx_data[0], rx_data[1]});

    if (crc_check != rx_data[2])
        return SGP41_CRC_FAILURE;

    if ((rx_data[1] & 0x03) != 0)
        return SGP41_SELF_TEST_FAILURE;

    return SGP41_SUCCESS;
}

/**
 * @brief Verifies the checksum for a given data array using the SGP41 device's CRC function.
 *
 * This function calculates the CRC-8 checksum of the provided 2-byte data array and compares it
 * with the given checksum. If the SGP41 device has a custom CRC calculation function defined, it uses that function.
 * Otherwise, it falls back to the default software CRC-8 algorithm.
 *
 * @param[in] device The `SGP41Device` struct containing the CRC calculation function.
 * @param[in] data A 2-byte array for which the checksum is to be verified.
 * @param[in] checksum A `uint8_t` containing the expected checksum.
 *
 * @retval `SGP41_SUCCESS` Checksum verified successfully.
 * @retval `SGP41_CRC_FAILURE` CRC verification using the device's function failed.
 */
static Sgp41Status sgp41_check_checksum(Sgp41Device *device, uint8_t data[2], uint8_t checksum)
{
    uint8_t calculated_checksum = 0;
    SGP41_CHECK_STATUS(sgp41_calculate_checksum(device, data, &calculated_checksum));

    if (calculated_checksum != checksum)
        return SGP41_CRC_FAILURE;

    return SGP41_SUCCESS;
}

/**
 * @brief Calculates the checksum for a given data array using the SGP41 device's CRC function.
 *
 * This function calculates the CRC-8 checksum of the provided 2-byte data array.
 * If the SGP41 device has a custom CRC calculation function defined, it uses that function.
 * Otherwise, it falls back to the default software CRC-8 algorithm.
 *
 * @param[in] device The `SGP41Device` struct containing the CRC calculation function.
 * @param[in] data A 2-byte array for which the checksum is to be calculated.
 * @param[out] checksum A pointer to a `uint8_t` where the calculated checksum is stored.
 *
 * @retval `SGP41_SUCCESS` Checksum calculated successfully.
 * @retval `SGP41_CRC_FAILURE` CRC calculation using the device's function failed.
 */
static Sgp41Status sgp41_calculate_checksum(Sgp41Device *device, uint8_t data[2], uint8_t *checksum)
{
    if (device->calculate_crc != NULL)
    {
        if (device->calculate_crc(data, 2, SGP41_CRC8_POLYNOMIAL, checksum) != 0)
            return SGP41_CRC_FAILURE;
    }
    else
        *checksum = sgp41_calculate_checksum_default(data);

    return SGP41_SUCCESS;
}

/**
 * @brief Calculates the CRC-8 checksum of two bytes of data.
 *
 * This function implements the CRC-8 algorithm as specified in the SGP41
 * datasheet. It takes two bytes of data and returns the calculated CRC-8
 * checksum.
 *
 * @param[in] data Two bytes of data to calculate the CRC-8 checksum for.
 *
 * @return The calculated CRC-8 checksum.
 */
static uint8_t sgp41_calculate_checksum_default(uint8_t data[2])
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ SGP41_CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Checks whether the provided `Sgp41Device` struct contains valid pointers.
 *
 * Returns -1 if the `device`pointer is NULL, or if the `i2c_write`, `i2c_read` or
 * `delay_ms` function pointers are `NULL`.
 *
 * WARNING: This function does not check whether the function pointers are
 * pointing to valid functions.
 *
 * @param[in] device The `Sgp41Device` struct to be checked.
 *
 * @retval `SGP41_SUCCESS` The `Sgp41Device` struct pointers are valid.
 * @retval `SGP41_POINTER_NULL` One of the pointers is `NULL`.
 */
static Sgp41Status sgp41_check_device(Sgp41Device *device)
{
    if (device == NULL)
        return SGP41_POINTER_NULL;
    if (device->i2c_write == NULL || device->i2c_read == NULL)
        return SGP41_POINTER_NULL;
    return SGP41_SUCCESS;
}