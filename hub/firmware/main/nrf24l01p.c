#include "nrf24l01p.h"
#include <stddef.h> // definitions of NULL, size_t

/*----------- nRF24L01+ Commands -----------*/

#define NRF24L01P_CMD_R_REGISTER 0b00000000
#define NRF24L01P_CMD_W_REGISTER 0b00100000
#define NRF24L01P_CMD_R_RX_PAYLOAD 0b01100001
#define NRF24L01P_CMD_W_TX_PAYLOAD 0b10100000
#define NRF24L01P_CMD_FLUSH_TX 0b11100001
#define NRF24L01P_CMD_FLUSH_RX 0b11100010
#define NRF24L01P_CMD_REUSE_TX_PL 0b11100011
#define NRF24L01P_CMD_R_RX_PL_WID 0b01100000
#define NRF24L01P_CMD_W_ACK_PAYLOAD 0b10101000
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK 0b10110000
#define NRF24L01P_CMD_NOP 0b11111111

/* ------------------------------ Macros ------------------------------ */
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
 * Error-checking macro: if `expr` returns anything other than `NRF24L01P_SUCCESS`,
 * this macro returns that return value, exiting the function where this macro was used immediately.
 */
#define NRF24L01P_CHECK_STATUS(expr)     \
	do                                   \
	{                                    \
		Nrf24l01pStatus retval = expr;   \
		if (retval != NRF24L01P_SUCCESS) \
		{                                \
			return retval;               \
		}                                \
	} while (0)

/**
 * Error-checking macro: if `expr` is `NULL`, this macro returns `NRF24L01P_POINTER_NULL`,
 * exiting the function where this macro was used immediately.
 */
#define NRF24L01P_CHECK_NULL(expr)         \
	do                                     \
	{                                      \
		if (expr == NULL)                  \
		{                                  \
			return NRF24L01P_POINTER_NULL; \
		}                                  \
	} while (0)

/* ------------------------------ Module variables ------------------------------ */

Nrf24l01p8bitRegRstVals reg_rst_vals_8bit[] = {
	{NRF24L01P_REG_CONFIG, NRF24L01P_REG_CONFIG_RSTVAL},
	{NRF24L01P_REG_EN_AA, NRF24L01P_REG_EN_AA_RSTVAL},
	{NRF24L01P_REG_EN_RXADDR, NRF24L01P_REG_EN_RXADDR_RSTVAL},
	{NRF24L01P_REG_SETUP_AW, NRF24L01P_REG_SETUP_AW_RSTVAL},
	{NRF24L01P_REG_SETUP_RETR, NRF24L01P_REG_SETUP_RETR_RSTVAL},
	{NRF24L01P_REG_RF_CH, NRF24L01P_REG_RF_CH_RSTVAL},
	{NRF24L01P_REG_RF_SETUP, NRF24L01P_REG_RF_SETUP_RSTVAL},
	{NRF24L01P_REG_STATUS, NRF24L01P_REG_STATUS_RSTVAL},
	{NRF24L01P_REG_RX_ADDR_P2, NRF24L01P_REG_RX_ADDR_P2_RSTVAL},
	{NRF24L01P_REG_RX_ADDR_P3, NRF24L01P_REG_RX_ADDR_P3_RSTVAL},
	{NRF24L01P_REG_RX_ADDR_P4, NRF24L01P_REG_RX_ADDR_P4_RSTVAL},
	{NRF24L01P_REG_RX_ADDR_P5, NRF24L01P_REG_RX_ADDR_P5_RSTVAL},
	{NRF24L01P_REG_RX_PW_P0, NRF24L01P_REG_RX_PW_P0_RSTVAL},
	{NRF24L01P_REG_RX_PW_P1, NRF24L01P_REG_RX_PW_P1_RSTVAL},
	{NRF24L01P_REG_RX_PW_P2, NRF24L01P_REG_RX_PW_P2_RSTVAL},
	{NRF24L01P_REG_RX_PW_P3, NRF24L01P_REG_RX_PW_P3_RSTVAL},
	{NRF24L01P_REG_RX_PW_P4, NRF24L01P_REG_RX_PW_P4_RSTVAL},
	{NRF24L01P_REG_RX_PW_P5, NRF24L01P_REG_RX_PW_P5_RSTVAL},
	{NRF24L01P_REG_FIFO_STATUS, NRF24L01P_REG_FIFO_STATUS_RSTVAL},
	{NRF24L01P_REG_DYNPD, NRF24L01P_REG_DYNPD_RSTVAL},
	{NRF24L01P_REG_FEATURE, NRF24L01P_REG_FEATURE_RSTVAL}};

Nrf24l01p40bitRegRstVals reg_rst_vals_40bit[] = {
	{NRF24L01P_REG_RX_ADDR_P0, NRF24L01P_REG_RX_ADDR_P0_RSTVAL},
	{NRF24L01P_REG_RX_ADDR_P1, NRF24L01P_REG_RX_ADDR_P1_RSTVAL},
	{NRF24L01P_REG_TX_ADDR, NRF24L01P_REG_TX_ADDR_RSTVAL},
};

/* ------------------------------ Static functions ------------------------------ */

/**
 * @brief Reads a single byte from a register in the nRF24L01+ chip.
 *
 * @param[in]  device  A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[in]  address The address of the register to read from.
 * @param[out] value   The address of a `uint8_t` to store the read value in.
 *
 * @retval `NRF24L01P_SUCCESS`		SPI transfer succeeded.
 * @retval `NRF24L01P_SPI_ERROR`	SPI transfer failed.
 */
static Nrf24l01pStatus read_register(Nrf24l01pDevice *device, uint8_t address, uint8_t *value)
{
	uint8_t command = NRF24L01P_CMD_R_REGISTER | address;
	uint8_t tx_data[2] = {command, 0x00};
	uint8_t rx_data[2];
	int8_t spi_status;

	device->interface.set_cs(0);
	spi_status = device->interface.spi_tx_rx(tx_data, rx_data, 2);
	device->interface.set_cs(1);

	if (spi_status != 0)
		return NRF24L01P_SPI_ERROR;

	*value = rx_data[1];

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Reads multiple bytes from a register in the nRF24L01+ chip.
 *
 * @param[in]  device  A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[in]  address The address of the register to read from.
 * @param[out] value   The address of a `uint64_t` to store the read value in.
 * @param[in]  length  The number of bytes to read from the register.
 *
 * @retval `NRF24L01P_SUCCESS`		SPI transfer succeeded.
 * @retval `NRF24L01P_SPI_ERROR`	SPI transfer failed.
 */
static Nrf24l01pStatus read_register_multibyte(Nrf24l01pDevice *device, uint8_t address, uint64_t *value, uint8_t length)
{
	uint8_t rx_data[length + 1];
	uint8_t tx_data[length + 1];
	int8_t spi_status;

	tx_data[0] = NRF24L01P_CMD_R_REGISTER | address;

	device->interface.set_cs(0);
	spi_status = device->interface.spi_tx_rx(tx_data, rx_data, length + 1);
	device->interface.set_cs(1);

	if (spi_status != 0)
		return NRF24L01P_SPI_ERROR;

	*value = 0; // reset output value to zeros
	for (int i = 0; i < length; i++)
		*value |= ((uint64_t)rx_data[i + 1] << (i * 8));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Writes a single byte to a register in the nRF24L01+ chip.
 *
 * @param[in] device  A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[in] address The address of the register to write to.
 * @param[in] payload The byte value to write to the register.
 *
 * @retval `NRF24L01P_SUCCESS`   SPI transfer succeeded.
 * @retval `NRF24L01P_SPI_ERROR` SPI transfer failed.
 */
static Nrf24l01pStatus write_register(Nrf24l01pDevice *device, uint8_t address, uint8_t payload)
{
	uint8_t tx_data[2] = {NRF24L01P_CMD_W_REGISTER | address, payload};
	uint8_t rx_data[2];
	int8_t spi_status;

	device->interface.set_cs(0);
	spi_status = device->interface.spi_tx_rx(tx_data, rx_data, 2);
	device->interface.set_cs(1);

	if (spi_status != 0)
		return NRF24L01P_SPI_ERROR;
	return NRF24L01P_SUCCESS;
}

/**
 * @brief Writes multiple bytes to a register in the nRF24L01+ chip.
 *
 * @param[in] device   A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[in] address  The address of the register to write to.
 * @param[in] payload  The value to write to the register, as a 64-bit unsigned integer.
 * @param[in] length   The number of bytes to write from the payload.
 *
 * @retval `NRF24L01P_SUCCESS`   SPI transfer succeeded.
 * @retval `NRF24L01P_SPI_ERROR` SPI transfer failed.
 */
static Nrf24l01pStatus write_register_multibyte(Nrf24l01pDevice *device, uint8_t address, uint64_t payload, uint8_t length)
{
	uint8_t tx_data[length + 1];
	uint8_t rx_data[length + 1];
	int8_t spi_status;

	tx_data[0] = NRF24L01P_CMD_W_REGISTER | address;
	for (int i = 0; i < length; i++)
		tx_data[i + 1] = (uint8_t)((payload >> (i * 8)) & 0xFF);

	device->interface.set_cs(0);
	spi_status = device->interface.spi_tx_rx(tx_data, rx_data, length + 1);
	device->interface.set_cs(1);

	if (spi_status != 0)
		return NRF24L01P_SPI_ERROR;
	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sends a single-byte command to the nRF24L01+ chip and reads the resulting status byte.
 *
 * @param[in] device   A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[in] command  The command byte to send to the chip.
 * @param[out] status  The status byte returned by the chip.
 *
 * @retval `NRF24L01P_SUCCESS`   SPI transfer succeeded.
 * @retval `NRF24L01P_SPI_ERROR` SPI transfer failed.
 */
static Nrf24l01pStatus send_command(Nrf24l01pDevice *device, uint8_t command, uint8_t *status)
{
	int8_t spi_status;

	device->interface.set_cs(0);
	spi_status = device->interface.spi_tx_rx(&command, status, 1);
	device->interface.set_cs(1);

	if (spi_status != 0)
		return NRF24L01P_SPI_ERROR;
	return NRF24L01P_SUCCESS;
}

/* ------------------------------ High-level API ------------------------------ */

/**
 * @brief Initializes the nRF24L01+ chip for TX (transmit) operation.
 *
 * This function must be called before calling any other nRF24L01+ functions.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`		Initialization succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed.
 * @retval `NRF24L01P_POINTER_NULL` `device` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_init_ptx(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	NRF24L01P_CHECK_STATUS(nrf24l01p_reset(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_init_general_config(device));

	// TX config
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_tx_output_power(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_tx_auto_retransmit_count(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_tx_auto_retransmit_delay(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_tx_addr(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_ptx_mode(device));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Initializes the nRF24L01+ chip for RX (receive) operation.
 *
 * This function must be called before calling any other nRF24L01+ functions.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS` 		Initialization succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed.
 * @retval `NRF24L01P_POINTER_NULL` `device` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_init_prx(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	NRF24L01P_CHECK_STATUS(nrf24l01p_reset(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_init_general_config(device));

	NRF24L01P_CHECK_STATUS(nrf24l01p_set_rx_pipes(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_rx_auto_ack_pipes(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_rx_addresses(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_rx_payload_length(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_prx_mode(device));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Checks if the RX FIFO is empty.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[out] empty A pointer to a boolean that will be set to `true` if the RX FIFO is empty, or `false` if it isn't.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while reading from FIFO STATUS register.
 * @retval `NRF24L01P_POINTER_NULL` Either `device` or `empty` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_is_rx_fifo_empty(Nrf24l01pDevice *device, bool *empty)
{
	NRF24L01P_CHECK_NULL(device);
	NRF24L01P_CHECK_NULL(empty);

	uint8_t fifo_status;
	NRF24L01P_CHECK_STATUS(nrf24l01p_get_fifo_status(device, &fifo_status));

	*empty = READ_BIT(fifo_status, NRF24L01P_REG_FIFO_STATUS_RX_EMPTY);
	return NRF24L01P_SUCCESS;
}

/**
 * @brief Receives data from the nRF24L01+ chip and stores it in the provided buffer.
 *
 * This function reads the RX FIFO of the nRF24L01+ chip.
 *
 * @param[in]  device     A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[out] rx_payload A pointer to a buffer where the received data will be stored.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while reading from RX FIFO.
 * @retval `NRF24L01P_POINTER_NULL` Either `device` or `rx_payload` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_rx_receive(Nrf24l01pDevice *device, uint8_t *rx_payload)
{
	NRF24L01P_CHECK_NULL(device);
	NRF24L01P_CHECK_NULL(rx_payload);

	NRF24L01P_CHECK_STATUS(nrf24l01p_read_rx_fifo(device, rx_payload));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Transmits data from the nRF24L01+ chip.
 *
 * This function writes the given data to the TX FIFO of the nRF24L01+ chip.
 *
 * @param[in] device     A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[in] tx_payload A pointer to the data to be transmitted.
 * @param[in] num_bytes  The number of bytes to transmit.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to TX FIFO.
 * @retval `NRF24L01P_POINTER_NULL` Either `device` or `tx_payload` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_tx_transmit(Nrf24l01pDevice *device, uint8_t *tx_payload, uint8_t num_bytes)
{
	NRF24L01P_CHECK_NULL(device);
	NRF24L01P_CHECK_NULL(tx_payload);

	NRF24L01P_CHECK_STATUS(nrf24l01p_write_tx_fifo(device, tx_payload, num_bytes));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Reads the IRQ flags of the nRF24L01+ chip and clears them.
 *
 * The `Nrf24l01pIrq` struct is filled with the values of the IRQ flags at the time of the call.
 *
 * @param[in]  device     	A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[out] irq_sources 	A pointer to a `Nrf24l01pIrq` struct that will be filled with the IRQ flag values.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while reading from STATUS register.
 * @retval `NRF24L01P_POINTER_NULL` Either `device` or `irq_sources` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_get_and_clear_irq_flags(Nrf24l01pDevice *device, Nrf24l01pIrq *irq_sources)
{
	NRF24L01P_CHECK_NULL(device);
	NRF24L01P_CHECK_NULL(irq_sources);

	uint8_t status;

	NRF24L01P_CHECK_STATUS(nrf24l01p_get_status(device, &status));
	NRF24L01P_CHECK_STATUS(nrf24l01p_clear_status_flags(device,
														NRF24L01P_REG_STATUS_MAX_RT |
															NRF24L01P_REG_STATUS_TX_DS |
															NRF24L01P_REG_STATUS_RX_DR));

	irq_sources->rx_dr = READ_BIT(status, NRF24L01P_REG_STATUS_RX_DR);
	irq_sources->tx_ds = READ_BIT(status, NRF24L01P_REG_STATUS_TX_DS);
	irq_sources->max_rt = READ_BIT(status, NRF24L01P_REG_STATUS_MAX_RT);

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Resets the nRF24L01+ chip to its default state.
 *
 * This function will reset the pins and registers of the nRF24L01+ chip to their default values and
 * reset the FIFOs.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to registers.
 * @retval `NRF24L01P_POINTER_NULL` `device` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_reset(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	// Reset pins
	device->interface.set_cs(1);

	// Reset registers
	for (size_t i = 0; i < sizeof(reg_rst_vals_8bit) / sizeof(reg_rst_vals_8bit[0]); i++)
		NRF24L01P_CHECK_STATUS(write_register(device, reg_rst_vals_8bit[i].address, reg_rst_vals_8bit[i].value));

	for (size_t i = 0; i < sizeof(reg_rst_vals_40bit) / sizeof(reg_rst_vals_40bit[0]); i++)
		NRF24L01P_CHECK_STATUS(write_register_multibyte(device, reg_rst_vals_40bit[i].address, reg_rst_vals_40bit[i].value, 40 / 8));

	// Reset FIFOs
	NRF24L01P_CHECK_STATUS(nrf24l01p_flush_rx_fifo(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_flush_tx_fifo(device));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the nRF24L01+ chip to RX mode.
 *
 * This function sets the `PRIM_RX` bit in the `CONFIG` register, which sets the nRF24L01+ chip to RX mode.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the CONFIG register.
 * @retval `NRF24L01P_POINTER_NULL` `device` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_set_prx_mode(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	uint8_t config_reg;
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_CONFIG, &config_reg));
	SET_BIT(config_reg, NRF24L01P_REG_CONFIG_PRIM_RX);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_CONFIG, config_reg));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the nRF24L01+ chip to TX mode.
 *
 * This function clears the `PRIM_RX` bit in the `CONFIG` register, which sets the nRF24L01+ chip to TX mode.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the CONFIG register.
 * @retval `NRF24L01P_POINTER_NULL` `device` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_set_ptx_mode(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	uint8_t config_reg;
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_CONFIG, &config_reg));
	CLEAR_BIT(config_reg, NRF24L01P_REG_CONFIG_PRIM_RX);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_CONFIG, config_reg));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the RX addresses of the nRF24L01+ chip.
 *
 * This function sets the RX addresses in the nRF24L01+ chip according to the values in the `Nrf24l01pRxConfig` struct.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   		The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 		SPI transfer failed while writing to the RX address registers.
 * @retval `NRF24L01P_POINTER_NULL` 	`device` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_set_rx_addresses(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	uint64_t address;
	uint8_t address_width;

	for (int i = 0; i < 6; i++)
	{
		switch (i)
		{
		case 0:
			address = device->rx_config.address_p0;
			break;
		case 1:
			address = device->rx_config.address_p1;
			break;
		case 2:
			address = (uint64_t)device->rx_config.address_p2;
			break;
		case 3:
			address = (uint64_t)device->rx_config.address_p3;
			break;
		case 4:
			address = (uint64_t)device->rx_config.address_p4;
			break;
		default:
			address = (uint64_t)device->rx_config.address_p5;
		}

		if (i < 2)
			address_width = 5;
		else
			address_width = 1;

		NRF24L01P_CHECK_STATUS(write_register_multibyte(device, NRF24L01P_REG_RX_ADDR_P0 + i, address, address_width));
	}

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the TX address of the nRF24L01+ chip.
 *
 * This function sets the TX address in the nRF24L01+ chip according to the value in the `Nrf24l01pTxConfig` struct.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the TX address register.
 * @retval `NRF24L01P_POINTER_NULL` `device` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_set_tx_addr(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	NRF24L01P_CHECK_STATUS(write_register_multibyte(device, NRF24L01P_REG_TX_ADDR, device->tx_config.address, device->config.address_width));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Reads the RX address of a specified pipe from the nRF24L01+ chip.
 *
 * This function retrieves the RX address for a given pipe index (0 to 5) and stores it
 * in the provided `address` pointer. For pipe indices 0 and 1, the full address is read,
 * while for indices 2 to 5, only a single byte is read.
 *
 * @param[in]  device  A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[in]  index   The pipe index (0 to 5) to read the RX address from.
 * @param[out] address A pointer to a `uint64_t` where the read RX address will be stored.
 *
 * @retval `NRF24L01P_SUCCESS`      	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR`    	SPI transfer failed while reading the address.
 * @retval `NRF24L01P_POINTER_NULL` 	Either `device` or `address` is `NULL`.
 * @retval `NRF24L01P_INVALID_VALUE` 	The provided pipe index is out of range (not between 0 and 5).
 */
Nrf24l01pStatus nrf24l01p_read_rx_addr(Nrf24l01pDevice *device, uint8_t index, uint64_t *address) // index must be integer from 0 to 5
{
	NRF24L01P_CHECK_NULL(device);
	NRF24L01P_CHECK_NULL(address);

	if (index > 5)
		return NRF24L01P_INVALID_VALUE;

	if (index == 0 || index == 1)
		NRF24L01P_CHECK_STATUS(read_register_multibyte(device, NRF24L01P_REG_RX_ADDR_P0 + index, address, device->config.address_width));
	else
	{
		uint8_t address_byte;
		NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_RX_ADDR_P0 + index, &address_byte));
		*address = (uint64_t)address_byte;
	}

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Reads the TX address from the nRF24L01+ chip.
 *
 * This function retrieves the TX address from the nRF24L01+ chip and stores it
 * in the provided `address` pointer. The address is read from the `TX_ADDR` register.
 *
 * @param[in]  device  A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[out] address A pointer to a `uint64_t` where the read TX address will be stored.
 *
 * @retval `NRF24L01P_SUCCESS`      	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR`    	SPI transfer failed while reading the TX address.
 * @retval `NRF24L01P_POINTER_NULL` 	Either `device` or `address` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_read_tx_addr(Nrf24l01pDevice *device, uint64_t *address)
{
	NRF24L01P_CHECK_NULL(device);
	NRF24L01P_CHECK_NULL(address);

	NRF24L01P_CHECK_STATUS(read_register_multibyte(device, NRF24L01P_REG_TX_ADDR, address, device->config.address_width));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Powers up the nRF24L01+ chip.
 *
 * This function powers up the nRF24L01+ chip by setting the `PWR_UP` bit in the `CONFIG` register.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the CONFIG register.
 * @retval `NRF24L01P_POINTER_NULL` `device` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_power_up(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	uint8_t config_reg;
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_CONFIG, &config_reg));
	SET_BIT(config_reg, NRF24L01P_REG_CONFIG_PWR_UP);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_CONFIG, config_reg));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Powers down the nRF24L01+ chip.
 *
 * This function powers down the nRF24L01+ chip by clearing the `PWR_UP` bit in the `CONFIG` register.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the CONFIG register.
 * @retval `NRF24L01P_POINTER_NULL` `device` is `NULL`.
 */
Nrf24l01pStatus nrf24l01p_power_down(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	uint8_t config_reg;
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_CONFIG, &config_reg));
	CLEAR_BIT(config_reg, NRF24L01P_REG_CONFIG_PWR_UP);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_CONFIG, config_reg));

	return NRF24L01P_SUCCESS;
}

/* ------------------------------ Low-level API ------------------------------ */

/**
 * @brief Reads the status register of the nRF24L01+ chip.
 *
 * This function reads the value of the `STATUS` register from the nRF24L01+ chip and stores it
 * in the provided `status` pointer.
 *
 * @param[in]  device 	A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[out] status 	A pointer to a `uint8_t` where the read status register value will be stored.
 *
 * @retval `NRF24L01P_SUCCESS`      	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR`    	SPI transfer failed while reading the STATUS register.
 */
Nrf24l01pStatus nrf24l01p_get_status(Nrf24l01pDevice *device, uint8_t *status)
{
	return send_command(device, NRF24L01P_CMD_NOP, status);
}

/**
 * @brief Reads the status register of the nRF24L01+ chip and clears the IRQ flags.
 *
 * This function reads the value of the `STATUS` register from the nRF24L01+ chip and stores it
 * in the provided `status` pointer. It also clears the IRQ flags by writing the value back to
 * the register.
 *
 * @param[in]  device 	A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[out] status 	A pointer to a `uint8_t` where the read status register value will be stored.
 *
 * @retval `NRF24L01P_SUCCESS`      	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR`    	SPI transfer failed while reading/writing the STATUS register.
 */
Nrf24l01pStatus nrf24l01p_get_status_and_clear_IRQ_flags(Nrf24l01pDevice *device, uint8_t *status)
{
	int8_t spi_status;
	uint8_t command = NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_STATUS;

	device->interface.set_cs(0);
	spi_status = device->interface.spi_tx_rx(&command, status, 1);
	if (spi_status != 0)
	{
		device->interface.set_cs(1);
		return NRF24L01P_SPI_ERROR;
	}
	// Following line takes advantage of the fact that active flag is 1, and writing 1 to it clears it
	NRF24L01P_CHECK_STATUS(device->interface.spi_tx(status, 1));
	device->interface.set_cs(1);

	if (spi_status != 0)
		return NRF24L01P_SPI_ERROR;
	return NRF24L01P_SUCCESS;
}

/**
 * @brief Retrieves the FIFO status of the nRF24L01+ chip.
 *
 * This function reads the `FIFO_STATUS` register of the nRF24L01+ chip and stores
 * its value in the provided `fifo_status` pointer.
 *
 * @param[in]  device      A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[out] fifo_status A pointer to a `uint8_t` where the FIFO status register value will be stored.
 *
 * @retval `NRF24L01P_SUCCESS`      	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR`    	SPI transfer failed while reading the `FIFO_STATUS` register.
 */
Nrf24l01pStatus nrf24l01p_get_fifo_status(Nrf24l01pDevice *device, uint8_t *fifo_status)
{
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_FIFO_STATUS, fifo_status));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Flushes the RX FIFO of the nRF24L01+ chip.
 *
 * This function clears the RX FIFO of the nRF24L01+ chip.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`      	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR`    	SPI transfer failed while flushing the RX FIFO.
 */
Nrf24l01pStatus nrf24l01p_flush_rx_fifo(Nrf24l01pDevice *device)
{
	uint8_t status;
	NRF24L01P_CHECK_STATUS(send_command(device, NRF24L01P_CMD_FLUSH_RX, &status));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Flushes the TX FIFO of the nRF24L01+ chip.
 *
 * This function clears the TX FIFO of the nRF24L01+ chip by sending the FLUSH_TX command.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`      	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR`    	SPI transfer failed while flushing the TX FIFO.
 */
Nrf24l01pStatus nrf24l01p_flush_tx_fifo(Nrf24l01pDevice *device)
{
	uint8_t status;
	NRF24L01P_CHECK_STATUS(send_command(device, NRF24L01P_CMD_FLUSH_TX, &status));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Reads the RX FIFO of the nRF24L01+ chip and retrieves the received payload.
 *
 * This function checks the status register to determine which pipe contains the received data
 * and reads the payload from the RX FIFO into the provided buffer. If the RX FIFO is empty,
 * it returns an error. It also handles invalid pipe numbers.
 *
 * @param[in]  device     A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[out] rx_payload A pointer to a buffer where the received data will be stored.
 *
 * @retval `NRF24L01P_SUCCESS`          The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR`        SPI transfer failed while reading from RX FIFO.
 * @retval `NRF24L01P_INVALID_VALUE`    The pipe number is invalid (greater than 5).
 * @retval `NRF24L01P_INVALID_OPERATION` The RX FIFO is empty.
 */
Nrf24l01pStatus nrf24l01p_read_rx_fifo(Nrf24l01pDevice *device, uint8_t *rx_payload)
{
	uint8_t status;
	int8_t spi_status;

	NRF24L01P_CHECK_STATUS(send_command(device, NRF24L01P_CMD_NOP, &status));

	uint8_t pipe = READ_FIELD(status, NRF24L01P_REG_STATUS_RX_P_NO_MASK);
	if (pipe == 0b110)
		return NRF24L01P_INVALID_VALUE; // there is no pipe '6' (0-5 only)
	if (pipe == 0b111)
		return NRF24L01P_INVALID_OPERATION; // RX FIFO is empty

	uint8_t length = device->rx_config.data_length[pipe];
	uint8_t tx_data[length + 1];
	uint8_t rx_data[length + 1];
	tx_data[0] = NRF24L01P_CMD_R_RX_PAYLOAD; // the rest of the data does not matter
	for (int i = 0; i < length; i++)
		tx_data[i + 1] = 0xFF;

	device->interface.set_cs(0);
	spi_status = device->interface.spi_tx_rx(tx_data, rx_data, length + 1);
	device->interface.set_cs(1);

	if (spi_status != 0)
		return NRF24L01P_SPI_ERROR;

	for (int i = 0; i < length; i++)
		rx_payload[i] = rx_data[i + 1];

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Writes data to the TX FIFO of the nRF24L01+ chip.
 *
 * This function writes the given data to the TX FIFO of the nRF24L01+ chip.
 *
 * @param[in] device     A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[in] tx_payload A pointer to the data to be transmitted.
 * @param[in] length     The number of bytes to transmit.
 *
 * @retval `NRF24L01P_SUCCESS`    SPI transfer succeeded.
 * @retval `NRF24L01P_SPI_ERROR`  SPI transfer failed while writing to TX FIFO.
 */
Nrf24l01pStatus nrf24l01p_write_tx_fifo(Nrf24l01pDevice *device, uint8_t *tx_payload, uint8_t length)
{
	uint8_t rx_data[length + 1];
	uint8_t tx_data[length + 1];
	int8_t spi_status;

	tx_data[0] = NRF24L01P_CMD_W_TX_PAYLOAD;
	for (int i = 0; i < length; i++)
		tx_data[i + 1] = tx_payload[i];

	device->interface.set_cs(0);
	spi_status = device->interface.spi_tx_rx(tx_data, rx_data, length + 1);
	device->interface.set_cs(1);

	if (spi_status != 0)
		return NRF24L01P_SPI_ERROR;

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the RX payload length of the nRF24L01+ chip.
 *
 * This function sets the RX payload length for each of the 6 pipes in the nRF24L01+ chip according to the values
 * in the `Nrf24l01pRxConfig` struct.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the RX payload length registers.
 * @retval `NRF24L01P_INVALID_VALUE` One of the RX payload lengths is invalid (0 or greater than 32).
 */
Nrf24l01pStatus nrf24l01p_set_rx_payload_length(Nrf24l01pDevice *device)
{
	for (int i = 0; i < 6; i++)
	{
		if (device->rx_config.data_length[i] > 32 || device->rx_config.data_length[i] == 0)
			return NRF24L01P_INVALID_VALUE;
		NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_RX_PW_P0 + i, device->rx_config.data_length[i]));
	}

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Clears the given flags in the `STATUS` register of the nRF24L01+ chip.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 * @param[in] flags The flags to clear in the `STATUS` register.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while reading/writing the STATUS register.
 */
Nrf24l01pStatus nrf24l01p_clear_status_flags(Nrf24l01pDevice *device, uint8_t flags)
{
	uint8_t status = flags;

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_STATUS, status));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the CRC length for the nRF24L01+ chip.
 *
 * This function configures the CRC length of the nRF24L01+ by updating the `CRCO` bit
 * in the `CONFIG` register based on the `crc_length` specified in the device's configuration.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while reading/writing the CONFIG register.
 */
Nrf24l01pStatus nrf24l01p_set_crc_length(Nrf24l01pDevice *device)
{

	uint8_t config_reg;
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_CONFIG, &config_reg));

	CLEAR_BIT(config_reg, NRF24L01P_REG_CONFIG_CRCO);
	config_reg |= device->config.crc_length; // enum values already reflect physical encoding

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_CONFIG, config_reg));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the address width for the nRF24L01+ chip.
 *
 * This function sets the address width to the specified value in the
 * `Nrf24l01pConfig` struct. The address width must be between 3 and 5 bytes.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the SETUP_AW register.
 * @retval `NRF24L01P_INVALID_VALUE` The address width is out of range (less than 3 or greater than 5).
 */
Nrf24l01pStatus nrf24l01p_set_address_width(Nrf24l01pDevice *device)
{
	if (device->config.address_width < 3 || device->config.address_width > 5)
		return NRF24L01P_INVALID_VALUE;

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_SETUP_AW, device->config.address_width - 2));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the auto retransmit count for the nRF24L01+ chip.
 *
 * This function configures the number of automatic retransmissions
 * for failed transmissions. The value is set in the `SETUP_RETR` register.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while reading/writing the SETUP_RETR register.
 * @retval `NRF24L01P_INVALID_VALUE` The auto retransmit count exceeds the maximum allowed value (15).
 */
Nrf24l01pStatus nrf24l01p_set_tx_auto_retransmit_count(Nrf24l01pDevice *device)
{
	if (device->tx_config.auto_retransmit_count > 15)
		return NRF24L01P_INVALID_VALUE;

	uint8_t setup_retr;

	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_SETUP_RETR, &setup_retr));

	CLEAR_FIELD(setup_retr, NRF24L01P_REG_SETUP_RETR_ARC_MASK);
	SET_FIELD(setup_retr, device->tx_config.auto_retransmit_count, NRF24L01P_REG_SETUP_RETR_ARC_MASK, NRF24L01P_REG_SETUP_RETR_ARC_POS);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_SETUP_RETR, setup_retr));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the auto retransmit delay for the nRF24L01+ chip.
 *
 * This function configures the auto retransmit delay in 250 us increments
 * from 250 us (1) to 4000 us (16). The value is set in the `SETUP_RETR` register.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   		The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 		SPI transfer failed while reading/writing the SETUP_RETR register.
 * @retval `NRF24L01P_INVALID_VALUE` 	The auto retransmit delay exceeds the maximum allowed value (16).
 */
Nrf24l01pStatus nrf24l01p_set_tx_auto_retransmit_delay(Nrf24l01pDevice *device)
{
	if (device->tx_config.auto_retransmit_delay_250us < 1 || device->tx_config.auto_retransmit_delay_250us > 16)
		return NRF24L01P_INVALID_VALUE;

	uint8_t setup_retr;
	uint8_t code = (device->tx_config.auto_retransmit_delay_250us - 1);

	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_SETUP_RETR, &setup_retr));

	CLEAR_FIELD(setup_retr, NRF24L01P_REG_SETUP_RETR_ARD_MASK);
	SET_FIELD(setup_retr, code, NRF24L01P_REG_SETUP_RETR_ARD_MASK, NRF24L01P_REG_SETUP_RETR_ARD_POS);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_SETUP_RETR, setup_retr));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Enables/disables the RX pipes of the nRF24L01+ chip.
 *
 * This function writes the `enable_pipes` member of the `Nrf24l01pRxConfig` struct
 * to the `EN_RXADDR` register, which enables/disables the corresponding RX pipes.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the EN_RXADDR register.
 */
Nrf24l01pStatus nrf24l01p_set_rx_pipes(Nrf24l01pDevice *device)
{
	uint8_t enable_pipes = (device->rx_config.enable_pipes & 0b00111111); // force bits 7 and 6 to zero

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_EN_RXADDR, enable_pipes));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the auto ACK status of the RX pipes of the nRF24L01+ chip.
 *
 * This function writes the `auto_ack_pipes` member of the `Nrf24l01pRxConfig` struct
 * to the `EN_AA` register, which enables/disables the automatic acknowledgement of
 * received packets for the corresponding RX pipes.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the EN_AA register.
 */
Nrf24l01pStatus nrf24l01p_set_rx_auto_ack_pipes(Nrf24l01pDevice *device)
{
	uint8_t en_aa = (device->rx_config.auto_ack_pipes & 0b00111111); // force bits 7 and 6 to zero

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_EN_AA, en_aa));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the RF channel of the nRF24L01+ chip.
 *
 * This function writes the `channel_MHz` member of the `Nrf24l01pConfig` struct
 * to the `RF_CH` register, which sets the RF channel of the nRF24L01+ chip.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the RF_CH register.
 * @retval `NRF24L01P_INVALID_VALUE` The RF channel is outside the valid range (2400 to 2525 [MHz]).
 */
Nrf24l01pStatus nrf24l01p_set_rf_channel(Nrf24l01pDevice *device)
{
	if (device->config.channel_MHz < 2400 || device->config.channel_MHz > 2525)
		return NRF24L01P_INVALID_VALUE;

	uint8_t rf_ch = device->config.channel_MHz - 2400;

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_RF_CH, rf_ch));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the TX output power of the nRF24L01+ chip.
 *
 * This function configures the transmission output power of the nRF24L01+ chip.
 * The value is set in the `RF_SETUP` register.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the `RF_SETUP` register.
 */
Nrf24l01pStatus nrf24l01p_set_tx_output_power(Nrf24l01pDevice *device)
{
	uint8_t rf_setup;

	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_RF_SETUP, &rf_setup));
	CLEAR_FIELD(rf_setup, NRF24L01P_REG_RF_SETUP_RF_PWR_MASK);
	rf_setup |= device->tx_config.output_power; // enum values already reflect physical encoding

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_RF_SETUP, rf_setup));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Sets the data rate of the nRF24L01+ chip.
 *
 * This function configures the data rate of the nRF24L01+ chip.
 * The value is set in the `RF_SETUP` register.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to the `RF_SETUP` register.
 */
Nrf24l01pStatus nrf24l01p_set_rf_data_rate(Nrf24l01pDevice *device)
{
	uint8_t rf_setup;

	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_RF_SETUP, &rf_setup));
	rf_setup &= ~(NRF24L01P_REG_RF_SETUP_RF_DR_HIGH | NRF24L01P_REG_RF_SETUP_RF_DR_LOW);
	rf_setup |= device->config.data_rate; // enum values already reflect physical encoding

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_RF_SETUP, rf_setup));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Configures the interrupt mask for the nRF24L01+ chip.
 *
 * This function updates the mask for interrupts in the `CONFIG` register based on the
 * `enable_irq_max_rt`, `enable_irq_tx_ds`, and `enable_irq_rx_dr` fields of the `Nrf24l01pConfig`
 * struct.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while reading/writing the CONFIG register.
 */
Nrf24l01pStatus nrf24l01p_set_irq_masks(Nrf24l01pDevice *device)
{
	uint8_t config_reg;

	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_CONFIG, &config_reg));
	config_reg &= ~(NRF24L01P_REG_CONFIG_MASK_MAX_RT | NRF24L01P_REG_CONFIG_MASK_TX_DS | NRF24L01P_REG_CONFIG_MASK_RX_DR);
	config_reg |= (device->config.enable_irq_max_rt ? 0 : NRF24L01P_REG_CONFIG_MASK_MAX_RT) |
				  (device->config.enable_irq_tx_ds ? 0 : NRF24L01P_REG_CONFIG_MASK_TX_DS) |
				  (device->config.enable_irq_rx_dr ? 0 : NRF24L01P_REG_CONFIG_MASK_RX_DR);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_CONFIG, config_reg));

	return NRF24L01P_SUCCESS;
}

/**
 * @brief Initializes the nRF24L01+ chip's general configuration.
 *
 * This function configures the RF channel, address width, RF data rate, CRC length, and interrupt masks
 * based on the fields in the `Nrf24l01pConfig` struct.
 *
 * @param[in] device A pointer to the `Nrf24l01pDevice` struct representing the nRF24L01+ chip.
 *
 * @retval `NRF24L01P_SUCCESS`   	The operation succeeded.
 * @retval `NRF24L01P_SPI_ERROR` 	SPI transfer failed while writing to one of the configuration registers.
 */
Nrf24l01pStatus nrf24l01p_init_general_config(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_rf_channel(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_address_width(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_rf_data_rate(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_crc_length(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_irq_masks(device));

	return NRF24L01P_SUCCESS;
}
