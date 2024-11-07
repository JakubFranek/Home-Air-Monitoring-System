#include "nrf24l01p.h"
#include <stddef.h> // definitions of NULL, size_t

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

// Error-checking macro: if "expr" returns anything other than NRF24L01P_SUCCESS,
// this macro returns that return value, exiting the function where this macro was used immediately
#define NRF24L01P_CHECK_STATUS(expr)     \
	do                                   \
	{                                    \
		Nrf24l01pStatus retval = expr;   \
		if (retval != NRF24L01P_SUCCESS) \
		{                                \
			return retval;               \
		}                                \
	} while (0)

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
static Nrf24l01pStatus read_register(Nrf24l01pDevice *device, uint8_t address, uint8_t *value)
{
	uint8_t command = NRF24L01P_CMD_R_REGISTER | address;
	uint8_t status;

	device->interface.set_cs(0);

	if (device->interface.spi_tx_rx(command, &status) != 0)
		return NRF24L01P_SPI_ERROR;
	if (device->interface.spi_tx_rx(0xFF, value) != 0)
		return NRF24L01P_SPI_ERROR;

	device->interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

static Nrf24l01pStatus read_register_multibyte(Nrf24l01pDevice *device, uint8_t address, uint64_t *value, uint8_t num_bytes)
{
	uint8_t command = NRF24L01P_CMD_R_REGISTER | address;
	uint8_t read_byte, status;
	*value = 0;

	device->interface.set_cs(0);

	if (device->interface.spi_tx_rx(command, &status) != 0)
		return NRF24L01P_SPI_ERROR;

	for (int i = 0; i < num_bytes; i++)
	{
		if (device->interface.spi_tx_rx(0xFF, &read_byte) != 0)
			return NRF24L01P_SPI_ERROR;
		*value |= ((uint64_t)read_byte << (i * 8));
	}

	device->interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

static Nrf24l01pStatus write_register(Nrf24l01pDevice *device, uint8_t address, uint8_t payload)
{
	uint8_t command = NRF24L01P_CMD_W_REGISTER | address;
	uint8_t status;

	device->interface.set_cs(0);

	if (device->interface.spi_tx_rx(command, &status) != 0)
		return NRF24L01P_SPI_ERROR;

	if (device->interface.spi_tx(payload) != 0)
		return NRF24L01P_SPI_ERROR;

	device->interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

static Nrf24l01pStatus write_register_multibyte(Nrf24l01pDevice *device, uint8_t address, uint64_t payload, uint8_t num_bytes)
{
	uint8_t command = NRF24L01P_CMD_W_REGISTER | address;
	uint8_t payload_byte;
	uint8_t status;

	device->interface.set_cs(0);

	if (device->interface.spi_tx_rx(command, &status) != 0)
		return NRF24L01P_SPI_ERROR;

	for (int i = 0; i < num_bytes; i++)
	{
		payload_byte = (uint8_t)((payload >> (i * 8)) & 0xFF); // Extract byte from payload (LSB first)
		if (device->interface.spi_tx(payload_byte))
			return NRF24L01P_SPI_ERROR;
	}

	device->interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

static Nrf24l01pStatus send_command(Nrf24l01pDevice *device, uint8_t command, uint8_t *status)
{
	device->interface.set_cs(0);

	if (device->interface.spi_tx_rx(command, status) != 0)
		return NRF24L01P_SPI_ERROR;

	device->interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

/* ------------------------------ High-level API ------------------------------ */
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

Nrf24l01pStatus nrf24l01p_rx_receive(Nrf24l01pDevice *device, uint8_t *rx_payload)
{
	NRF24L01P_CHECK_NULL(device);
	NRF24L01P_CHECK_NULL(rx_payload);

	NRF24L01P_CHECK_STATUS(nrf24l01p_read_rx_fifo(device, rx_payload));
	NRF24L01P_CHECK_STATUS(nrf24l01p_clear_flag(device, NRF24L01P_REG_STATUS_RX_DR));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_tx_transmit(Nrf24l01pDevice *device, uint8_t *tx_payload, uint8_t num_bytes)
{
	NRF24L01P_CHECK_NULL(device);
	NRF24L01P_CHECK_NULL(tx_payload);

	NRF24L01P_CHECK_STATUS(nrf24l01p_write_tx_fifo(device, tx_payload, num_bytes));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_get_and_clear_irq_flags(Nrf24l01pDevice *device, Nrf24l01pIrq *irq_sources)
{
	NRF24L01P_CHECK_NULL(device);
	NRF24L01P_CHECK_NULL(irq_sources);

	uint8_t dummy;
	device->interface.spi_rx(&dummy); // Ensure SPI RX buffer is empty

	uint8_t status;
	NRF24L01P_CHECK_STATUS(nrf24l01p_get_status_and_clear_IRQ_flags(device, &status));

	irq_sources->rx_dr = READ_BIT(status, NRF24L01P_REG_STATUS_RX_DR);
	irq_sources->tx_ds = READ_BIT(status, NRF24L01P_REG_STATUS_TX_DS);
	irq_sources->max_rt = READ_BIT(status, NRF24L01P_REG_STATUS_MAX_RT);

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_reset(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	// Reset pins
	device->interface.set_cs(1);
	device->interface.set_ce(0);

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

Nrf24l01pStatus nrf24l01p_set_prx_mode(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	uint8_t config_reg;
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_CONFIG, &config_reg));
	SET_BIT(config_reg, NRF24L01P_REG_CONFIG_PRIM_RX);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_CONFIG, config_reg));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_set_ptx_mode(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	uint8_t config_reg;
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_CONFIG, &config_reg));
	CLEAR_BIT(config_reg, NRF24L01P_REG_CONFIG_PRIM_RX);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_CONFIG, config_reg));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_set_rx_addresses(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	uint64_t address;

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
		case 5:
			address = (uint64_t)device->rx_config.address_p5;
			break;
		}

		NRF24L01P_CHECK_STATUS(write_register_multibyte(device, NRF24L01P_REG_RX_ADDR_P0 + i, address, device->config.address_width));
	}

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_set_tx_addr(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	NRF24L01P_CHECK_STATUS(write_register_multibyte(device, NRF24L01P_REG_TX_ADDR, device->tx_config.address, device->config.address_width));

	return NRF24L01P_SUCCESS;
}

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

Nrf24l01pStatus nrf24l01p_read_tx_addr(Nrf24l01pDevice *device, uint64_t *address)
{
	NRF24L01P_CHECK_NULL(device);
	NRF24L01P_CHECK_NULL(address);

	NRF24L01P_CHECK_STATUS(read_register_multibyte(device, NRF24L01P_REG_TX_ADDR, address, device->config.address_width));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_power_up(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_NULL(device);

	uint8_t config_reg;
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_CONFIG, &config_reg));
	SET_BIT(config_reg, NRF24L01P_REG_CONFIG_PWR_UP);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_CONFIG, config_reg));

	return NRF24L01P_SUCCESS;
}

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
Nrf24l01pStatus nrf24l01p_get_status(Nrf24l01pDevice *device, uint8_t *status)
{
	device->interface.set_cs(0);

	NRF24L01P_CHECK_STATUS(device->interface.spi_tx_rx(NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_STATUS, status));

	device->interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_get_status_and_clear_IRQ_flags(Nrf24l01pDevice *device, uint8_t *status)
{
	device->interface.set_cs(0);

	NRF24L01P_CHECK_STATUS(device->interface.spi_tx_rx(NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_STATUS, status));
	// Following line takes advantage of the fact that active flag is 1, and writing 1 to it clears it
	NRF24L01P_CHECK_STATUS(device->interface.spi_tx(*status));

	device->interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_get_fifo_status(Nrf24l01pDevice *device, uint8_t *fifo_status)
{
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_FIFO_STATUS, fifo_status));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_flush_rx_fifo(Nrf24l01pDevice *device)
{
	uint8_t status;
	NRF24L01P_CHECK_STATUS(send_command(device, NRF24L01P_CMD_FLUSH_RX, &status));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_flush_tx_fifo(Nrf24l01pDevice *device)
{
	uint8_t status;
	NRF24L01P_CHECK_STATUS(send_command(device, NRF24L01P_CMD_FLUSH_TX, &status));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_read_rx_fifo(Nrf24l01pDevice *device, uint8_t *rx_payload)
{
	uint8_t status;
	NRF24L01P_CHECK_STATUS(send_command(device, NRF24L01P_CMD_R_RX_PAYLOAD, &status));

	uint8_t pipe = READ_FIELD(status, NRF24L01P_REG_STATUS_RX_P_NO_MASK);
	if (pipe == 0b110)
		return NRF24L01P_INVALID_VALUE;		// there is no pipe '6' (0-5 only)
	if (pipe == 0b111)
		return NRF24L01P_INVALID_OPERATION;	// RX FIFO is empty

	device->interface.set_cs(0);

	for (uint8_t i = 0; i < device->rx_config.data_length[pipe]; i++)
		NRF24L01P_CHECK_STATUS(device->interface.spi_tx_rx(0xFF, &rx_payload[i]));

	device->interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_write_tx_fifo(Nrf24l01pDevice *device, uint8_t *tx_payload, uint8_t num_bytes)
{
	uint8_t status;

	device->interface.set_cs(0);

	NRF24L01P_CHECK_STATUS(device->interface.spi_tx_rx(NRF24L01P_CMD_W_TX_PAYLOAD, &status));

	for (uint8_t i = 0; i < num_bytes; i++)
		NRF24L01P_CHECK_STATUS(device->interface.spi_tx(tx_payload[i]));

	device->interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

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

Nrf24l01pStatus nrf24l01p_clear_status_flag(Nrf24l01pDevice *device, uint8_t flags)
{
	uint8_t status;

	NRF24L01P_CHECK_STATUS(nrf24l01p_get_status(device, &status));
	SET_BIT(status, flags);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_STATUS, status));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_set_crc_length(Nrf24l01pDevice *device)
{

	uint8_t config_reg;
	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_CONFIG, &config_reg));

	CLEAR_BIT(config_reg, NRF24L01P_REG_CONFIG_CRCO);
	config_reg |= device->config.crc_length; // enum values already reflect physical encoding

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_CONFIG, config_reg));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_set_address_width(Nrf24l01pDevice *device)
{
	if (device->config.address_width < 3 || device->config.address_width > 5)
		return NRF24L01P_INVALID_VALUE;

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_SETUP_AW, device->config.address_width - 2));

	return NRF24L01P_SUCCESS;
}

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

Nrf24l01pStatus nrf24l01p_set_rx_pipes(Nrf24l01pDevice *device)
{
	uint8_t enable_pipes = (device->rx_config.enable_pipes & 0b00111111); // force bits 7 and 6 to zero

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_EN_RXADDR, enable_pipes));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_set_rx_auto_ack_pipes(Nrf24l01pDevice *device)
{
	uint8_t en_aa = (device->rx_config.auto_ack_pipes & 0b00111111); // force bits 7 and 6 to zero

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_EN_AA, en_aa));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_set_rf_channel(Nrf24l01pDevice *device)
{
	if (device->config.channel_MHz < 2400 || device->config.channel_MHz > 2525)
		return NRF24L01P_INVALID_VALUE;

	uint8_t rf_ch = device->config.channel_MHz - 2400;

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_RF_CH, rf_ch));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_set_tx_output_power(Nrf24l01pDevice *device)
{
	uint8_t rf_setup;

	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_RF_SETUP, &rf_setup));
	CLEAR_FIELD(rf_setup, NRF24L01P_REG_RF_SETUP_RF_PWR_MASK);
	rf_setup |= device->tx_config.output_power; // enum values already reflect physical encoding

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_RF_SETUP, rf_setup));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_set_rf_data_rate(Nrf24l01pDevice *device)
{
	uint8_t rf_setup;

	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_RF_SETUP, &rf_setup));
	rf_setup &= ~(NRF24L01P_REG_RF_SETUP_RF_DR_HIGH | NRF24L01P_REG_RF_SETUP_RF_DR_LOW);
	rf_setup |= device->config.data_rate; // enum values already reflect physical encoding

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_RF_SETUP, rf_setup));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_set_irq_masks(Nrf24l01pDevice *device)
{
	uint8_t config_reg;

	NRF24L01P_CHECK_STATUS(read_register(device, NRF24L01P_REG_RF_SETUP, &config_reg));
	config_reg &= ~(NRF24L01P_REG_CONFIG_MASK_MAX_RT | NRF24L01P_REG_CONFIG_MASK_TX_DS | NRF24L01P_REG_CONFIG_MASK_RX_DR);
	config_reg |= (device->config.enable_irq_max_rt ? NRF24L01P_REG_CONFIG_MASK_MAX_RT : 0) |
				  (device->config.enable_irq_tx_ds ? NRF24L01P_REG_CONFIG_MASK_TX_DS : 0) |
				  (device->config.enable_irq_rx_dr ? NRF24L01P_REG_CONFIG_MASK_RX_DR : 0);

	NRF24L01P_CHECK_STATUS(write_register(device, NRF24L01P_REG_RF_SETUP, config_reg));

	return NRF24L01P_SUCCESS;
}

Nrf24l01pStatus nrf24l01p_init_general_config(Nrf24l01pDevice *device)
{
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_rf_channel(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_address_width(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_rf_data_rate(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_crc_length(device));
	NRF24L01P_CHECK_STATUS(nrf24l01p_set_irq_masks(device));

	return NRF24L01P_SUCCESS;
}
