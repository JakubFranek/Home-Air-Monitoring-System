#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__

#include <stdint.h>	 // definition of uint8_t etc
#include <stdbool.h> // definition of bool

// TODO: double check whether _POS values are correct

/*----------- nRF24L01+ Commands -----------*/
// TODO: this could be enum
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

/*----------- nRF24L01+ Registers -----------*/
// TODO: this could be enum
#define NRF24L01P_REG_CONFIG 0x00
#define NRF24L01P_REG_EN_AA 0x01
#define NRF24L01P_REG_EN_RXADDR 0x02
#define NRF24L01P_REG_SETUP_AW 0x03
#define NRF24L01P_REG_SETUP_RETR 0x04
#define NRF24L01P_REG_RF_CH 0x05
#define NRF24L01P_REG_RF_SETUP 0x06
#define NRF24L01P_REG_STATUS 0x07
#define NRF24L01P_REG_OBSERVE_TX 0x08 // Read-Only
#define NRF24L01P_REG_RPD 0x09		  // Read-Only
#define NRF24L01P_REG_RX_ADDR_P0 0x0A
#define NRF24L01P_REG_RX_ADDR_P1 0x0B
#define NRF24L01P_REG_RX_ADDR_P2 0x0C
#define NRF24L01P_REG_RX_ADDR_P3 0x0D
#define NRF24L01P_REG_RX_ADDR_P4 0x0E
#define NRF24L01P_REG_RX_ADDR_P5 0x0F
#define NRF24L01P_REG_TX_ADDR 0x10
#define NRF24L01P_REG_RX_PW_P0 0x11
#define NRF24L01P_REG_RX_PW_P1 0x12
#define NRF24L01P_REG_RX_PW_P2 0x13
#define NRF24L01P_REG_RX_PW_P3 0x14
#define NRF24L01P_REG_RX_PW_P4 0x15
#define NRF24L01P_REG_RX_PW_P5 0x16
#define NRF24L01P_REG_FIFO_STATUS 0x17
#define NRF24L01P_REG_DYNPD 0x1C
#define NRF24L01P_REG_FEATURE 0x1D

/*----------- nRF24L01+ Register Reset Values -----------*/
#define NRF24L01P_REG_CONFIG_RSTVAL 0x08
#define NRF24L01P_REG_EN_AA_RSTVAL 0x3F
#define NRF24L01P_REG_EN_RXADDR_RSTVAL 0x03
#define NRF24L01P_REG_SETUP_AW_RSTVAL 0x03
#define NRF24L01P_REG_SETUP_RETR_RSTVAL 0x03
#define NRF24L01P_REG_RF_CH_RSTVAL 0x02
#define NRF24L01P_REG_RF_SETUP_RSTVAL 0x0E
#define NRF24L01P_REG_STATUS_RSTVAL 0x0E
#define NRF24L01P_REG_RX_ADDR_P0_RSTVAL 0xE7E7E7E7E7
#define NRF24L01P_REG_RX_ADDR_P1_RSTVAL 0xC2C2C2C2C2
#define NRF24L01P_REG_RX_ADDR_P2_RSTVAL 0xC3
#define NRF24L01P_REG_RX_ADDR_P3_RSTVAL 0xC4
#define NRF24L01P_REG_RX_ADDR_P4_RSTVAL 0xC5
#define NRF24L01P_REG_RX_ADDR_P5_RSTVAL 0xC6
#define NRF24L01P_REG_TX_ADDR_RSTVAL 0xE7E7E7E7E7
#define NRF24L01P_REG_RX_PW_P0_RSTVAL 0x00
#define NRF24L01P_REG_RX_PW_P1_RSTVAL 0x00
#define NRF24L01P_REG_RX_PW_P2_RSTVAL 0x00
#define NRF24L01P_REG_RX_PW_P3_RSTVAL 0x00
#define NRF24L01P_REG_RX_PW_P4_RSTVAL 0x00
#define NRF24L01P_REG_RX_PW_P5_RSTVAL 0x00
#define NRF24L01P_REG_FIFO_STATUS_RSTVAL 0x11
#define NRF24L01P_REG_DYNPD_RSTVAL 0x00
#define NRF24L01P_REG_FEATURE_RSTVAL 0x00

/*----------- nRF24L01+ Register Bits -----------*/
// CONFIG
#define NRF24L01P_REG_CONFIG_MASK_RX_DR (1U << 6)
#define NRF24L01P_REG_CONFIG_MASK_TX_DS (1U << 5)
#define NRF24L01P_REG_CONFIG_MASK_MAX_RT (1U << 4)
#define NRF24L01P_REG_CONFIG_EN_CRC (1U << 3)
#define NRF24L01P_REG_CONFIG_CRCO (1U << 2)
#define NRF24L01P_REG_CONFIG_PWR_UP (1U << 1)
#define NRF24L01P_REG_CONFIG_PRIM_RX (1U << 0)
// EN_AA
#define NRF24L01P_REG_EN_AA_ENAA_P5 (1U << 5)
#define NRF24L01P_REG_EN_AA_ENAA_P4 (1U << 4)
#define NRF24L01P_REG_EN_AA_ENAA_P3 (1U << 3)
#define NRF24L01P_REG_EN_AA_ENAA_P2 (1U << 2)
#define NRF24L01P_REG_EN_AA_ENAA_P1 (1U << 1)
#define NRF24L01P_REG_EN_AA_ENAA_P0 (1U << 0)
// EN_RXADDR
#define NRF24L01P_REG_EN_RXADDR_ERX_P5 (1U << 5)
#define NRF24L01P_REG_EN_RXADDR_ERX_P4 (1U << 4)
#define NRF24L01P_REG_EN_RXADDR_ERX_P3 (1U << 3)
#define NRF24L01P_REG_EN_RXADDR_ERX_P2 (1U << 2)
#define NRF24L01P_REG_EN_RXADDR_ERX_P1 (1U << 1)
#define NRF24L01P_REG_EN_RXADDR_ERX_P0 (1U << 0)
// SETUP_AW
#define NRF24L01P_REG_SETUP_AW_AW_POS (1U << 0)
#define NRF24L01P_REG_SETUP_AW_AW_MASK (0b11 << 0)
// SETUP_RETR
#define NRF24L01P_REG_SETUP_RETR_ARD_POS (1U << 4)
#define NRF24L01P_REG_SETUP_RETR_ARD_MASK (0b1111 << 4)
#define NRF24L01P_REG_SETUP_RETR_ARC_POS (1U << 0)
#define NRF24L01P_REG_SETUP_RETR_ARC_MASK (0b1111 << 0)
// RF_CH
#define NRF24L01P_REG_RF_CH_POS (1U << 0)
#define NRF24L01P_REG_RF_CH_MASK (0b1111111 << 0)
// RF_SETUP
#define NRF24L01P_REG_RF_SETUP_CONT_WAVE (1U << 7)
#define NRF24L01P_REG_RF_SETUP_RF_DR_LOW (1U << 5)
#define NRF24L01P_REG_RF_SETUP_PLL_LOCK (1U << 4)
#define NRF24L01P_REG_RF_SETUP_RF_DR_HIGH (1U << 3)
#define NRF24L01P_REG_RF_SETUP_RF_PWR_POS (1U << 1)
#define NRF24L01P_REG_RF_SETUP_RF_PWR_MASK (0b11 << 1)
// RF_STATUS
#define NRF24L01P_REG_STATUS_RX_DR (1U << 6)
#define NRF24L01P_REG_STATUS_TX_DS (1U << 5)
#define NRF24L01P_REG_STATUS_MAX_RT (1U << 4)
#define NRF24L01P_REG_STATUS_RX_P_NO_POS (1U << 1)
#define NRF24L01P_REG_STATUS_RX_P_NO_MASK (0b111 << 1)
#define NRF24L01P_REG_STATUS_TX_FULL (1U << 0)
// OBSERVE_TX
#define NRF24L01P_REG_OBSERVE_TX_PLOS_CNT_POS (1U << 4)
#define NRF24L01P_REG_OBSERVE_TX_PLOS_CNT_MASK (0b1111 << 4)
#define NRF24L01P_REG_OBSERVE_TX_ARC_CNT_POS (1U << 0)
#define NRF24L01P_REG_OBSERVE_TX_ARC_CNT_MASK (0b1111 << 0)
// RPD
#define NRF24L01P_REG_RPD_RPD (1U << 0)
// RX_PW_P0
#define NRF24L01P_REG_RX_PW_P0_POS (1U << 0)
#define NRF24L01P_REG_RX_PW_P0_MASK (0b111111 << 0)
// RX_PW_P1
#define NRF24L01P_REG_RX_PW_P1_POS (1U << 0)
#define NRF24L01P_REG_RX_PW_P1_MASK (0b111111 << 0)
// RX_PW_P2
#define NRF24L01P_REG_RX_PW_P2_POS (1U << 0)
#define NRF24L01P_REG_RX_PW_P2_MASK (0b111111 << 0)
// RX_PW_P3
#define NRF24L01P_REG_RX_PW_P3_POS (1U << 0)
#define NRF24L01P_REG_RX_PW_P3_MASK (0b111111 << 0)
// RX_PW_P4
#define NRF24L01P_REG_RX_PW_P4_POS (1U << 0)
#define NRF24L01P_REG_RX_PW_P4_MASK (0b111111 << 0)
// RX_PW_P5
#define NRF24L01P_REG_RX_PW_P5_POS (1U << 0)
#define NRF24L01P_REG_RX_PW_P5_MASK (0b111111 << 0)
// FIFO_STATUS
#define NRF24L01P_REG_FIFO_STATUS_TX_REUSE (1U << 6)
#define NRF24L01P_REG_FIFO_STATUS_TX_FULL (1U << 5)
#define NRF24L01P_REG_FIFO_STATUS_TX_EMPTY (1U << 4)
#define NRF24L01P_REG_FIFO_STATUS_RX_FULL (1U << 1)
#define NRF24L01P_REG_FIFO_STATUS_RX_EMPTY (1U << 0)
// DYNPD
#define NRF24L01P_REG_DYNPD_DPL_P5 (1U << 5)
#define NRF24L01P_REG_DYNPD_DPL_P4 (1U << 4)
#define NRF24L01P_REG_DYNPD_DPL_P3 (1U << 3)
#define NRF24L01P_REG_DYNPD_DPL_P2 (1U << 2)
#define NRF24L01P_REG_DYNPD_DPL_P1 (1U << 1)
#define NRF24L01P_REG_DYNPD_DPL_P0 (1U << 0)
// FEATURE
#define NRF24L01P_REG_FEATURE_EN_DPL (1U << 2)
#define NRF24L01P_REG_FEATURE_EN_ACK_PAY (1U << 1)
#define NRF24L01P_REG_FEATURE_EN_DYN_ACK (1U << 0)

/*----------- nRF24L01+ Other defines -----------*/
#define NRF24L01P_POWER_UP_DELAY_US 1500 // time between power up command and arrival at Standby-I mode
#define NRF24L01P_PTX_MIN_CE_PULSE_US 10 // minimum CE = '1' pulse to start transmitting in PTX mode

/*----------- nRF24L01+ Enums, Structs and Function Pointers -----------*/
typedef enum Nrf24l01pError
{
	NRF24L01P_SUCCESS = 0,
	NRF24L01P_SPI_ERROR = -1,
	NRF24L01P_INVALID_VALUE = -2,
	NRF24L01P_INVALID_OPERATION = -3,
	NRF24L01P_POINTER_NULL = -4
} Nrf24l01pStatus;

typedef struct Nrf24l01pIrq
{
	bool rx_dr;	 // data received
	bool tx_ds;	 // transmission successful
	bool max_rt; // maximum retransmit attempts reached
} Nrf24l01pIrq;

typedef struct Nrf24l01p8bitRegRstVals
{
	uint8_t address;
	uint8_t value;
} Nrf24l01p8bitRegRstVals;

extern Nrf24l01p8bitRegRstVals reg_rst_vals_8bit[]; // initialized in .c

typedef struct Nrf24l01p40bitRegRstVals
{
	uint8_t address;
	uint64_t value;
} Nrf24l01p40bitRegRstVals;

extern Nrf24l01p40bitRegRstVals reg_rst_vals_40bit[]; // initialized in .c

typedef enum Nrf24l01pDataRate
{
	// member values reflect physical register map encoding
	NRF24L01P_250KBPS = (1U << 5), // 250 kbps
	NRF24L01P_1MBPS = 0U,		   // 1 Mbps
	NRF24L01P_2MBPS = (1U << 3)	   // 2 Mbps
} Nrf24l01pDataRate;

typedef enum Nrf24l01pOutputPower
{
	// member values reflect physical register map encoding
	NRF24L01P_0DBM = (3U << 1),		// 0 dBm
	NRF24L01P_NEG6DBM = (2U << 1),	// -6 dBm
	NRF24L01P_NEG12DBM = (1U << 1), // -12 dBm
	NRF24L01P_NEG18DBM = (0U << 1)	// -18 dBm
} Nrf24l01pOutputPower;

typedef enum Nrf24l01pCrcLength
{
	// member values reflect physical register map encoding
	NRF24L01P_CRC_1BYTE = (0U << 2),
	NRF24L01P_CRC_2BYTE = (1U << 2)
} Nrf24l01pCrcLength;

// Argument state = 0 for low, 1 for high
typedef void (*nrf24l01p_set_cs_t)(uint8_t state);
typedef void (*nrf24l01p_set_ce_t)(uint8_t state);

// Return value of following functions is error code, 0 is only accepted success value
typedef int8_t (*nrf24l01p_spi_tx_t)(const uint8_t *tx_data, uint8_t length);
typedef int8_t (*nrf24l01p_spi_rx_t)(uint8_t *rx_data, uint8_t length);
typedef int8_t (*nrf24l01p_spi_tx_rx_t)(const uint8_t *tx_data, uint8_t *rx_data, uint8_t length);

typedef struct Nrf24l01pInterface
{
	nrf24l01p_set_cs_t set_cs;
	nrf24l01p_set_ce_t set_ce;
	nrf24l01p_spi_tx_t spi_tx;
	nrf24l01p_spi_rx_t spi_rx;
	nrf24l01p_spi_tx_rx_t spi_tx_rx;
} Nrf24l01pInterface;

typedef struct Nrf24l01pConfig
{
	uint16_t channel_MHz;  // range 2400 to 2525 MHz
	uint8_t address_width; // range 3 to 5 bytes
	Nrf24l01pDataRate data_rate;
	Nrf24l01pCrcLength crc_length;
	bool enable_irq_tx_ds;
	bool enable_irq_max_rt;
	bool enable_irq_rx_dr;
} Nrf24l01pConfig;

typedef struct Nrf24l01pTxConfig
{
	Nrf24l01pOutputPower output_power;
	uint8_t auto_retransmit_count;		 // range 0 to 15
	uint8_t auto_retransmit_delay_250us; // range 1 to 16 (in multiples of 250 us)
	uint64_t address;					 // exactly address_width bytes (see "General settings")
} Nrf24l01pTxConfig;

typedef struct Nrf24l01pRxConfig
{
	uint8_t enable_pipes;	// interpreted as binary, 2 MSBs ignored, '1' = pipe enabled
	uint8_t auto_ack_pipes; // interpreted as binary, 2 MSBs ignored, '1' = auto ACK enabled
	uint64_t address_p0;	// exactly address_width bytes (see "General settings")
	uint64_t address_p1;	// exactly address_width bytes (see "General settings")
	uint8_t address_p2;		// always 1 byte
	uint8_t address_p3;		// always 1 byte
	uint8_t address_p4;		// always 1 byte
	uint8_t address_p5;		// always 1 byte
	uint8_t data_length[6]; // range 1 to 32 bytes
} Nrf24l01pRxConfig;

// Handler struct for use in application code
typedef struct Nrf24l01pDevice
{
	Nrf24l01pInterface interface;
	Nrf24l01pConfig config;
	Nrf24l01pTxConfig tx_config;
	Nrf24l01pRxConfig rx_config;
} Nrf24l01pDevice;

/*----------- High-level API functions -----------*/
Nrf24l01pStatus nrf24l01p_init_prx(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_init_ptx(Nrf24l01pDevice *device);

Nrf24l01pStatus nrf24l01p_rx_receive(Nrf24l01pDevice *device, uint8_t *rx_payload);
Nrf24l01pStatus nrf24l01p_tx_transmit(Nrf24l01pDevice *device, uint8_t *tx_payload, uint8_t num_bytes);

Nrf24l01pStatus nrf24l01p_get_and_clear_irq_flags(Nrf24l01pDevice *device, Nrf24l01pIrq *irq_sources);
Nrf24l01pStatus nrf24l01p_reset(Nrf24l01pDevice *device);

Nrf24l01pStatus nrf24l01p_read_rx_addr(Nrf24l01pDevice *device, uint8_t pipe, uint64_t *address);
Nrf24l01pStatus nrf24l01p_read_tx_addr(Nrf24l01pDevice *device, uint64_t *address);

Nrf24l01pStatus nrf24l01p_power_up(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_power_down(Nrf24l01pDevice *device);

/*----------- Low-level API functions -----------*/
Nrf24l01pStatus nrf24l01p_set_prx_mode(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_ptx_mode(Nrf24l01pDevice *device);

Nrf24l01pStatus nrf24l01p_get_status(Nrf24l01pDevice *device, uint8_t *status);
Nrf24l01pStatus nrf24l01p_get_status_and_clear_IRQ_flags(Nrf24l01pDevice *device, uint8_t *status);
Nrf24l01pStatus nrf24l01p_clear_flag(Nrf24l01pDevice *device, uint8_t flag);
Nrf24l01pStatus nrf24l01p_get_fifo_status(Nrf24l01pDevice *device, uint8_t *fifo_status);

Nrf24l01pStatus nrf24l01p_read_rx_fifo(Nrf24l01pDevice *device, uint8_t *rx_payload);
Nrf24l01pStatus nrf24l01p_write_tx_fifo(Nrf24l01pDevice *device, uint8_t *tx_payload, uint8_t num_bytes);
Nrf24l01pStatus nrf24l01p_flush_rx_fifo(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_flush_tx_fifo(Nrf24l01pDevice *device);

Nrf24l01pStatus nrf24l01p_init_general_config(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_crc_length(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_address_width(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_irq_masks(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_rf_channel(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_rf_data_rate(Nrf24l01pDevice *device);

Nrf24l01pStatus nrf24l01p_set_tx_auto_retransmit_count(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_tx_auto_retransmit_delay(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_tx_output_power(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_tx_addr(Nrf24l01pDevice *device);

Nrf24l01pStatus nrf24l01p_set_rx_addresses(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_rx_auto_ack_pipes(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_rx_pipes(Nrf24l01pDevice *device);
Nrf24l01pStatus nrf24l01p_set_rx_payload_length(Nrf24l01pDevice *device);

#endif /* __NRF24L01P_H__ */
