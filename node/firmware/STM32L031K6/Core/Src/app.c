#include "app.h"
#include <stdbool.h>
#include "main.h"
#include "sht4x.h"
#include "nrf24l01p.h"
#include "tim_custom.h"
#include "i2c_custom.h"
#include "spi_custom.h"
#include "gpio_custom.h"
#include "crc_custom.h"

#define NRF24_CHECK_ERROR_RETURN(expr)         	\
    do {                                       	\
        nrf24_status = (expr);                 	\
        if (nrf24_status != NRF24L01P_SUCCESS) 	\
            return STATE_ERROR;                	\
    } while(0)

#define NRF24_CHECK_ERROR_SET_STATE(expr)     	\
    do {                                       	\
        nrf24_status = (expr);                 	\
        if (nrf24_status != NRF24L01P_SUCCESS) 	\
            state = STATE_ERROR;               	\
    } while(0)

#define SHT4X_CHECK_ERROR_RETURN(expr)         	\
    do {                                       	\
        sht4x_status = (expr);                 	\
        if (sht4x_status != SHT4X_SUCCESS) 		\
            return STATE_ERROR;                	\
    } while(0)

#define SHT4X_CHECK_ERROR_SET_STATE(expr)      	\
    do {                                       	\
        sht4x_status = (expr);                 	\
        if (sht4x_status != SHT4X_SUCCESS) 		\
            state = STATE_ERROR;               	\
    } while(0)

#define CHECK_ERROR_SET_STATE(expr)      		\
    do {                    					\
        if (expr != 0) 							\
            state = STATE_ERROR;               	\
    } while(0)

#define CHECK_ERROR_RETURN(expr)    	  		\
    do {                    					\
        if (expr != 0) 							\
            return STATE_ERROR;               	\
    } while(0)


#define PHASE1_LENGTH SHT4X_MEAS_HIGH_PREC_PERIOD_US - NRF24L01P_POWER_UP_DELAY_US
#define PHASE2_LENGTH NRF24L01P_POWER_UP_DELAY_US

static Sht4xStatus sht4x_status = SHT4X_SUCCESS;
static Sht4xData sht4x_data = {
		.humidity = 0xFFFFFFFF,
		.temperature = 0xFFFFFFFF
};
static Sht4xDevice sht4x = {
	.i2c_address = SHT4X_I2C_ADDR_A,
	.i2c_write = &I2C1_transmit_byte,
	.i2c_read = &I2C1_receive,
	.calculate_crc = &calculate_CRC8
};

static Nrf24l01pStatus nrf24_status = NRF24L01P_SUCCESS;
static Nrf24l01pIrq nrf24_irq_sources = {
		.max_rt = false,
		.rx_dr = false,
		.tx_ds = false
};
static Nrf24l01pDevice nrf24_device = {
	.interface = {
		.set_cs = &nrf24l01p_set_cs,
		.set_ce = &nrf24l01p_set_ce,
		.spi_tx = &SPI1_Transmit_Multi,
		.spi_rx = &SPI1_Receive,
		.spi_tx_rx = &SPI1_TransmitReceive
	},
	.config = {
			.channel_MHz = 2500,
			.address_width = 5,
			.data_rate = NRF24L01P_1MBPS,
			.crc_length = NRF24L01P_CRC_1BYTE
	},
	.tx_config = {
			.output_power = NRF24L01P_0DBM,
			.auto_retransmit_count = 3,
			.auto_retransmit_delay_250us = 1,
			.address = NRF24L01P_REG_TX_ADDR_RSTVAL
	},
	.rx_config = {
			.enable_pipes = 0b00000001,
			.auto_ack_pipes = 0b00000001,
			.address_p0 = NRF24L01P_REG_RX_ADDR_P0_RSTVAL,
			.address_p1 = NRF24L01P_REG_RX_ADDR_P1_RSTVAL,
			.address_p2 = NRF24L01P_REG_RX_ADDR_P2_RSTVAL,
			.address_p3 = NRF24L01P_REG_RX_ADDR_P3_RSTVAL,
			.address_p4 = NRF24L01P_REG_RX_ADDR_P4_RSTVAL,
			.address_p5 = NRF24L01P_REG_RX_ADDR_P5_RSTVAL,
			.data_length = {8}
	}
};
static uint8_t tx_payload[NRF24_DATA_LENGTH] = {0, 1, 2, 3, 4, 5, 6, 7};

typedef enum AppState
{
	STATE_SLEEP = 0,
	STATE_IDLE = 1,
	STATE_PHASE1 = 2,
	STATE_PHASE2 = 3,
	STATE_TX_READY = 4,
	STATE_AWAITING_ACK = 5,
	STATE_ERROR = 6
} AppState;

typedef enum AppEvent
{
	EVENT_NONE = 0,
	EVENT_PHASE1_DONE = 1,
	EVENT_PHASE2_DONE = 2,
	EVENT_RADIO_IRQ = 3
} AppEvent;

typedef enum AppError
{
	ERROR_NONE = 0,
	ERROR_ = -1
} AppError;


AppState dispatch_states(AppState state, volatile AppEvent* event);
void GPIO_EXTI1_IRQ_callback(void);
void irs_phase1_done(void);
void irs_phase2_done(void);
void build_payload(uint8_t* payload);


volatile static AppEvent event = EVENT_NONE;	// changes value during ISRs
static AppState state = STATE_IDLE;
static AppError app_status = ERROR_NONE;

/**
 * @brief Application setup function
 *
 * This function is called once at startup (presumably within the main()).
 *
 * @param None
 * @retval None
 */
void app_setup(void)
{
	LL_I2C_Enable(I2C1);
	LL_SPI_Enable(SPI1);


	// TODO: clean this up
	// RTC wake up timer setup
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_20);  			// Enable interrupt for EXTI line 20 (RTC)
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_20);  	// Trigger on rising edge for line 20 (RTC)
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_WAKEUP_Disable(RTC);
	while(!LL_RTC_IsActiveFlag_WUTW(RTC));
	LL_RTC_WAKEUP_SetAutoReload(RTC, RTC_WAKE_UP_PERIOD_S - 1);
	LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
	LL_RTC_EnableIT_WUT(RTC);
	LL_RTC_WAKEUP_Enable(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_EnableWriteProtection(RTC);

	NRF24_CHECK_ERROR_SET_STATE(nrf24l01p_init_ptx(&nrf24_device));
	NRF24_CHECK_ERROR_SET_STATE(nrf24l01p_get_and_clear_irq_flags(&nrf24_device, &nrf24_irq_sources));
	NRF24_CHECK_ERROR_SET_STATE(nrf24l01p_set_ptx_mode(&nrf24_device));
}

/**
 * @brief Application loop function
 *
 * This function is called in an endless loop (presumably within the main()).
 *
 * @param None
 * @retval None
 */
void app_loop(void)
{
	// TODO: add ADC supply voltage measurement
	// TODO: use internal HSI with lower frequency, higher frequency I2C/SPI
	state = dispatch_states(state, &event);
}


AppState handle_state_idle(volatile AppEvent* event)
{
	SHT4X_CHECK_ERROR_RETURN(sht4x_send_command(&sht4x, SHT4X_I2C_CMD_MEAS_HIGH_PREC));
	CHECK_ERROR_RETURN(TIMx_schedule_interrupt(TIM21, PHASE1_LENGTH, &irs_phase1_done));
	return STATE_PHASE1;
}

AppState handle_state_phase1(volatile AppEvent* event)
{
	if (*event != EVENT_PHASE1_DONE)
	{
		LL_PWR_DisableUltraLowPower();
		LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_MAIN);
		CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
		__WFI();
		/* --- Sleep mode --- */
		return STATE_PHASE1;
	}

	*event = EVENT_NONE; 	// clear event flag

	NRF24_CHECK_ERROR_RETURN(nrf24l01p_power_up(&nrf24_device));

	CHECK_ERROR_RETURN(TIMx_schedule_interrupt(TIM21, PHASE2_LENGTH, &irs_phase2_done));

	return STATE_PHASE2;
}

AppState handle_state_phase2(volatile AppEvent* event)
{
	if (*event != EVENT_PHASE2_DONE)
	{
		LL_PWR_DisableUltraLowPower();
		LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_MAIN);
		CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
		__WFI();
		/* --- Sleep mode --- */
		return STATE_PHASE2;
	}

	*event = EVENT_NONE; 	// clear event flag

	SHT4X_CHECK_ERROR_RETURN(sht4x_read_and_check_measurement(&sht4x, &sht4x_data));

	build_payload(tx_payload);

	// TX data can be written any time, nRF24L01+ will send it when ready¨
	NRF24_CHECK_ERROR_RETURN(nrf24l01p_write_tx_fifo(&nrf24_device, tx_payload, NRF24_DATA_LENGTH));

	// min 10 us CE pulse according to nRF24 datasheet
	nrf24l01p_set_ce(1);
	CHECK_ERROR_RETURN(TIMx_delay_us(TIM2, NRF24L01P_PTX_MIN_CE_PULSE_US));
	nrf24l01p_set_ce(0);

	return STATE_AWAITING_ACK;
}

AppState handle_state_awaiting_ack(volatile AppEvent* event)
{
	if (*event != EVENT_RADIO_IRQ)
	{
		LL_PWR_DisableUltraLowPower();
		LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_MAIN);
		CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
		__WFI();
		/* --- Sleep mode --- */
		return STATE_AWAITING_ACK;
	}

	*event = EVENT_NONE; // clear event flag

	NRF24_CHECK_ERROR_RETURN(nrf24l01p_get_and_clear_irq_flags(&nrf24_device, &nrf24_irq_sources));

	if (nrf24_irq_sources.max_rt)
		return STATE_ERROR;

	NRF24_CHECK_ERROR_RETURN(nrf24l01p_power_down(&nrf24_device));

	return STATE_SLEEP;
}

AppState handle_state_sleep(volatile AppEvent* event)
{
	// for some reason turning SPI MISO hi-Z really lowers the IDD
	set_pins_to_analog_mode(GPIOA, LL_GPIO_PIN_ALL & ~nRF24_CE_Pin);
	set_pins_to_analog_mode(GPIOB, LL_GPIO_PIN_ALL);
	set_pins_to_analog_mode(GPIOC, LL_GPIO_PIN_ALL);

	LL_PWR_EnableUltraLowPower();
	LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_LOW_POWER);
	LL_PWR_SetPowerMode(LL_PWR_MODE_STOP);
	LL_LPM_EnableDeepSleep();
	__WFI();

	/* --------- Stop mode here --------------*/

	SystemClock_Config();	// first action after wake up

	MX_GPIO_Init();
	MX_CRC_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_TIM21_Init();

	if (LL_PWR_IsActiveFlag_WU())
		LL_PWR_ClearFlag_WU();

	return STATE_IDLE;
}

AppState handle_state_error(volatile AppEvent* event)
{
	// TODO: error handling
	return STATE_SLEEP;
}

AppState dispatch_states(AppState state, volatile AppEvent* event)
{
	switch (state) {
		case STATE_IDLE:
			return handle_state_idle(event);
		case STATE_PHASE1:
			return handle_state_phase1(event);
		case STATE_PHASE2:
			return handle_state_phase2(event);
		case STATE_AWAITING_ACK:
			return handle_state_awaiting_ack(event);
		case STATE_SLEEP:
			return handle_state_sleep(event);
		case STATE_ERROR:
			return handle_state_error(event);
		default:
			return STATE_ERROR;
	}
}

void GPIO_EXTI1_IRQ_callback(void)
{
	event = EVENT_RADIO_IRQ;
}

void RTC_WAKEUP_IRQ_callback(void)
{
	if (LL_RTC_IsActiveFlag_WUT(RTC) != 0)
		LL_RTC_ClearFlag_WUT(RTC); 	// Clear the wakeup timer interrupt flag

	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_20))
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);	// Clear the EXTI line 20 pending flag (RTC wakeup interrupt)
}

void irs_phase1_done(void)
{
	event = EVENT_PHASE1_DONE;
}

void irs_phase2_done(void)
{
	event = EVENT_PHASE2_DONE;
}

void build_payload(uint8_t* payload)
{
	int16_t temperature, humidity;

	temperature = sht4x_data.temperature / 10;	// limit data to 16 bits (losing 3rd decimal point precision)
	humidity = sht4x_data.humidity / 10;		// limit data to 16 bits (losing 3rd decimal point precision)

	payload[0] = NODE_ID;
	payload[1] = app_status;	// to be interpreted as signed at receiver
	payload[2] = sht4x_status;	// to be interpreted as signed at receiver
	payload[3] = nrf24_status;	// to be interpreted as signed at receiver
	payload[4] = (temperature & 0xFF00) >> 8;
	payload[5] = (temperature & 0x00FF);
	payload[6] = (humidity & 0xFF00) >> 8;
	payload[7] = (humidity & 0x00FF);
	payload[8] = 0x00;			// reserved
	payload[9] = 0x00;			// reserved
}
