#include "app.h"
#include "main.h"
#include "sht4x.h"
#include "nrf24l01p.h"
#include "tim_custom.h"
#include "i2c_custom.h"
#include "spi_custom.h"
#include "gpio_custom.h"
#include "crc_custom.h"
#include "adc_custom.h"
#include "usart.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ---------------- Macros --------------------*/
#define NRF24_CHECK_ERROR_RETURN(expr, error)  \
	do                                         \
	{                                          \
		nrf24_status = (expr);                 \
		if (nrf24_status != NRF24L01P_SUCCESS) \
		{                                      \
			app_status = error;                \
			return STATE_ERROR;                \
		}                                      \
	} while (0)

#define NRF24_CHECK_ERROR_SET_STATE(expr, error) \
	do                                           \
	{                                            \
		nrf24_status = (expr);                   \
		if (nrf24_status != NRF24L01P_SUCCESS)   \
		{                                        \
			state = STATE_ERROR;                 \
			app_status = error;                  \
		}                                        \
	} while (0)

#define SHT4X_CHECK_ERROR_RETURN(expr, error) \
	do                                        \
	{                                         \
		sht4x_status = (expr);                \
		if (sht4x_status != SHT4X_SUCCESS)    \
		{                                     \
			app_status = error;               \
			return STATE_ERROR;               \
		}                                     \
	} while (0)

#define SHT4X_CHECK_ERROR_SET_STATE(expr, error) \
	do                                           \
	{                                            \
		sht4x_status = (expr);                   \
		if (sht4x_status != SHT4X_SUCCESS)       \
		{                                        \
			state = STATE_ERROR;                 \
			app_status = error;                  \
		}                                        \
	} while (0)

#define CHECK_ERROR_SET_STATE(expr, error) \
	do                                     \
	{                                      \
		if (expr != 0)                     \
		{                                  \
			state = STATE_ERROR;           \
			app_status = error;            \
		}                                  \
	} while (0)

#define CHECK_ERROR_RETURN(expr, error) \
	do                                  \
	{                                   \
		if (expr != 0)                  \
		{                               \
			app_status = error;         \
			return STATE_ERROR;         \
		}                               \
	} while (0)

#define PHASE1_LENGTH SHT4X_MEAS_HIGH_PREC_PERIOD_US - NRF24L01P_POWER_UP_DELAY_US
#define PHASE2_LENGTH NRF24L01P_POWER_UP_DELAY_US
#define NRF24_IRQ_WAIT_TIME 50000

/* ---------------- Enums --------------------*/
typedef enum AppState
{
	STATE_SLEEP = 0,
	STATE_IDLE = 1,
	STATE_PHASE1 = 2,
	STATE_PHASE2 = 3,
	STATE_AWAITING_ACK = 4,
	STATE_ERROR = 5
} AppState;

typedef enum AppEvent
{
	EVENT_NONE = 0,
	EVENT_PHASE1_DONE = 1,
	EVENT_PHASE2_DONE = 2,
	EVENT_RADIO_IRQ = 3,
	EVENT_RTC_WAKEUP = 4,
	EVENT_NRF24_IRQ_TIMEOUT = 5
} AppEvent;

typedef enum AppError
{
	ERROR_NONE = 0,
	ERROR_DISPATCHER = -1,
	ERROR_IDLE = -2,
	ERROR_PHASE1 = -3,
	ERROR_PHASE2 = -4,
	ERROR_AWAITING_ACK = -5,
	ERROR_SLEEP = -6,
	ERROR_SETUP = -7
} AppError;

/* ---------------- VDDA variable --------------------*/
static uint16_t vdda_mv;

/* ---------------- SHT4x variables --------------------*/
static Sht4xStatus sht4x_status = SHT4X_SUCCESS;
static Sht4xData sht4x_data = {
	.humidity = 0xFFFFFFFF,
	.temperature = 0xFFFFFFFF,
};
static Sht4xDevice sht4x = {
	.i2c_address = (SHT4X_I2C_ADDR_A << 1),
	.i2c_write = &I2C1_transmit,
	.i2c_read = &I2C1_receive,
	.calculate_crc = &calculate_CRC8,
};

/* ---------------- nRF24 variable --------------------*/
static Nrf24l01pStatus nrf24_status = NRF24L01P_SUCCESS;
static Nrf24l01pIrq nrf24_irq_sources = {
	.max_rt = false,
	.rx_dr = false,
	.tx_ds = false,
};
static Nrf24l01pDevice nrf24_device = {
	.interface = {
		.set_cs = &nrf24l01p_set_cs,
		.spi_tx = &SPI1_transmit,
		.spi_rx = &SPI1_receive,
		.spi_tx_rx = &SPI1_transmit_receive,
	},
	.config = {
		.channel_MHz = 2500,
		.address_width = 5,
		.data_rate = NRF24L01P_250KBPS,
		.crc_length = NRF24L01P_CRC_1BYTE,
		.enable_irq_tx_ds = true,
		.enable_irq_max_rt = true,
		.enable_irq_rx_dr = true
	},
	.tx_config = {
		.output_power = NRF24L01P_0DBM,
		.auto_retransmit_count = 15,
		.auto_retransmit_delay_250us = NRF24_AUTO_RETRANSMIT_DELAY, // defined in app.h, different for each node
		.address = NRF24L01P_REG_TX_ADDR_RSTVAL,
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
		.data_length = {8},
	}};
static uint8_t tx_payload[NRF24_DATA_LENGTH] = {0};

// The following variables are initialized in the "initialize_state_variables" function
volatile static AppEvent event;
static AppState state;
static AppError app_status;
volatile static bool debug_mode;
volatile static bool button_pending;

static char string_buffer[256] = {'0'};

/* ---------------- Prototypes --------------------*/
AppState dispatch_states(AppState state, volatile AppEvent *event);
void irs_phase1_done(void);
void irs_phase2_done(void);
void irq_timeout(void);
void irs_check_led_button(void);
void build_payload(uint8_t *payload);
void initialize_state_variables(void);
void UART2_Transmit(char *string);
void log_UART(const char *tag, const char *message);

/**
 * @brief Application setup function
 *
 * @param None
 * @retval None
 */
void app_setup(void)
{
	initialize_state_variables();

	LL_I2C_Enable(I2C1);
	LL_SPI_Enable(SPI1);

	CHECK_ERROR_SET_STATE(ADC_setup(), ERROR_SETUP);
	/* --- RTC wake up timer setup --- */
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_20);			// Enable interrupt for EXTI line 20 (RTC)
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_20); // Trigger on rising edge for line 20 (RTC)
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_WAKEUP_Disable(RTC);
	while (!LL_RTC_IsActiveFlag_WUTW(RTC))
		;
	LL_RTC_WAKEUP_SetAutoReload(RTC, RTC_WAKE_UP_PERIOD_S - 1);
	LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
	LL_RTC_EnableIT_WUT(RTC);
	LL_RTC_WAKEUP_Enable(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_EnableWriteProtection(RTC);

	NRF24_CHECK_ERROR_SET_STATE(nrf24l01p_init_ptx(&nrf24_device), ERROR_SETUP);
	NRF24_CHECK_ERROR_SET_STATE(nrf24l01p_get_and_clear_irq_flags(&nrf24_device, &nrf24_irq_sources), ERROR_SETUP);

	log_UART("app_setup", "");
}

/**
 * @brief Application loop function
 *
 * @param None
 * @retval None
 */
void app_loop(void)
{
	log_UART("app_loop", "");

	state = dispatch_states(state, &event);

	if (debug_mode && (app_status != ERROR_NONE))
		LL_GPIO_SetOutputPin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
	else
		LL_GPIO_ResetOutputPin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
}

/**
 * @brief Handles the `STATE_IDLE` application state
 *
 * In this state, a measurement is started on the SHT4x sensor, and a timer
 * is set to interrupt in `PHASE1_LENGTH` milliseconds. The ADC is also
 * configured to measure the VDDA voltage.
 *
 * @param event Pointer to an `AppEvent` variable which indicates the event that
 * caused the state transition
 *
 * @retval `STATE_PHASE1` on success
 * @retval `STATE_ERROR` if an error occurs
 */
AppState handle_state_idle(volatile AppEvent *event)
{
	SHT4X_CHECK_ERROR_RETURN(sht4x_start_measurement(&sht4x, SHT4X_I2C_CMD_MEAS_HIGH_PREC), ERROR_IDLE);
	CHECK_ERROR_RETURN(TIMx_schedule_interrupt(TIM21, PHASE1_LENGTH, &irs_phase1_done), ERROR_IDLE);
	CHECK_ERROR_RETURN(ADC_measure_vdda(&vdda_mv), ERROR_IDLE);
	return STATE_PHASE1;
}

/**
 * @brief Handles the `STATE_PHASE1` application state
 *
 * In this state, the MCU is put into sleep mode until the `PHASE1_LENGTH`
 * timer expires. When the timer expires, the state transitions to
 * `STATE_PHASE2` and the nRF24L01+ is powered up.
 *
 * @param event Pointer to an `AppEvent` variable which indicates the event that
 * caused the state transition
 *
 * @retval `STATE_PHASE1` if the timer has not expired
 * @retval `STATE_PHASE2` if the timer has expired and the nRF24L01P is powered up
 * @retval `STATE_ERROR` if an error occurs
 */
AppState handle_state_phase1(volatile AppEvent *event)
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

	*event = EVENT_NONE; // clear event flag

	NRF24_CHECK_ERROR_RETURN(nrf24l01p_power_up(&nrf24_device), ERROR_PHASE1);
	CHECK_ERROR_RETURN(TIMx_schedule_interrupt(TIM21, PHASE2_LENGTH, &irs_phase2_done), ERROR_PHASE1);

	return STATE_PHASE2;
}

/**
 * @brief Handles the `STATE_PHASE2` application state
 *
 * In this state, the MCU is put into sleep mode until the `PHASE2_LENGTH`
 * timer expires. When the timer expires, the state transitions to
 * `STATE_AWAITING_ACK` and the VDDA voltage is measured. The SHT4x sensor
 * measurement result is read and stored in `sht4x_data`. The measurement
 * result is formatted and stored in `tx_payload`. The nRF24L01+ transmit FIFO
 * is flushed. The `tx_payload` is written to the transmit FIFO and the `CE` pin
 * is pulsed for at least 10 us to start the transmission. A timeout is set to
 * interrupt in `NRF24_IRQ_WAIT_TIME` ms if nRF24L01+ IRQ does not activate.
 *
 * @param event Pointer to an `AppEvent` variable which indicates the event that
 * caused the state transition
 *
 * @retval `STATE_PHASE2` if the timer has not expired
 * @retval `STATE_AWAITING_ACK` if the `PHASE2_LENGTH` timer has expired and the
 * nRF24L01P is powered up and transmitting
 * @retval `STATE_ERROR` if an error occurs
 */
AppState handle_state_phase2(volatile AppEvent *event)
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

	*event = EVENT_NONE; // clear event flag

	SHT4X_CHECK_ERROR_RETURN(sht4x_read_measurement(&sht4x, &sht4x_data), ERROR_PHASE2);

	build_payload(tx_payload);

	// Flushing TX FIFO, because previous unacknowledged packets are still in it and we don't want to send them
	NRF24_CHECK_ERROR_RETURN(nrf24l01p_flush_tx_fifo(&nrf24_device), ERROR_AWAITING_ACK);
	// TX data can be written any time, nRF24L01+ will send it when ready
	NRF24_CHECK_ERROR_RETURN(nrf24l01p_write_tx_fifo(&nrf24_device, tx_payload, NRF24_DATA_LENGTH), ERROR_PHASE2);

	nrf24l01p_set_ce(1); // nRF24 CE pin is deactivated in the next state after IRQ or timeout (nRF24 clones need long CE pulse)

	CHECK_ERROR_RETURN(TIMx_schedule_interrupt(TIM21, NRF24_IRQ_WAIT_TIME * (1 + 9 * debug_mode), &irq_timeout), ERROR_PHASE2);

	return STATE_AWAITING_ACK;
}

/**
 * @brief Handles the `STATE_AWAITING_ACK` application state
 *
 * In this state, the MCU is put into sleep mode until the nRF24L01+ IRQ pin
 * is asserted. If the IRQ is caused by a transmission success event, the
 * `STATE_AWAITING_ACK` state is exited and the `STATE_SLEEP` state is entered.
 * If the IRQ is caused by a transmission failure event or the `NRF24_IRQ_WAIT_TIME`
 * timer expires, the `STATE_AWAITING_ACK` state is exited and the `STATE_ERROR`
 * state is entered.
 *
 * @param event Pointer to an `AppEvent` variable which indicates the event that
 * caused the state transition
 *
 * @retval `STATE_AWAITING_ACK` if the nRF24L01+ IRQ pin is not asserted yet and the
 * `NRF24_IRQ_WAIT_TIME` timer has not expired yet
 * @retval `STATE_ERROR` if the nRF24L01+ IRQ is caused by a transmission failure
 * or the `NRF24_IRQ_WAIT_TIME` timer expires
 * @retval `STATE_SLEEP` if the nRF24L01+ IRQ is caused by a transmission success
 * event
 */
AppState handle_state_awaiting_ack(volatile AppEvent *event)
{
	if (*event == EVENT_NRF24_IRQ_TIMEOUT)
	{
		nrf24l01p_set_ce(0);
		NRF24_CHECK_ERROR_RETURN(nrf24l01p_power_down(&nrf24_device), ERROR_AWAITING_ACK);
		return STATE_ERROR;
	}
	else if (*event != EVENT_RADIO_IRQ)
	{
		LL_PWR_DisableUltraLowPower();
		LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_MAIN);
		CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
		__WFI();
		/* --- Sleep mode --- */
		return STATE_AWAITING_ACK;
	}

	TIMx_disable_scheduled_interrupt(TIM21); // Disable scheduled nRF24_IRQ timeout
	nrf24l01p_set_ce(0);

	*event = EVENT_NONE; // clear event flag

	NRF24_CHECK_ERROR_RETURN(nrf24l01p_get_and_clear_irq_flags(&nrf24_device, &nrf24_irq_sources), ERROR_AWAITING_ACK);
	NRF24_CHECK_ERROR_RETURN(nrf24l01p_flush_tx_fifo(&nrf24_device), ERROR_AWAITING_ACK);
	NRF24_CHECK_ERROR_RETURN(nrf24l01p_power_down(&nrf24_device), ERROR_AWAITING_ACK);

	if (!nrf24_irq_sources.tx_ds)
		return STATE_ERROR;

	app_status = ERROR_NONE; // clear error

	return STATE_SLEEP;
}

/**
 * @brief Handles the `STATE_SLEEP` application state
 *
 * In this state, the system enters a low-power stop mode (if button handling is
 * not pending).
 * It configures GPIO pins to analog mode to reduce IDD and sets the power mode
 * to ultra-low power. The system is put into "stop mode" until an event such as
 * an RTC wakeup or a button press occur. Upon waking up, the system clock is
 * reconfigured and peripherals are reinitialized. If the wakeup event is caused
 * by an RTC alarm, the state transitions to `STATE_IDLE`.
 *
 * @param event Pointer to an `AppEvent` variable which indicates the event that
 * caused the state transition
 *
 * @retval `STATE_IDLE` if the wakeup event is due to an RTC alarm
 * @retval `STATE_SLEEP` otherwise
 */

AppState handle_state_sleep(volatile AppEvent *event)
{
	if (!button_pending)
	{
		log_UART("handle_state_sleep", "entering sleep");

		// turning SPI MISO hi-Z lowers the IDD
		set_pins_to_analog_mode(GPIOA, LL_GPIO_PIN_ALL & ~LED_STATUS_Pin & ~LED_ERROR_Pin & ~BUTTON_LED_Pin);
		set_pins_to_analog_mode(GPIOB, LL_GPIO_PIN_ALL & ~nRF24_CE_Pin & ~nRF24_CSN_Pin & ~SPI_SCK_Pin & ~SPI_MOSI_Pin);
		set_pins_to_analog_mode(GPIOC, LL_GPIO_PIN_ALL);

		LL_PWR_EnableUltraLowPower();
		LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_LOW_POWER);
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP);
		LL_LPM_EnableDeepSleep();
		__WFI();

		/* --------- Stop mode here --------------*/

		SystemClock_Config(); // first action after wake up

		if (LL_PWR_IsActiveFlag_WU())
			LL_PWR_ClearFlag_WU();

		reinitialize_gpio();
		MX_I2C1_Init();
		MX_SPI1_Init();

		MX_USART2_UART_Init();
	}

	if (*event == EVENT_RTC_WAKEUP)
	{
		log_UART("handle_state_sleep", "woke up due to RTC");

		*event = EVENT_NONE; // clear event flag
		return STATE_IDLE;
	}

	return STATE_SLEEP;
}

/**
 * @brief Handles the `STATE_ERROR` application state
 *
 * An empty dummy state. Error handling is currently handled in debug mode via UART logs and error LED.
 *
 * @param event Pointer to an `AppEvent` variable which indicates the event that
 * caused the state transition
 *
 * @retval `STATE_SLEEP` after error handling
 */

AppState handle_state_error(volatile AppEvent *event)
{
	// Error handling is currently handled in debug mode via UART logs and error LED
	return STATE_SLEEP;
}

AppState dispatch_states(AppState state, volatile AppEvent *event)
{
	switch (state)
	{
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
		app_status = ERROR_DISPATCHER;
		return STATE_ERROR;
	}
}

void GPIO_EXTI15_IRQ_callback(void)
{
	event = EVENT_RADIO_IRQ;
}

void GPIO_EXTI4_IRQ_callback(void)
{
	TIMx_schedule_interrupt(TIM22, 20000, &irs_check_led_button);
	button_pending = true;
}

void RTC_WAKEUP_IRQ_callback(void)
{
	if (LL_RTC_IsActiveFlag_WUT(RTC) != 0)
		LL_RTC_ClearFlag_WUT(RTC); // Clear the wakeup timer interrupt flag

	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_20))
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20); // Clear the EXTI line 20 pending flag (RTC wakeup interrupt)

	event = EVENT_RTC_WAKEUP;
}

void irs_phase1_done(void)
{
	event = EVENT_PHASE1_DONE;
}

void irs_phase2_done(void)
{
	event = EVENT_PHASE2_DONE;
}

void irq_timeout(void)
{
	event = EVENT_NRF24_IRQ_TIMEOUT;
}

void irs_check_led_button(void)
{
	if ((LL_GPIO_ReadInputPort(BUTTON_LED_GPIO_Port) & BUTTON_LED_Pin) == 0) // button is still pressed
	{
		debug_mode = !debug_mode;

		if (debug_mode)
		{
			LL_GPIO_SetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
		}
		else
		{
			LL_GPIO_ResetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
			LL_GPIO_ResetOutputPin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
		}
	}

	button_pending = false;
}

void build_payload(uint8_t *payload)
{
	int16_t temperature, humidity;

	temperature = sht4x_data.temperature / 10; // limit data to 16 bits (losing 3rd decimal point precision)
	humidity = sht4x_data.humidity / 10;	   // limit data to 16 bits (losing 3rd decimal point precision)

	payload[0] = NODE_ID;
	payload[1] = app_status;   // to be interpreted as signed at receiver
	payload[2] = sht4x_status; // to be interpreted as signed at receiver
	payload[3] = nrf24_status; // to be interpreted as signed at receiver
	payload[4] = (temperature & 0xFF00) >> 8;
	payload[5] = (temperature & 0x00FF);
	payload[6] = (humidity & 0xFF00) >> 8;
	payload[7] = (humidity & 0x00FF);
	payload[8] = (vdda_mv & 0xFF00) >> 8;
	payload[9] = (vdda_mv & 0x00FF);
}

// The function below initializes important state variables
// Initialization in declaration did not work for some unknown reason
void initialize_state_variables(void)
{
	event = EVENT_NONE;
	state = STATE_IDLE;
	app_status = ERROR_NONE;
	debug_mode = 0;
	button_pending = 0;
}

void UART2_Transmit(char *string)
{
	while (*string)
	{
		while (!LL_USART_IsActiveFlag_TXE(USART2))
			;									   // Wait until TXE (Transmit Empty) is set
		LL_USART_TransmitData8(USART2, *string++); // Send character
	}
	while (!LL_USART_IsActiveFlag_TC(USART2))
		; // Wait for last transmission to complete
}

void log_UART(const char *tag, const char *message)
{
	if (debug_mode)
	{
		snprintf(string_buffer, sizeof(string_buffer), "[%s]%s: state = %d, app_status = %d, nrf24_status = %d, sht4x_status = %d, event = %d\r\n",
				 tag, message, state, app_status, nrf24_status, sht4x_status, event);
		UART2_Transmit(string_buffer);
	}
}
