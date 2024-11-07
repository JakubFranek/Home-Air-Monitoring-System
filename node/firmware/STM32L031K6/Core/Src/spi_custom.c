#include <stdint.h>
#include "main.h"
#include "tim_custom.h" // needed for TIM2-based timeout checks

#define SPI_TIMEOUT_US 1000
#define SPI_TIMEOUT_TIMER TIM2

static uint8_t transmit_done()
{
	uint8_t done = LL_SPI_IsActiveFlag_TXE(SPI1) && !LL_SPI_IsActiveFlag_BSY(SPI1);
	return done;
}

static uint8_t receive_done()
{
	uint8_t done = LL_SPI_IsActiveFlag_RXNE(SPI1) && !LL_SPI_IsActiveFlag_BSY(SPI1);
	return done;
}

uint8_t SPI1_Transmit(uint8_t tx_data)
{
	uint16_t count;

	TIMx_restart(SPI_TIMEOUT_TIMER);

	LL_SPI_TransmitData8(SPI1, tx_data);

	while (!transmit_done())
	{
		TIMx_get_count(SPI_TIMEOUT_TIMER, &count);
			if (count > SPI_TIMEOUT_US)
				return -1;
	}

	return 0;
}

uint8_t SPI1_Receive(uint8_t *rx_data)
{
	uint16_t count;

	TIMx_restart(SPI_TIMEOUT_TIMER);

	while (!receive_done())
	{
		TIMx_get_count(SPI_TIMEOUT_TIMER, &count);
			if (count > SPI_TIMEOUT_US)
				return -1;
	}

	*rx_data = LL_SPI_ReceiveData8(SPI1);
	return 0;
}

uint8_t SPI1_TransmitReceive(uint8_t tx_data, uint8_t *rx_data)
{
	uint16_t count;

	TIMx_restart(SPI_TIMEOUT_TIMER);

	LL_SPI_TransmitData8(SPI1, tx_data);

	while (!transmit_done() || !receive_done())
	{
		TIMx_get_count(SPI_TIMEOUT_TIMER, &count);
		if (count > SPI_TIMEOUT_US)
			return -1;
	}

	*rx_data = LL_SPI_ReceiveData8(SPI1);

	return 0;
}
