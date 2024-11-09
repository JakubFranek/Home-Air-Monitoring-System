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

// TODO: get rid of this function
int8_t SPI1_Transmit(uint8_t tx_data)
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

int8_t SPI1_Transmit_Multi(uint8_t* tx_data, uint8_t length)
{
	uint16_t count;

	for (int i = 0; i < length; i++)
	{
		TIMx_restart(SPI_TIMEOUT_TIMER);
		LL_SPI_TransmitData8(SPI1, tx_data[i]);
		while (!LL_SPI_IsActiveFlag_TXE(SPI1))
		{
			TIMx_get_count(SPI_TIMEOUT_TIMER, &count);
			if (count > SPI_TIMEOUT_US)
				return -1;
		}
	}

	TIMx_restart(SPI_TIMEOUT_TIMER);
	while (LL_SPI_IsActiveFlag_BSY(SPI1))
	{
		TIMx_get_count(SPI_TIMEOUT_TIMER, &count);
		if (count > SPI_TIMEOUT_US)
			return -1;
	}

	return 0;
}

// TODO: add multibyte support
int8_t SPI1_Receive(uint8_t *rx_data)
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

// TODO: add multibyte support
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
