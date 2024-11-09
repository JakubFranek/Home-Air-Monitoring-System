#include <stdint.h>
#include "main.h"
#include "tim_custom.h" // needed for TIM2-based timeout checks

#define SPI_TIMEOUT_US 1000
#define SPI_TIMEOUT_TIMER TIM2

int8_t SPI1_transmit(const uint8_t* tx_data, uint8_t length)
{
	uint16_t count;

	// Make sure RX buffer is empty before SPI transaction
	(void) LL_SPI_ReceiveData8(SPI1);

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

int8_t SPI1_receive(uint8_t *rx_data, uint8_t length)
{
	uint16_t count;

	// Make sure RX buffer is empty before SPI transaction
	(void) LL_SPI_ReceiveData8(SPI1);

	for(int i = 0; i < length; i++)
	{
		TIMx_restart(SPI_TIMEOUT_TIMER);
		while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
		{
			TIMx_get_count(SPI_TIMEOUT_TIMER, &count);
			if (count > SPI_TIMEOUT_US)
				return -1;
		}
		rx_data[i] = LL_SPI_ReceiveData8(SPI1);
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

uint8_t SPI1_transmit_receive(const uint8_t* tx_data, uint8_t *rx_data, uint8_t length)
{
	uint16_t count;

	uint8_t tx_i = 0;
	uint8_t rx_i = 0;

	// Make sure RX buffer is empty before SPI transaction
	(void) LL_SPI_ReceiveData8(SPI1);

	TIMx_restart(SPI_TIMEOUT_TIMER);
	while(rx_i < length)
	{
		// If TX buffer is empty, load more data
		if(LL_SPI_IsActiveFlag_TXE(SPI1) && tx_i < length)
		{
			LL_SPI_TransmitData8(SPI1, tx_data[tx_i]);
			tx_i++;
			while(LL_SPI_IsActiveFlag_TXE(SPI1))
			{
				TIMx_get_count(SPI_TIMEOUT_TIMER, &count);
				if (count > SPI_TIMEOUT_US)
					return -1;
			}
		}

		// If RX buffer is not empty, read data
		if(LL_SPI_IsActiveFlag_RXNE(SPI1))
		{
			rx_data[rx_i] = LL_SPI_ReceiveData8(SPI1);
			rx_i++;
			while(LL_SPI_IsActiveFlag_RXNE(SPI1))
			{
				TIMx_get_count(SPI_TIMEOUT_TIMER, &count);
				if (count > SPI_TIMEOUT_US)
					return -1;
			}
		}

		TIMx_get_count(SPI_TIMEOUT_TIMER, &count);
		if (count > SPI_TIMEOUT_US)
			return -1;
	}

	return 0;
}
