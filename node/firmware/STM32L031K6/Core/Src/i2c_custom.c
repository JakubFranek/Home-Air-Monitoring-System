#include <stdint.h>
#include "main.h"
#include "tim_custom.h" // needed for TIM2-based timeout checks

#define I2C_TIMEOUT_US 200
#define I2C_TIMEOUT_TIMER TIM2

int8_t I2C1_transmit_byte(uint8_t address, uint8_t payload)
{
	uint16_t count;

	TIMx_restart(I2C_TIMEOUT_TIMER);

	LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT,
						  1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	while (!LL_I2C_IsActiveFlag_TXIS(I2C1))
	{
		TIMx_get_count(I2C_TIMEOUT_TIMER, &count);
		if (count > I2C_TIMEOUT_US)
			return -1;
	}

	LL_I2C_TransmitData8(I2C1, payload);

	TIMx_restart(I2C_TIMEOUT_TIMER);
	while (!LL_I2C_IsActiveFlag_STOP(I2C1))
	{
		TIMx_get_count(I2C_TIMEOUT_TIMER, &count);
			if (count > I2C_TIMEOUT_US)
				return -1;
	}

	LL_I2C_ClearFlag_STOP(I2C1);

	return 0;
}

int8_t I2C1_receive(uint8_t address, uint8_t *payload, uint8_t bytes)
{
	uint16_t count;

	LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT,
						  bytes, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

	uint8_t data[bytes];

	for (int i = 0; i < bytes; i++)
	{
		TIMx_restart(I2C_TIMEOUT_TIMER);
		while (!LL_I2C_IsActiveFlag_RXNE(I2C1))
		{
			TIMx_get_count(I2C_TIMEOUT_TIMER, &count);
				if (count > I2C_TIMEOUT_US)
					return -1;
		}
		data[i] = LL_I2C_ReceiveData8(I2C1);
	}

	while (!LL_I2C_IsActiveFlag_STOP(I2C1))
	{
		TIMx_get_count(I2C_TIMEOUT_TIMER, &count);
			if (count > I2C_TIMEOUT_US)
				return -1;
	}

	LL_I2C_ClearFlag_STOP(I2C1);

	for (int i = 0; i < bytes; i++)
	{
		payload[i] = data[i];
	}

	return 0;
}
