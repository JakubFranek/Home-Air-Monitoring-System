#include <stdint.h>
#include "main.h"

/*---------- Functions to be passed to nRF24 driver via pointer ---------*/
void nrf24l01p_set_cs(uint8_t state)
{
	if (state)
		LL_GPIO_SetOutputPin(nRF24_CSN_GPIO_Port, nRF24_CSN_Pin);
	else
		LL_GPIO_ResetOutputPin(nRF24_CSN_GPIO_Port, nRF24_CSN_Pin);
}

void nrf24l01p_set_ce(uint8_t state)
{
	if (state)
		LL_GPIO_SetOutputPin(nRF24_CE_GPIO_Port, nRF24_CE_Pin);
	else
		LL_GPIO_ResetOutputPin(nRF24_CE_GPIO_Port, nRF24_CE_Pin);
}
