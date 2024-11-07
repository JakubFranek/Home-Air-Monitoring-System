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

void set_pins_to_analog_mode(GPIO_TypeDef* port, uint32_t pins)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = pins;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(port, &GPIO_InitStruct);
}
