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

void reinitialize_gpio(void)
{
	LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
	  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

	  /**/
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = LED_ERROR_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(LED_ERROR_GPIO_Port, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = LED_STATUS_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = nRF24_CE_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(nRF24_CE_GPIO_Port, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = nRF24_CSN_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(nRF24_CSN_GPIO_Port, &GPIO_InitStruct);

	  /**/
	  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE4);

	  /**/
	  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE15);

	  /**/
	  LL_GPIO_SetPinPull(BUTTON_LED_GPIO_Port, BUTTON_LED_Pin, LL_GPIO_PULL_NO);

	  /**/
	  LL_GPIO_SetPinPull(nRF24_IRQ_GPIO_Port, nRF24_IRQ_Pin, LL_GPIO_PULL_NO);

	  /**/
	  LL_GPIO_SetPinMode(BUTTON_LED_GPIO_Port, BUTTON_LED_Pin, LL_GPIO_MODE_INPUT);

	  /**/
	  LL_GPIO_SetPinMode(nRF24_IRQ_GPIO_Port, nRF24_IRQ_Pin, LL_GPIO_MODE_INPUT);

	  /**/
	  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
	  EXTI_InitStruct.LineCommand = ENABLE;
	  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
	  LL_EXTI_Init(&EXTI_InitStruct);

	  /**/
	  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_15;
	  EXTI_InitStruct.LineCommand = ENABLE;
	  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
	  LL_EXTI_Init(&EXTI_InitStruct);

	  /* EXTI interrupt init*/
	  NVIC_SetPriority(EXTI4_15_IRQn, 0);
	  NVIC_EnableIRQ(EXTI4_15_IRQn);
}
