#ifndef INC_GPIO_CUSTOM_H_
#define INC_GPIO_CUSTOM_H_

void nrf24l01p_set_cs(uint8_t state);
void nrf24l01p_set_ce(uint8_t state);
void set_pins_to_analog_mode(GPIO_TypeDef* port, uint32_t pins);

#endif /* INC_GPIO_CUSTOM_H_ */
