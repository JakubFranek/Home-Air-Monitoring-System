#ifndef INC_SPI_CUSTOM_H_
#define INC_SPI_CUSTOM_H_

int8_t SPI1_transmit(const uint8_t* tx_data, uint8_t length);
int8_t SPI1_receive(uint8_t *rx_data, uint8_t length);
int8_t SPI1_transmit_receive(const uint8_t* tx_data, uint8_t *rx_data, uint8_t length);

#endif /* INC_SPI_CUSTOM_H_ */
