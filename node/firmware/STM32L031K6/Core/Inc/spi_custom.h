#ifndef INC_SPI_CUSTOM_H_
#define INC_SPI_CUSTOM_H_

int8_t SPI1_Transmit(uint8_t tx_data);
int8_t SPI1_Transmit_Multi(uint8_t* tx_data, uint8_t length);
int8_t SPI1_Receive(uint8_t *rx_data);
int8_t SPI1_TransmitReceive(uint8_t tx_data, uint8_t *rx_data);

#endif /* INC_SPI_CUSTOM_H_ */
