/*
 * i2c_custom.h
 *
 *  Created on: Oct 31, 2024
 *      Author: jfran
 */

#ifndef INC_I2C_CUSTOM_H_
#define INC_I2C_CUSTOM_H_

int8_t I2C1_transmit_byte(uint8_t address, uint8_t payload);
int8_t I2C1_receive(uint8_t address, uint8_t *payload, uint8_t bytes);

#endif /* INC_I2C_CUSTOM_H_ */
