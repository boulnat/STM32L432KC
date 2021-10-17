/*
 * I2CREADWRITEREGISTER.h
 *
 *  Created on: Aug 6, 2021
 *      Author: boulnat
 */

ifndef I2CREADWRITEREGISTER_I2CREADWRITEREGISTER_H_
#define I2CREADWRITEREGISTER_I2CREADWRITEREGISTER_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <i2c.h>

/*!
 * @brief 	read single register
 * @param 	module: I2C pointer
 * 			address: address of the device I2C
 * 			reg: register in the device that we want to read from
 * @return 	value in 8bit from the reg of the device at the address
 */
uint8_t readSingleRegister(I2C_HandleTypeDef *module, uint8_t address, uint8_t reg);

/*!
 * @brief 	read double register
 * @param 	module: I2C pointer
 * 			address: address of the device I2C
 * 			reg: register in the device that we want to read from
 * @return 	value in 16bit from the reg of the device at the address
 */
uint16_t readDoubleRegister(I2C_HandleTypeDef *module, uint8_t address, uint8_t reg);

/*!
 * @brief 	write single register
 * @param 	module: I2C pointer
 * 			address: address of the device I2C
 * 			reg: register in the device that we want to read from
 * 			data: value in 8bit to write in the reg of the device at the address
 * @return 	return 0 if successfull write
 */
bool  writeSingleRegister(I2C_HandleTypeDef *module, uint8_t address, uint8_t reg, uint8_t data);

/*!
 * @brief 	write double register
 * @param 	module: I2C pointer
 * 			address: address of the device I2C
 * 			reg: register in the device that we want to read from
 * 			data: value in 16bit to write in the reg of the device at the address
 * @return 	return 0 if successfull write
 */
bool  writeDoubleRegister(I2C_HandleTypeDef *module, uint8_t address, uint8_t reg, uint16_t data);

uint8_t readSingleRegister[2];
uint8_t readSingleValue;

uint8_t writeSingleRegister[2];
uint8_t writeSingleValue;

uint16_t readDouvleRegister[2];
uint16_t readDoubleValue;

uint16_t writeDouvleRegister[3];
uint16_t writeDoubleValue;

#endif /* I2CREADWRITEREGISTER_I2CREADWRITEREGISTER_H_ */
