/*
 * I2CREADWRITEREGISTER.c
 *
 *  Created on: Aug 6, 2021
 *      Author: boulnat
 */
#ifndef I2CREADWRITEREGISTER_I2CREADWRITEREGISTER_H_
#define I2CREADWRITEREGISTER_I2CREADWRITEREGISTER_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <I2CREADWRITEREGISTER.h>

uint8_t readSingleRegister(I2C_HandleTypeDef *module, uint8_t address, uint8_t reg){
	  readSingleValue = 0;

	  //module I2C ask at the address infromation avout the reg of size 1
	  //wait for device to be ready for more contact
	  //
	  while(HAL_I2C_Master_Transmit(&module, address, reg, 1, HAL_MAX_DELAY) != HAL_OK);
	  while(HAL_I2C_IsDeviceReady(&module,address,10,200)!=HAL_OK);
	  while(HAL_I2C_Master_Receive(&module, 0XCF, readSingleRegister, sizeof(readSingleRegister), HAL_MAX_DELAY)!= HAL_OK);

	  readSingleValue = readSingleRegister[0];
	  return readSingleValue;
}

uint16_t readDoubleRegister(I2C_HandleTypeDef *module, uint8_t address, uint8_t reg){
	readDoubleValue = 0;

	while(HAL_I2C_Master_Transmit(&module, address, reg, 1, HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&module,address,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(&module, 0XCF, readDouvleRegister, sizeof(readDouvleRegister), HAL_MAX_DELAY)!= HAL_OK);
	readDoubleValue = (readDouvleRegister[0] << 8) | readDouvleRegister[1];
	return readDoubleValue;
}

bool  writeSingleRegister(I2C_HandleTypeDef *module, uint8_t address, uint8_t reg, uint8_t data){
	writeSingleRegister[0] = reg;
	writeSingleRegister[1] = data;

	while(HAL_I2C_Master_Transmit(&module, address, writeSingleRegister, sizeof(writeSingleRegister), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&module, address, 10, 200)!=HAL_OK);
	return 0;
}

bool  writeDoubleRegister(I2C_HandleTypeDef *module, uint8_t address, uint8_t reg, uint16_t data){
	// TODO to be implemented
}
