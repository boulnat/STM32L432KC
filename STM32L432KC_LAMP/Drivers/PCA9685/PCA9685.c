/*
 * PCA9685.c
 *
 *  Created on: Apr 1, 2021
 *      Author: boulnat
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <main.h>
#include <PCA9685.h>

PCA9685_ReturnError_t errPCA9685 = 0;

bool PCA9685begin(PCA9685_t *module, I2C_HandleTypeDef hi2c1, uint8_t prescale){
	module->hi2c = hi2c1;
	module->sensor_ID = 0x80;
	return 0;
}

void reset(){
	uint8_t data[] = {PCA9685_MODE1, MODE1_RESTART};
	//while(HAL_I2C_Master_Transmit(&hi2c, PCA9685_ADR, data, sizeof(data), HAL_MAX_DELAY) != HAL_OK);
	//while(HAL_I2C_IsDeviceReady(&hi2c,PCA9685_ADR,10,200)!=HAL_OK);
}

void sleep();

void wakeup();

void setExtClk(uint8_t prescale);
void setPWMFreq(float freq);
void setOutputMode(bool totempole);
uint8_t getPWM(uint8_t num);
void setPWM(uint8_t num, uint16_t on, uint16_t off);
uint8_t readPrescale(void);
void writeMicroseconds(uint8_t num, uint16_t Microseconds);

void setOscillatorFrequency(uint32_t freq);
uint32_t getOscillatorFrequency(void);

uint8_t PCA9685_read(PCA9685_t module, unsigned char reg)
{
	uint8_t res={0};
	HAL_I2C_Master_Transmit(&module.hi2c, module.sensor_ID, 0x00, 1, 1);
	//HAL_I2C_Master_Receive(hi2c,address,res,1,1);
	return res;
}

void pca9685_init(PCA9685_t *module)
{
 uint8_t initStruct[2];
 uint8_t prescale = 0x03; // hardcoded
 errPCA9685 = HAL_I2C_Master_Transmit(&module->hi2c, 0x80, PCA9685_MODE1, 1, 1);
 uint8_t oldmode = 0x00; // hardcoded
 //uint8_t oldmode = PCA9685_read(hi2c,address,PCA9685_MODE1);
 // HAL_I2C_Master_Receive(hi2c, address, &oldmode, 1, 1);
 uint8_t newmode = ((oldmode & 0x7F) | 0x10);
 initStruct[0] = PCA9685_MODE1;
 initStruct[1] = newmode;
 errPCA9685 = HAL_I2C_Master_Transmit(&module->hi2c, module->sensor_ID, initStruct, 2, 1);
 //initStruct[0] = 0xFE;
 initStruct[1] = prescale;
 errPCA9685 = HAL_I2C_Master_Transmit(&module->hi2c, module->sensor_ID, initStruct, 2, 1);
 //initStruct[0] = PCA9685_MODE1;
 initStruct[1] = oldmode;
 errPCA9685 = HAL_I2C_Master_Transmit(&module->hi2c, module->sensor_ID, initStruct, 2, 1);
 //osDelay(5);
 initStruct[1] = (oldmode | 0xA1);
 errPCA9685 = HAL_I2C_Master_Transmit(&module->hi2c, module->sensor_ID, initStruct, 2, 1);
 //turn off all LED
 //all_led_off(address);
 if(errPCA9685){
	 errPCA9685=0;
 }
}

void pca9685_pwm(PCA9685_t *module, uint8_t num, uint16_t on, uint16_t off)
{
	uint8_t outputBuffer[] = {0x06 + 4*num, on, (on >> 8), off, (off >> 8)};
	HAL_I2C_Master_Transmit(&module->hi2c, module->sensor_ID, outputBuffer, sizeof(outputBuffer), 1);
}

void pca9685_mult_pwm(PCA9685_t *module, uint16_t num, uint16_t on, uint16_t off)
{


	int i, iter;

	for (i=1, iter=1; i<65535; i<<=1, iter++)
	{
		if (num & i)
		{
			uint8_t outputBuffer[] = {0x06 + 4*((iter)-1), on, (on >> 8), off, (off >> 8)};
			HAL_I2C_Master_Transmit(&module->hi2c, module->sensor_ID, outputBuffer, sizeof(outputBuffer), 1);
		}
		else
		{
			uint8_t outputBuffer[] = {0x06 + 4*((iter)-1), 0, (0 >> 8), 4096, (4096 >> 8)};
			HAL_I2C_Master_Transmit(&module->hi2c, module->sensor_ID, outputBuffer, sizeof(outputBuffer), 1);
		}
	}


	/*
	uint8_t channel = 0;
	if(num & 0b00000001)
	{
		uint8_t outputBuffer[] = {0x06 + 4*((num & 0b00000001)-1), on, (on >> 8), off, (off >> 8)};
		HAL_I2C_Master_Transmit(hi2c, address, outputBuffer, sizeof(outputBuffer), 1);
	}
	if(num & 0b00000010)
	{
		uint8_t outputBuffer[] = {0x06 + 4*((num & 0b00000010)-1), on, (on >> 8), off, (off >> 8)};
		HAL_I2C_Master_Transmit(hi2c, address, outputBuffer, sizeof(outputBuffer), 1);
	}
	*/
}

void all_led_off(PCA9685_t *module){

	 uint8_t ALL_LED_OFF = 0xFC;
	 uint8_t outputBuffer[] = {ALL_LED_OFF, 0, (0 >> 8), 4095, (4096 >> 8)};
	 HAL_I2C_Master_Transmit(&module->hi2c, module->sensor_ID, outputBuffer, sizeof(outputBuffer), 1);
}

HAL_StatusTypeDef pca9685_all_pwm(PCA9685_t *module, uint16_t on, uint16_t off)
{
	uint8_t ALL_LED_ON = 0xFA;
	uint8_t outputBuffer[] = {ALL_LED_ON, on, (on >> 8), off, (off >> 8)};
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&module->hi2c, module->sensor_ID, outputBuffer, sizeof(outputBuffer), 1);
	return status;
}
