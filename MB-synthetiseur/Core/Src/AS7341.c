/*
 * AS7341.c
 *
 *  Created on: Mar 30, 2021
 *      Author: boulnat
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <main.h>
#include <AS7341.h>

uint16_t readChannel(I2C_HandleTypeDef *hi2c1, as7341_adc_channel_t channel) {

	uint8_t read[2];
	uint16_t read16bits = 0;
	uint8_t regCh[] = {0x61,0x62};
	uint8_t data[] = {0xA9, 0x08};
	//uint8_t data[] = {0x80, 0x01};
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, data, sizeof(data), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);


	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, regCh, 2, HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(hi2c1, 0x72, read, sizeof(read), HAL_MAX_DELAY)!= HAL_OK);


	read16bits = (read[0] << 8) | read[1];
	return read16bits;

}

uint16_t getChannel(as7341_color_channel_t channel) {
  return _channel_readings[channel];
}

uint16_t readAllChannels(I2C_HandleTypeDef *hi2c1,uint16_t *readings_buffer) {
    __disable_irq();
    for(int i=0; i<12; i++){
        _channel_readings[i]=0;
    }

	setSMUXLowChannels(hi2c1,true);        // Configure SMUX to read low channels
  	enableSpectralMeasurement(hi2c1,true); // Start integration
  	delayForData(0);                 // I'll wait for you for all time

	//uint8_t read[12];
	uint16_t read16bits = 0;
	uint8_t regCh[] = {0x95};
	//uint8_t data[] = {0xA9, 0x08};

	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, regCh, sizeof(regCh), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(hi2c1, 0x72, _channel_readings, sizeof(_channel_readings), HAL_MAX_DELAY)!= HAL_OK);

  setSMUXLowChannels(hi2c1,false);       // Configure SMUX to read high channels
  enableSpectralMeasurement(hi2c1,true); // Start integration
  delayForData(0);                 // I'll wait for you for all time

  read16bits = 1; //(_channel_readings[0] << 8) | _channel_readings[1];
  __enable_irq();

  return read16bits;
}

void setSMUXLowChannels(I2C_HandleTypeDef *hi2c1, bool f1_f4) {
  enableSpectralMeasurement(hi2c1,false);
  setSMUXCommand(hi2c1,AS7341_SMUX_CMD_WRITE);
  if (f1_f4) {
    setup_F1F4_Clear_NIR(hi2c1);
  } else {
    setup_F5F8_Clear_NIR(hi2c1);
  }
  enableSMUX(hi2c1);
}

bool setSMUXCommand(I2C_HandleTypeDef *hi2c1 ,as7341_smux_cmd_t command) {

	uint8_t regCh[] = {0xAF,0x08, command};

	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, regCh, sizeof(regCh), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);

  return 1;
}

bool enableSpectralMeasurement(I2C_HandleTypeDef *hi2c1, bool enable_measurement) {

	uint8_t regCh[] = {0x80, 0x08};
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, regCh, sizeof(regCh), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);

  return 1;
}
bool enableSMUX(I2C_HandleTypeDef *hi2c1) {

	uint8_t regCh[] = {0x80, 0x10};
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, regCh, sizeof(regCh), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);

    return 1;
}

void delayForData(int waitTime) {
  osDelay(waitTime);
  /*
  if (waitTime == 0) // Wait forever
  {
    while (!getIsDataReady()) {
      delay(1);
    }
    return;
  }
  if (waitTime > 0) // Wait for that many milliseconds
  {
    uint32_t elapsedMillis = 0;
    while (!getIsDataReady() && elapsedMillis < waitTime) {
      delay(1);
      elapsedMillis++;
    }
    return;
  }
  if (waitTime < 0) {
    // For future use?
    return;
  }
  */
}

void setup_F1F4_Clear_NIR(I2C_HandleTypeDef *hi2c1) {
  // SMUX Config for F1,F2,F3,F4,NIR,Clear
  writeRegister(hi2c1, (0x00), (0x30)); // F3 left set to ADC2
  writeRegister(hi2c1, (0x01), (0x01)); // F1 left set to ADC0
  writeRegister(hi2c1, (0x02), (0x00)); // Reserved or disabled
  writeRegister(hi2c1, (0x03), (0x00)); // F8 left disabled
  writeRegister(hi2c1, (0x04), (0x00)); // F6 left disabled
  writeRegister(hi2c1, (0x05), (0x42)); // F4 left connected to ADC3/f2 left connected to ADC1
  writeRegister(hi2c1, (0x06), (0x00)); // F5 left disbled
  writeRegister(hi2c1, (0x07), (0x00)); // F7 left disbled
  writeRegister(hi2c1, (0x08), (0x50)); // CLEAR connected to ADC4
  writeRegister(hi2c1, (0x09), (0x00)); // F5 right disabled
  writeRegister(hi2c1, (0x0A), (0x00)); // F7 right disabled
  writeRegister(hi2c1, (0x0B), (0x00)); // Reserved or disabled
  writeRegister(hi2c1, (0x0C), (0x20)); // F2 right connected to ADC1
  writeRegister(hi2c1, (0x0D), (0x04)); // F4 right connected to ADC3
  writeRegister(hi2c1, (0x0E), (0x00)); // F6/F8 right disabled
  writeRegister(hi2c1, (0x0F), (0x30)); // F3 right connected to AD2
  writeRegister(hi2c1, (0x10), (0x01)); // F1 right connected to AD0
  writeRegister(hi2c1, (0x11), (0x50)); // CLEAR right connected to AD4
  writeRegister(hi2c1, (0x12), (0x00)); // Reserved or disabled
  writeRegister(hi2c1, (0x13), (0x06)); // NIR connected to ADC5
}

void setup_F5F8_Clear_NIR(I2C_HandleTypeDef *hi2c1) {
  // SMUX Config for F5,F6,F7,F8,NIR,Clear
  writeRegister(hi2c1, (0x00), (0x00)); // F3 left disable
  writeRegister(hi2c1, (0x01), (0x00)); // F1 left disable
  writeRegister(hi2c1, (0x02), (0x00)); // reserved/disable
  writeRegister(hi2c1, (0x03), (0x40)); // F8 left connected to ADC3
  writeRegister(hi2c1, (0x04), (0x02)); // F6 left connected to ADC1
  writeRegister(hi2c1, (0x05), (0x00)); // F4/ F2 disabled
  writeRegister(hi2c1, (0x06), (0x10)); // F5 left connected to ADC0
  writeRegister(hi2c1, (0x07), (0x03)); // F7 left connected to ADC2
  writeRegister(hi2c1, (0x08), (0x50)); // CLEAR Connected to ADC4
  writeRegister(hi2c1, (0x09), (0x10)); // F5 right connected to ADC0
  writeRegister(hi2c1, (0x0A), (0x03)); // F7 right connected to ADC2
  writeRegister(hi2c1, (0x0B), (0x00)); // Reserved or disabled
  writeRegister(hi2c1, (0x0C), (0x00)); // F2 right disabled
  writeRegister(hi2c1, (0x0D), (0x00)); // F4 right disabled
  writeRegister(hi2c1, (0x0E), (0x24)); // F8 right connected to ADC2/ F6 right connected to ADC1
  writeRegister(hi2c1, (0x0F), (0x00)); // F3 right disabled
  writeRegister(hi2c1, (0x10), (0x00)); // F1 right disabled
  writeRegister(hi2c1, (0x11), (0x50)); // CLEAR right connected to AD4
  writeRegister(hi2c1, (0x12), (0x00)); // Reserved or disabled
  writeRegister(hi2c1, (0x13), (0x06)); // NIR connected to ADC5
}

void writeRegister(I2C_HandleTypeDef *hi2c1, uint8_t addr, uint8_t val) {
	uint8_t data[] = {addr, val};
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, data, sizeof(data), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);

}

bool setATIME(I2C_HandleTypeDef *hi2c1, uint8_t atime_value) {
	uint8_t data[] = {AS7341_ATIME, atime_value};
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, data, sizeof(data), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);

	return 1;
}

uint8_t getATIME(I2C_HandleTypeDef *hi2c1) {
	uint8_t regtest[]={AS7341_ATIME};
	uint8_t regRead[1]={0};
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, regtest, 1, HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(hi2c1, 0x73, regRead, sizeof(regRead), HAL_MAX_DELAY)!= HAL_OK);
  return regRead[0];
}

bool setASTEP(I2C_HandleTypeDef *hi2c1, uint8_t astep_value) {
	uint8_t data[] = {AS7341_ASTEP_L, astep_value};
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, data, sizeof(data), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);

	data[0] = AS7341_ASTEP_H;
	data[1] = 0x03;
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, data, sizeof(data), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);

	return 1;
}

uint16_t getASTEP(I2C_HandleTypeDef *hi2c1) {
	uint8_t regtest[]={AS7341_ASTEP_L};
	uint16_t regRead[1]={0};
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, regtest, 1, HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(hi2c1, 0x72, regRead, sizeof(regRead), HAL_MAX_DELAY)!= HAL_OK);
  return regRead[0];

}

bool setGain(I2C_HandleTypeDef *hi2c1, as7341_gain_t gain_value) {
	uint8_t data[] = {AS7341_CFG1, gain_value};
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, data, sizeof(data), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);

	return 1;
  // AGAIN bitfield is only[0:4] but the rest is empty
}

as7341_gain_t getGain(I2C_HandleTypeDef *hi2c1) {
	uint8_t regtest[]={AS7341_CFG1};
	uint8_t regRead[1]={0};
	while(HAL_I2C_Master_Transmit(hi2c1, 0x72, regtest, 1, HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(hi2c1,0x72,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(hi2c1, 0x72, regRead, sizeof(regRead), HAL_MAX_DELAY)!= HAL_OK);

  return (as7341_gain_t)regRead[0];
}
