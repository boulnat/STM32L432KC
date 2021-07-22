/*
 * MCP9600.c
 *
 *  Created on: Mar 29, 2021
 *      Author: boulnat
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <main.h>
#include <MCP9600.h>

bool PCM9600begin(PCM9600_t *module, I2C_HandleTypeDef hi2c1){
	module->hi2c = hi2c1;
	module->sensor_ID = 0x80;
	return 0;
}
/*
bool available()
{
  //uint8_t status = readSingleRegister(SENSOR_STATUS);
  //return status;
}

uint16_t deviceID()
{
  //return readDoubleRegister(DEVICE_ID);
}

bool checkDeviceID()
{
  deviceID(); //this is here because the first read doesn't seem to work, but the second does. No idea why :/
  return ((deviceID())>>8 == DEV_ID_UPPER);
}
*/
bool resetToDefaults()
{
  bool success = writeSingleRegister(SENSOR_STATUS, 0x00);
  success |= writeSingleRegister(THERMO_SENSOR_CONFIG, 0x00);
  success |= writeSingleRegister(DEVICE_CONFIG, 0x00);
  success |= writeSingleRegister(ALERT1_CONFIG, 0x00);
  success |= writeSingleRegister(ALERT2_CONFIG, 0x00);
  success |= writeSingleRegister(ALERT3_CONFIG, 0x00);
  success |= writeSingleRegister(ALERT4_CONFIG, 0x00);
  success |= writeSingleRegister(ALERT1_HYSTERESIS, 0x00);
  success |= writeSingleRegister(ALERT2_HYSTERESIS, 0x00);
  success |= writeSingleRegister(ALERT3_HYSTERESIS, 0x00);
  success |= writeSingleRegister(ALERT4_HYSTERESIS, 0x00);
  success |= writeDoubleRegister(ALERT1_LIMIT, 0x0000);
  success |= writeDoubleRegister(ALERT2_LIMIT, 0x0000);
  success |= writeDoubleRegister(ALERT3_LIMIT, 0x0000);
  success |= writeDoubleRegister(ALERT4_LIMIT, 0x0000);
  return success;
}

/*----------------------------- Sensor Measurements ---------------------*/

uint8_t getThermocoupleTemp(PCM9600_t *module, bool units)
{
  int16_t raw = readDoubleRegister(module, HOT_JUNC_TEMP);
  uint8_t LSB = raw & 0x00FF;
  uint8_t MSB = raw>>8;

  if((MSB & 0x80) == 0x80){
	  return(((MSB*16)+(LSB/16))-4096);
  }
  else{
	  return(((MSB*16)+(LSB/16)));
  }
}
/*
uint8_t getAmbientTemp(bool units)
{

  int16_t raw = readDoubleRegister(COLD_JUNC_TEMP);
  uint8_t LSB = raw & 0x00FF;
  uint8_t MSB = raw>>8;

  if((MSB & 0x80) == 0x80){
	  return(((MSB*16)+(LSB/16))-4096);
  }
  else{
	  return(((MSB*16)+(LSB/16)));
  }

}

uint8_t getTempDelta(bool units)
{

  int16_t raw = readDoubleRegister(DELTA_JUNC_TEMP);
  uint8_t LSB = raw & 0x00FF;
  uint8_t MSB = raw>>8;

  if((MSB & 0x80) == 0x80){
	  return(((MSB*16)+(LSB/16))-4096);
  }
  else{
	  return(((MSB*16)+(LSB/16)));
  }

}
*/
signed long getRawADC()
{
	uint8_t cmd = 0x03;
	uint8_t read[3]= {0,0,0};
	while(HAL_I2C_Master_Transmit(&hi2c, 0xCE, &cmd, 1, HAL_MAX_DELAY)!=HAL_OK);
	while(HAL_I2C_IsDeviceReady(&hi2c,0xCE,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c,0xCF,read,sizeof(read),HAL_MAX_DELAY) == HAL_OK);

	signed long data = read[2];
    data |= read[1];
    data |= read[0];

    return data;

}

bool isInputRangeExceeded()
{
  uint8_t status = (4 << readSingleRegister(SENSOR_STATUS));
  return status;
}

/*--------------------------- Measurement Configuration --------------- */

bool setAmbientResolution(Ambient_Resolution res)
{
  uint8_t config = readSingleRegister(DEVICE_CONFIG); //get current device configuration so we don't wipe everything else
  config^=1<<7;                           //set the bit that controls the ambient (cold) junction resolution

  bool failed = writeSingleRegister(DEVICE_CONFIG, config); //write new config register to MCP9600
  failed |= (readSingleRegister(DEVICE_CONFIG) != config);  //double check that it was set properly
  return failed;                                            //return 1 if the write failed or the register wasn't properly set, 0 otherwise
}

Ambient_Resolution getAmbientResolution()
{
  uint8_t config = readSingleRegister(DEVICE_CONFIG);         //grab current device configuration
  return (7<<config); //return 7th bit from config register
}

bool setThermocoupleResolution(Thermocouple_Resolution res)
{
  uint8_t config = readSingleRegister(DEVICE_CONFIG); //grab current device configuration so we don't wipe everything else
  //bool highResolutionBit = (1 << res);
  //bool lowResolutionBit = (0 << res);
  config |= 1UL << 6;//set 6th bit of config register to 1st bit of the resolution
  config &=~(1UL << 5);//set 5th bit of config register to 0th bit of the resolution

  bool failed = writeSingleRegister(DEVICE_CONFIG, config); //write new config register to MCP9600
  failed |= (readSingleRegister(DEVICE_CONFIG) != config);  //double check that it was written properly
  return failed;                                            //return 1 if the write failed or the register wasn't properly set, 0 otherwise
}

Thermocouple_Resolution getThermocoupleResolution()
{
  uint8_t config = readSingleRegister(DEVICE_CONFIG); //grab current device configuration
  uint8_t res=0;                                        //define new thermocoupleResolution enum to return
  //bool highResolutionBit = (6 << config);
  //bool lowResolutionBit = (5 << config);
  config |= 1UL << 6; //set 1st bit of the enum to the 6th bit of the config register
  config &=~(1UL << 5);  //set 0th bit of the enum to the 5th bit of the config register
  return (res);
}

uint8_t setThermocoupleType(Thermocouple_Type type)
{
  uint8_t config = readSingleRegister(THERMO_SENSOR_CONFIG); //grab current device configuration so we don't wipe everything else
  //bitClear(config, 4);                                       //clear the necessary bits so that they can be set
  //bitClear(config, 5);
  //bitClear(config, 6);
  config |= (type << 4); //set the necessary bits in the config register
  if (writeSingleRegister(THERMO_SENSOR_CONFIG, config))
    return 1; //if write fails, return 1
  if (readSingleRegister(THERMO_SENSOR_CONFIG) != config)
    return 2; //if the register didn't take the new value, return 2

  return 0; //otherwise return 0
}

Thermocouple_Type  getThermocoupleType()
{
  uint8_t config = readSingleRegister(THERMO_SENSOR_CONFIG);
  return (Thermocouple_Type)(config >> 4); //clear the non-thermocouple-type bits in the config registe
}
/*
uint8_t  setFilterCoefficient(uint8_t coefficient)
{
  if (coefficient > 7)
    return 3; //return immediately if the value is too big

  uint8_t config = readSingleRegister(THERMO_SENSOR_CONFIG);
  bitWrite(coefficient, 3, bitRead(config, 3));
  bitWrite(coefficient, 4, bitRead(config, 3));
  bitWrite(coefficient, 5, bitRead(config, 3));
  bitWrite(coefficient, 6, bitRead(config, 3));
  bitWrite(coefficient, 7, bitRead(config, 3));

  //config = config >> 3;
  //config = config << 3;
  //config |= coefficient; //set the necessary bits in the config register

  return writeSingleRegister(THERMO_SENSOR_CONFIG, coefficient);
}

uint8_t  getFilterCoefficient()
{
  uint8_t config = readSingleRegister(THERMO_SENSOR_CONFIG);
  uint8_t coeff = 0;
  bitWrite(coeff, 0, bitRead(config, 0));
  bitWrite(coeff, 1, bitRead(config, 1));
  bitWrite(coeff, 2, bitRead(config, 2));

  return coeff; //clear the non-filter-Coefficient data in the config register
}

bool  setBurstSamples(Burst_Sample samples)
{
  uint8_t config = readSingleRegister(DEVICE_CONFIG);
  bool highResolutionBit = bitRead(samples, 2);
  bool midResolutionBit = bitRead(samples, 1);
  bool lowResolutionBit = bitRead(samples, 0);
  bitWrite(config, 4, highResolutionBit); //write 2nd bit of samples to 4th of config
  bitWrite(config, 3, midResolutionBit);  //write 1st bit of samples to 3rd of config
  bitWrite(config, 2, lowResolutionBit);  //write 0th bit of samples to 2nd of config

  bool failed = writeSingleRegister(DEVICE_CONFIG, config); //write new config register to MCP9600
  failed |= (readSingleRegister(DEVICE_CONFIG) != config);  //double check that it was written properly
  return failed;                                            //return 1 if the write failed or the register wasn't properly set, 0 otherwise
}

Burst_Sample  getBurstSamples()
{
  uint8_t config = readSingleRegister(DEVICE_CONFIG);
  bool highResolutionBit = bitRead(config, 4);
  bool midResolutionBit = bitRead(config, 3);
  bool lowResolutionBit = bitRead(config, 2);
  uint8_t samples;
  bitWrite(samples, 2, highResolutionBit); //write 4th bit of config to 2nd bit of samples
  bitWrite(samples, 1, midResolutionBit);  //write 3rd bit of config to 1st bit of samples
  bitWrite(samples, 0, lowResolutionBit);  //write 2nd bit of config to 0th bit of samples
  return static_cast<Burst_Sample>(samples);
}

bool  burstAvailable()
{
  uint8_t status = readSingleRegister(SENSOR_STATUS);
  return (status >> 7); //return only the 7th bit where the burst complete flag is
}

bool  startBurst()
{
  uint8_t status = readSingleRegister(SENSOR_STATUS);
  bitWrite(status, 7, 0); //clear the 7th bit of the status register, and send over I2C

  bool failed = writeSingleRegister(SENSOR_STATUS, status); //return whether the write was successful
  failed |= setShutdownMode(BURST);

  return failed;
}

bool  setShutdownMode(Shutdown_Mode mode)
{
  uint8_t config = readSingleRegister(DEVICE_CONFIG);
  config = (config >> 2) << 2; //clear last two bits of the device config register
  config |= mode;

  bool failed = writeSingleRegister(DEVICE_CONFIG, config); //write new config register to MCP9600
  failed |= (readSingleRegister(DEVICE_CONFIG) != config);  //double check that it was written properly
  return failed;                                            //return 1 if the write failed or the register wasn't properly set, 0 otherwise
}

Shutdown_Mode  getShutdownMode()
{
  uint8_t config = readSingleRegister(DEVICE_CONFIG);
  uint8_t mode = 0;
  bitWrite(mode, 0, bitRead(config, 0));
  bitWrite(mode, 1, bitRead(config, 1));
  return static_cast<Shutdown_Mode>(mode); //clear all bits except the last two and return
}

*/

/*------------------------- Internal I2C Abstraction ---------------- */

uint8_t readSingleRegister(MCP9600_Register reg)
{
  //Attempt to read the register until we exit with no error code
  //This attempts to fix the bug where clock stretching sometimes failes, as
  //described in the MCP9600 eratta
  uint8_t read[1]={0};
  uint8_t read8bits = 0;


  while(HAL_I2C_Master_Transmit(&hi2c, 0xCE, &reg, 1, HAL_MAX_DELAY) != HAL_OK);
  while(HAL_I2C_IsDeviceReady(&hi2c,0xCE,10,200)!=HAL_OK);
  while(HAL_I2C_Master_Receive(&hi2c, 0XCF, read, sizeof(read), HAL_MAX_DELAY)!= HAL_OK);
  read8bits = read[0];
  return read8bits;

}

uint16_t readDoubleRegister(PCM9600_t *module, MCP9600_Register reg)
{
  //Attempt to read the register until we exit with no error code
  //This attempts to fix the bug where clock stretching sometimes failes, as
  //described in the MCP9600 eratta
	uint8_t read[2]={0,0};
	uint16_t read16bits = 0;

	while(HAL_I2C_Master_Transmit(&module->hi2c, 0xCE, &reg, 1, HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&module->hi2c,0xCE,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(&module->hi2c, 0XCF, read, sizeof(read), HAL_MAX_DELAY)!= HAL_OK);
	read16bits = (read[0] << 8) | read[1];
	return read16bits;


}

bool  writeSingleRegister(MCP9600_Register reg, uint8_t data)
{
	uint8_t write[2]={reg, data};

	while(HAL_I2C_Master_Transmit(&hi2c, 0xCE, write, sizeof(write), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&hi2c,0xCE,10,200)!=HAL_OK);

	return 0;

}

bool  writeDoubleRegister(MCP9600_Register reg, uint16_t data)
{
  uint8_t highByte = data >> 8;
  uint8_t lowByte = data & 0xFF;
  uint8_t initStruct[3]={reg, highByte, lowByte};

  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c, _deviceAddress, initStruct, sizeof(initStruct), 1);
  return (status == HAL_OK);
}
