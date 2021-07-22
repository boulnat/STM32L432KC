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

as7341_ReturnError_t errAS7341 = AS7341_ERROR_NO;
HAL_StatusTypeDef status;
as7341_t as7341;
uint8_t as7341_enable_reg = 0;

//TESTED
/*
void AS7341begin(I2C_HandleTypeDef hi2c1){

}
*/
bool AS7341init(I2C_HandleTypeDef hi2c1, int32_t sensor_id){
	as7341.hi2c 			= hi2c1;

	as7341.sensor_ID 		= sensor_id;
	as7341.writing_ID		= 0x72;

	as7341.astep.address_L 	= AS7341_ASTEP_L;
	as7341.astep.address_H 	= AS7341_ASTEP_H;
	as7341.astep.value		= 1;

	as7341.atime.address 	= AS7341_ATIME;
	as7341.atime.value		= 999;

	as7341.gain.address		= AS7341_CFG1;

	as7341.integrationTime	= (as7341.atime.value + 1) * (as7341.astep.value + 1) * 2.78 / 1000;

	as7341_enable_reg = 0x01; /* set PON to 1 */
	errAS7341 = writeRegister(AS7341_ENABLE, as7341_enable_reg);
	return errAS7341;
}
//TESTED
as7341_ReturnError_t setASTEP(uint16_t  astep_value) {
	//make sure ASTEP is between 0 and 65534
	if(astep_value<0 || astep_value>=65535){
		return AS7341_ERROR_ASTEP_OUT_OF_RANGE;
	}

	//write to the LSB astep_value
	errAS7341 = writeRegister(AS7341_ASTEP_L, astep_value);
	//write to the MSB astep_value
	errAS7341 = writeRegister(AS7341_ASTEP_H, astep_value>>8);
	//copy value to register stm32 if everything is ok
	as7341.astep.value = astep_value;

	return errAS7341;
}

//TESTED
as7341_ReturnError_t setATIME(uint8_t atime_value) {
	//make sure ATIME is between 0 and 255
	if(atime_value<0 || atime_value>=255){
		return AS7341_ERROR_ATIME_OUT_OF_RANGE;
	}

	//uint8_t data[] = {as7341.atime.address, atime_value};
	//set atime on AS7341
	//uint8_t data[] = {AS7341_ATIME, atime_value};
	errAS7341 = writeRegister(AS7341_ATIME, atime_value);
	//status = HAL_I2C_Master_Transmit(&as7341.hi2c, as7341.writing_ID, data, sizeof(data), HAL_MAX_DELAY);
	//status = HAL_I2C_IsDeviceReady(&as7341.hi2c,as7341.writing_ID,10,200);

	as7341.atime.value = atime_value;

	return errAS7341;
}

//TESTED
as7341_ReturnError_t setGain(uint8_t gain_value) {
	//make sure GAIN is between 0 and 10
	if(gain_value<AS7341_GAIN_0_5X || gain_value>=AS7341_GAIN_512X){
		return AS7341_ERROR_ATIME_OUT_OF_RANGE;
	}

	//uint8_t data[] = {as7341.gain.address, gain_value};
	//set gain on AS7341
	//uint8_t data[] = {as7341.gain.address, gain_value};
	errAS7341 = writeRegister(AS7341_CFG1, gain_value);
	//status = HAL_I2C_Master_Transmit(&as7341.hi2c, as7341.writing_ID, data, sizeof(data), HAL_MAX_DELAY);
	//status = HAL_I2C_IsDeviceReady(&as7341.hi2c, as7341.writing_ID, 10, 200);

	as7341.gain.value = gain_value;

	return errAS7341;
}
/**
//TESTED
as7341_ReturnError_t getASTEP() {
	errAS7341 = readRegister16(as7341.astep.address_L, &as7341.astep.value, 1);
	return errAS7341;
}

//TESTED
as7341_ReturnError_t getATIME() {
	uint8_t data[] = {as7341.atime.address};
	errAS7341 = readRegister16(data, &as7341.atime.value, 1);
	return AS7341_ERROR_NO;
}

//TESTED
as7341_ReturnError_t getGain() {
	uint8_t data[] = {as7341.atime.address};
	//errAS7341 = readRegister8(as7341.gain.address, NULL, &as7341.gain.value, 1);
	return AS7341_ERROR_NO;
}
*/
//TESTED
long getTINT(){
	  getASTEP();
	  getATIME();
	  as7341.integrationTime = (as7341.atime.value + 1) * (as7341.astep.value + 1) * 2.78 / 1000;

	  return as7341.integrationTime;
}

//TESTED
float toBasicCounts(uint16_t raw){
	  float gain_val = 0;
	  uint8_t gain = as7341.gain.value;
	  switch (gain) {
	  case AS7341_GAIN_0_5X:
	    gain_val = 0.5;
	    break;
	  case AS7341_GAIN_1X:
	    gain_val = 1;
	    break;
	  case AS7341_GAIN_2X:
	    gain_val = 2;
	    break;
	  case AS7341_GAIN_4X:
	    gain_val = 4;
	    break;
	  case AS7341_GAIN_8X:
	    gain_val = 8;
	    break;
	  case AS7341_GAIN_16X:
	    gain_val = 16;
	    break;
	  case AS7341_GAIN_32X:
	    gain_val = 32;
	    break;
	  case AS7341_GAIN_64X:
	    gain_val = 64;
	    break;
	  case AS7341_GAIN_128X:
	    gain_val = 128;
	    break;
	  case AS7341_GAIN_256X:
	    gain_val = 256;
	    break;
	  case AS7341_GAIN_512X:
	    gain_val = 512;
	    break;
	  }
	  as7341.rawToBasicCounts = raw / (gain_val * (as7341.atime.value + 1) * (as7341.astep.value + 1) * 2.78 / 1000);
	  return as7341.rawToBasicCounts;
}

//TESTED
as7341_ReturnError_t readAllChannels(uint16_t *readings_buffer) {
	uint8_t regwrite[1];

	regwrite[0]=AS7341_CH0_DATA_L;

    errAS7341 = setSMUXLowChannels(1);        /* Configure SMUX to read low channels */
    if(errAS7341){return errAS7341;}

    errAS7341 = enableSpectralMeasurement(1); /* Start integration */
    if(errAS7341){return errAS7341;}
    //osDelay(500); /* IMPORTANT NEED TO GIVE TIME FOR SPECTRO TO GET READY */

    errAS7341 = delayForData(0);                 /* I'll wait for you for all time */
    if(errAS7341){return errAS7341;}


    errAS7341 = HAL_I2C_Master_Transmit(&as7341.hi2c, as7341.writing_ID, regwrite, 1, HAL_MAX_DELAY);
    errAS7341 = HAL_I2C_IsDeviceReady(&as7341.hi2c,as7341.writing_ID,10,200);

  	errAS7341 = HAL_I2C_Master_Transmit(&as7341.hi2c, as7341.writing_ID, regwrite, sizeof(regwrite), HAL_MAX_DELAY);
  	errAS7341 = HAL_I2C_IsDeviceReady(&as7341.hi2c,as7341.writing_ID,10,200);
  	errAS7341 = HAL_I2C_Master_Receive(&as7341.hi2c, as7341.writing_ID, (uint8_t *)as7341._channel_readings, 12, HAL_MAX_DELAY);

  	if(errAS7341){return errAS7341;}

  	errAS7341 = setSMUXLowChannels(0);       /* Configure SMUX to read high channels */
  	if(errAS7341){return errAS7341;}

  	errAS7341 = enableSpectralMeasurement(1); /* Start integration */
  	if(errAS7341){return errAS7341;}
  	//osDelay(500); /* IMPORTANT NEED TO GIVE TIME FOR SPECTRO TO GET READY */

  	errAS7341 = delayForData(0);                 /* I'll wait for you for all time */
  	if(errAS7341){return errAS7341;}

  	errAS7341 = HAL_I2C_Master_Transmit(&as7341.hi2c, as7341.writing_ID, regwrite, 1, HAL_MAX_DELAY);
  	errAS7341 = HAL_I2C_IsDeviceReady(&as7341.hi2c,as7341.writing_ID,10,200);

  	errAS7341 = HAL_I2C_Master_Transmit(&as7341.hi2c, as7341.writing_ID, regwrite, sizeof(regwrite), HAL_MAX_DELAY);
  	errAS7341 = HAL_I2C_IsDeviceReady(&as7341.hi2c,as7341.writing_ID,10,200);
  	errAS7341 = HAL_I2C_Master_Receive(&as7341.hi2c, as7341.writing_ID, (uint8_t *)&as7341._channel_readings[6], 12, HAL_MAX_DELAY);
  	if(errAS7341){return errAS7341;}

	return AS7341_ERROR_NO;
}

//TESTED
as7341_ReturnError_t delayForData(int waitTime) {
	if(waitTime == 0){
		while(!getIsDataReady()) {
			/* TODO add a timeout */
		}
		return AS7341_ERROR_NO;
	}

	return AS7341_ERROR_NO;
}
//TESTED
as7341_ReturnError_t readChannel(as7341_adc_channel_t channel) {

	if(channel<0 || channel>5){
		return AS7341_ERROR_READ_CH_OUT_OF_RANGE;
	}

	//uint16_t read[1];
	//uint16_t read16bits = 0;
	uint8_t regCh[] = {AS7341_CH0_DATA_L + 2 * channel};

	status = HAL_I2C_Master_Transmit(&as7341.hi2c, as7341.writing_ID, regCh, 2, HAL_MAX_DELAY);
	status = HAL_I2C_IsDeviceReady(&as7341.hi2c,as7341.writing_ID,10,200);
	status = HAL_I2C_Master_Receive(&as7341.hi2c, as7341.writing_ID, (uint8_t *)as7341._channel_readings[channel], 2, HAL_MAX_DELAY);

	if(status != HAL_OK){
		return status;
	}

	//as7341._channel_readings[channel] = ((as7341._channel_readings[channel] & 0x00FF) << 8) | (as7341._channel_readings[channel]>>8);
	return AS7341_ERROR_NO;
}
//TESTED
uint16_t getChannel(as7341_color_channel_t channel) {
	/*  Swap msb and lsb  */
	return (((as7341._channel_readings[channel] & 0x00FF) << 8) | (as7341._channel_readings[channel]>>8));
}
/* TODO */
bool startReading(void){
	as7341._readingState = AS7341_WAITING_START; // Start the measurement please
	errAS7341 = checkReadingProgress();          // Call the check function to start it
	if(errAS7341){return false;}
	return true;
}
/* TODO */
bool checkReadingProgress(){
	uint8_t regwrite[]={AS7341_CH0_DATA_L};

	  if (as7341._readingState == AS7341_WAITING_START) {
		errAS7341 = setSMUXLowChannels(true);        // Configure SMUX to read low channels
		errAS7341 = enableSpectralMeasurement(1); // Start integration
	    as7341._readingState = AS7341_WAITING_LOW;
	    return false;
	  }

	  if (!getIsDataReady() || as7341._readingState == AS7341_WAITING_DONE)
	    return false;

	  if (as7341._readingState == AS7341_WAITING_LOW) // Check of getIsDataRead() is already done
	  {
		while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regwrite, sizeof(regwrite), HAL_MAX_DELAY)!=HAL_OK);
		while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
		while(HAL_I2C_Master_Receive(&as7341.hi2c, 0x72, (uint8_t *)as7341._channel_readings, 12, HAL_MAX_DELAY)!=HAL_OK);

	    setSMUXLowChannels(false);       // Configure SMUX to read high channels
	    enableSpectralMeasurement(1); // Start integration
	    as7341._readingState = AS7341_WAITING_HIGH;
	    return false;
	  }

	  if (as7341._readingState == AS7341_WAITING_HIGH) // Check of getIsDataRead() is already done
	  {
		while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regwrite, sizeof(regwrite), HAL_MAX_DELAY)!=HAL_OK);
		while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
		while(HAL_I2C_Master_Receive(&as7341.hi2c, 0x72, (uint8_t *)&as7341._channel_readings[6], 12, HAL_MAX_DELAY)!=HAL_OK);
	    as7341._readingState = AS7341_WAITING_DONE;
	    return true;
	  }

	  return false;
}
/* TODO */
bool getAllChannels(uint16_t *readings_buffer){
	for (int i = 0; i < 12; i++){
		readings_buffer[i] = as7341._channel_readings[i];
	}
	return true;
}
/* TODO */
uint16_t detectFlickerHz(void){
	  //bool isEnabled = true;
	  //bool isFdmeasReady = false;

	  // disable everything; Flicker detect, smux, wait, spectral, power
	  disableAll();
	  // re-enable power
	  powerEnable(true);

	  // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10
	  // to CFG6)
	  setSMUXCommand(AS7341_SMUX_CMD_WRITE);

	  // Write new configuration to all the 20 registers for detecting Flicker
	  FDConfig();

	  // Start SMUX command
	  enableSMUX();

	  // Enable SP_EN bit
	  enableSpectralMeasurement(true);

	  // Enable flicker detection bit
	  writeRegister(AS7341_ENABLE, 0x41);
	  //osDelay(500); // SF 2020-08-12 Does this really need to be so long?
	  uint16_t flicker_status = getFlickerDetectStatus();
	  enableFlickerDetection(false);
	  switch (flicker_status) {
	  case 44:
	    return 1;
	  case 45:
	    return 100;
	  case 46:
	    return 120;
	  default:
	    return 0;
	  }
}

void setup_F1F4_Clear_NIR() {
  // SMUX Config for F1,F2,F3,F4,NIR,Clear
  writeRegister((0x00), (0x30)); // F3 left set to ADC2
  writeRegister((0x01), (0x01)); // F1 left set to ADC0
  writeRegister((0x02), (0x00)); // Reserved or disabled
  writeRegister((0x03), (0x00)); // F8 left disabled
  writeRegister((0x04), (0x00)); // F6 left disabled
  writeRegister((0x05), (0x42)); // F4 left connected to ADC3/f2 left connected to ADC1
  writeRegister((0x06), (0x00)); // F5 left disbled
  writeRegister((0x07), (0x00)); // F7 left disbled
  writeRegister((0x08), (0x50)); // CLEAR connected to ADC4
  writeRegister((0x09), (0x00)); // F5 right disabled
  writeRegister((0x0A), (0x00)); // F7 right disabled
  writeRegister((0x0B), (0x00)); // Reserved or disabled
  writeRegister((0x0C), (0x20)); // F2 right connected to ADC1
  writeRegister((0x0D), (0x04)); // F4 right connected to ADC3
  writeRegister((0x0E), (0x00)); // F6/F8 right disabled
  writeRegister((0x0F), (0x30)); // F3 right connected to AD2
  writeRegister((0x10), (0x01)); // F1 right connected to AD0
  writeRegister((0x11), (0x50)); // CLEAR right connected to AD4
  writeRegister((0x12), (0x00)); // Reserved or disabled
  writeRegister((0x13), (0x06)); // NIR connected to ADC5
}

void setup_F5F8_Clear_NIR() {
  // SMUX Config for F5,F6,F7,F8,NIR,Clear
  writeRegister((0x00), (0x00)); // F3 left disable
  writeRegister((0x01), (0x00)); // F1 left disable
  writeRegister((0x02), (0x00)); // reserved/disable
  writeRegister((0x03), (0x40)); // F8 left connected to ADC3
  writeRegister((0x04), (0x02)); // F6 left connected to ADC1
  writeRegister((0x05), (0x00)); // F4/ F2 disabled
  writeRegister((0x06), (0x10)); // F5 left connected to ADC0
  writeRegister((0x07), (0x03)); // F7 left connected to ADC2
  writeRegister((0x08), (0x50)); // CLEAR Connected to ADC4
  writeRegister((0x09), (0x10)); // F5 right connected to ADC0
  writeRegister((0x0A), (0x03)); // F7 right connected to ADC2
  writeRegister((0x0B), (0x00)); // Reserved or disabled
  writeRegister((0x0C), (0x00)); // F2 right disabled
  writeRegister((0x0D), (0x00)); // F4 right disabled
  writeRegister((0x0E), (0x24)); // F8 right connected to ADC2/ F6 right connected to ADC1
  writeRegister((0x0F), (0x00)); // F3 right disabled
  writeRegister((0x10), (0x00)); // F1 right disabled
  writeRegister((0x11), (0x50)); // CLEAR right connected to AD4
  writeRegister((0x12), (0x00)); // Reserved or disabled
  writeRegister((0x13), (0x06)); // NIR connected to ADC5
}

void powerEnable(bool enable_power){
	//POWER enable true
	uint8_t regWrite[] = {AS7341_ENABLE, 0x01}; //PON to 1
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regWrite, sizeof(regWrite), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200) != HAL_OK);
}

as7341_ReturnError_t enableSpectralMeasurement(bool enable_measurement) {
	uint8_t regwrite[2];
	regwrite[0] = AS7341_ENABLE;

	/*
	 * 	   Changing the n_th bit to x
	 *     (number & ~(1UL << n)) will clear the nth bit and (x << n) will set the nth bit to x
	 */
	as7341_enable_reg = (as7341_enable_reg & ~(1UL << 0x01)) | (enable_measurement << 0x01); /* setting as7341_enable_reg bit 0x01 SP_EN to enable_measurement  */
	regwrite[1] = as7341_enable_reg;
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regwrite, sizeof(regwrite), HAL_MAX_DELAY)!=HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);

  return AS7341_ERROR_NO;
}
/* TODO */
bool setHighThreshold(uint16_t high_threshold){
	uint8_t regWrite[]={AS7341_SP_HIGH_TH_L,high_threshold}; //PON to 1
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regWrite, sizeof(regWrite), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
	return 1;
}
/* TODO */
bool setLowThreshold(uint16_t low_threshold){
	uint8_t regWrite[]={AS7341_SP_LOW_TH_L,low_threshold}; //PON to 1
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regWrite, sizeof(regWrite), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
	return 1;
}
/* TODO */
uint16_t getHighThreshold(void){
	uint8_t regtest[]={AS7341_SP_HIGH_TH_L};
	uint8_t regRead[2]={0,0};
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regtest, 1, HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(&as7341.hi2c, 0x72, regRead, sizeof(regRead), HAL_MAX_DELAY)!= HAL_OK);
	return regRead[0];
}
/* TODO */
uint16_t getLowThreshold(void){
	uint8_t regtest[]={AS7341_SP_LOW_TH_L};
	uint8_t regRead[2]={0,0};
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regtest, 1, HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(&as7341.hi2c, 0x72, regRead, sizeof(regRead), HAL_MAX_DELAY)!= HAL_OK);
	return regRead[0];
}
/* TODO */
bool enableSpectralInterrupt(bool enable_int);
bool enableSystemInterrupt(bool enable_int);

bool setAPERS(as7341_int_cycle_count_t cycle_count);
bool setSpectralThresholdChannel(as7341_adc_channel_t channel);

uint8_t getInterruptStatus(void);
bool clearInterruptStatus(void);

bool spectralInterruptTriggered(void);
uint8_t spectralInterruptSource(void);
bool spectralLowTriggered(void);
bool spectralHighTriggered(void);

bool enableLED(bool enable_led);
bool setLEDCurrent(uint16_t led_current_ma);
/* TODO */
void disableAll(void){
	//POWER disable
	uint8_t regwrite[]={AS7341_ENABLE,0x00}; //PON to 1
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regwrite, sizeof(regwrite), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
}

bool getIsDataReady(){
	uint8_t regwrite[2];
	uint8_t regRead[1];

	regwrite[0] = AS7341_STATUS2;
	status = HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regwrite, 1, HAL_MAX_DELAY);
	status = HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200);
	status = HAL_I2C_Master_Receive(&as7341.hi2c, as7341.writing_ID, regRead, sizeof(regRead), HAL_MAX_DELAY);
    /*  This register is self-clearing, meaning that writing a “1” to any bit in the
	 *  register clears that status bit. In this way, the user should read the STATUS register, handle all
	 *  indicated event(s) and then write the register value back to STATUS to clear the handled events.
	 *  Writing “0” will not clear those bits if they have a value of “1”, which means that new events that
	 *  occurred since the last read of the STATUS register will not be accidentally cleared.
	 */
	if(regRead[0]>>6){
    	regwrite[1] = regRead[0];
    	status = HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regwrite, sizeof(regwrite), HAL_MAX_DELAY);
    }

    return (regRead[0]>>6);
}
bool setBank(bool low); // low true gives access to 0x60 to 0x74

as7341_gpio_dir_t getGPIODirection(void);
bool setGPIODirection(as7341_gpio_dir_t gpio_direction);
bool getGPIOInverted(void);
bool setGPIOInverted(bool gpio_inverted);
bool getGPIOValue(void);
bool setGPIOValue(bool);

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
/*
bool AS7341init(int32_t sensor_id){
    //POWER enable true
    uint8_t regWrite[]={AS7341_ENABLE,0x01}; //PON to 1
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regWrite, sizeof(regWrite), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
	return 1;
}*/

bool enableSMUX() {
	//uint8_t regwrite[1];
	uint8_t regRead[1];

	/*
	 * 	   Changing the n_th bit to x
	 *     (number & ~(1UL << n)) will clear the nth bit and (x << n) will set the nth bit to x
	 */
	as7341_enable_reg = (as7341_enable_reg & ~(1UL << 0x04)) | (0x01 << 0x04); /* setting as7341_enable_reg bit 0x04 SMUXEN to 0x01 (activate)  */
	status = writeRegister(AS7341_ENABLE, as7341_enable_reg);

	//regwrite[0] = AS7341_ENABLE;
	do{
		//status = HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regwrite, 1, HAL_MAX_DELAY);
		//status = HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200);
		/*
		 *
		 * To read a register, it must be selected with an I2C write operation by sending the appropriate register pointer
		 * (Note that if you have written this register right before the read then you do not have to send again its address to the pointer register,
		 * as you have already set it during write). Then with an I2C read operation.
		 *
		 * */
		status = HAL_I2C_Master_Receive(&as7341.hi2c, 0x72, regRead, sizeof(regRead), HAL_MAX_DELAY);
	}while((regRead[0]>>4)!=0x01); /* TODO add timeout */

    return status;
}

bool enableFlickerDetection(bool enable_fd){
	uint8_t regWrite[]={AS7341_ENABLE,0x40}; //PON to 1
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regWrite, sizeof(regWrite), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
	return 1;
}

void FDConfig(void){
	  // SMUX Config for Flicker- register (0x13)left set to ADC6 for flicker
	  // detection
	  writeRegister((0x00), (0x00)); // disabled
	  writeRegister((0x01), (0x00)); // disabled
	  writeRegister((0x02), (0x00)); // reserved/disabled
	  writeRegister((0x03), (0x00)); // disabled
	  writeRegister((0x04), (0x00)); // disabled
	  writeRegister((0x05), (0x00)); // disabled
	  writeRegister((0x06), (0x00)); // disabled
	  writeRegister((0x07), (0x00)); // disabled
	  writeRegister((0x08), (0x00)); // disabled
	  writeRegister((0x09), (0x00)); // disabled
	  writeRegister((0x0A), (0x00)); // disabled
	  writeRegister((0x0B), (0x00)); // Reserved or disabled
	  writeRegister((0x0C), (0x00)); // disabled
	  writeRegister((0x0D), (0x00)); // disabled
	  writeRegister((0x0E), (0x00)); // disabled
	  writeRegister((0x0F), (0x00)); // disabled
	  writeRegister((0x10), (0x00)); // disabled
	  writeRegister((0x11), (0x00)); // disabled
	  writeRegister((0x12), (0x00)); // Reserved or disabled
	  writeRegister((0x13), (0x60)); // Flicker connected to ADC5 to left of 0x13
}

// maybe return a typedef enum
/**
 * @brief Returns the flicker detection status
 *
 * @return int8_t
 */
int8_t getFlickerDetectStatus(void){
	uint8_t read[1];
	uint8_t regCh[] = {AS7341_FD_STATUS};

	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regCh, 2, HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
	while(HAL_I2C_Master_Receive(&as7341.hi2c, 0x72, read, sizeof(read), HAL_MAX_DELAY)!= HAL_OK);

	return read[0];
}

bool setSMUXCommand(as7341_smux_cmd_t command) {
	uint8_t regwrite[2];
	status = 0; /* TODO check if needs to be set to 0 */

	regwrite[0] = AS7341_CFG6;
	regwrite[1] = command<<3;
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, regwrite, sizeof(regwrite), HAL_MAX_DELAY)!=HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);

	return AS7341_ERROR_NO;
}

as7341_ReturnError_t writeRegister(uint8_t addr, uint8_t val) {
	uint8_t data[] = {addr, val};
	while(HAL_I2C_Master_Transmit(&as7341.hi2c, 0x72, data, sizeof(data), HAL_MAX_DELAY) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&as7341.hi2c,0x72,10,200)!=HAL_OK);
	return AS7341_ERROR_NO;
}

/*
uint8_t readRegister8(uint8_t addr, uint8_t *readReg, uint8_t size){
	uint8_t data[] = {addr, val};
	status = HAL_I2C_Master_Transmit(&as7341.hi2c, as7341.writing_ID, writeReg, sizeof(writeReg), HAL_MAX_DELAY);
	status = HAL_I2C_IsDeviceReady(&as7341.hi2c,as7341.writing_ID,10,200);
	status = HAL_I2C_Master_Receive(&as7341.hi2c, as7341.writing_ID, readReg, size, HAL_MAX_DELAY);
	return AS7341_ERROR_NO;
}

uint16_t readRegister16(uint8_t *writeReg, uint16_t *readReg, uint8_t size){
	uint8_t data[] = {addr, val};
	status = HAL_I2C_Master_Transmit(&as7341.hi2c, as7341.writing_ID, &as7341.astep.address_L, 1, HAL_MAX_DELAY);
	status = HAL_I2C_IsDeviceReady(&as7341.hi2c,as7341.writing_ID,10,200);
	status = HAL_I2C_Master_Receive(&as7341.hi2c, as7341.writing_ID, &as7341.astep.value, 1, HAL_MAX_DELAY);
	return AS7341_ERROR_NO;
}

*/
as7341_ReturnError_t setSMUXLowChannels(bool f1_f4) {
  as7341_ReturnError_t err = AS7341_ERROR_NO;
  err = enableSpectralMeasurement(0);
  err = setSMUXCommand(AS7341_SMUX_CMD_WRITE);
  if (f1_f4) {
    setup_F1F4_Clear_NIR();
  } else {
    setup_F5F8_Clear_NIR();
  }
  err = enableSMUX();
  return err;
}













