/*
 * MCP9600.h
 *
 *  Created on: Mar 29, 2021
 *      Author: boulnat
 */

#ifndef INC_MCP9600_H_
#define INC_MCP9600_H_
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define DEV_ADDR 0x60 //device address of the MCP9600
#define DEV_ID_UPPER 0x40 //value of the upper half of the device ID register. lower half is used for device revision
#define DEV_RESOLUTION 0.0625 //device resolution (temperature in C that the LSB represents)
#define retryAttempts 3 //how many times to attempt to read a register from the thermocouple before giving up

typedef unsigned char BYTE;
// register pointers for various device functions
typedef enum {
  HOT_JUNC_TEMP = 0x00,
  DELTA_JUNC_TEMP = 0x01,
  COLD_JUNC_TEMP = 0x02,
  RAW_ADC = 0x03,
  SENSOR_STATUS = 0x04,
  THERMO_SENSOR_CONFIG = 0x05,
  DEVICE_CONFIG = 0x06,
  ALERT1_CONFIG = 0x08,
  ALERT2_CONFIG = 0x09,
  ALERT3_CONFIG = 0x0A,
  ALERT4_CONFIG = 0x0B,
  ALERT1_HYSTERESIS = 0x0C,
  ALERT2_HYSTERESIS = 0x0D,
  ALERT3_HYSTERESIS = 0x0E,
  ALERT4_HYSTERESIS = 0x0F,
  ALERT1_LIMIT = 0x10,
  ALERT2_LIMIT = 0x11,
  ALERT3_LIMIT = 0x12,
  ALERT4_LIMIT = 0x13,
  DEVICE_ID = 0x20,
}MCP9600_Register ;

typedef enum  {
  TYPE_K = 0b000,
  TYPE_J = 0b001,
  TYPE_T = 0b010,
  TYPE_N = 0b011,
  TYPE_S = 0b100,
  TYPE_E = 0b101,
  TYPE_B = 0b110,
  TYPE_R = 0b111,
}Thermocouple_Type;

typedef enum  {
  RES_ZERO_POINT_0625 = 0,
  RES_ZERO_POINT_25 = 1,
}Ambient_Resolution;

typedef enum  {
  RES_18_BIT = 0b00,
  RES_16_BIT = 0b01,
  RES_14_BIT = 0b10,
  RES_12_BIT = 0b11,
}Thermocouple_Resolution;

typedef enum  {
  SAMPLES_1 = 0b000,
  SAMPLES_2 = 0b001,
  SAMPLES_4 = 0b010,
  SAMPLES_8 = 0b011,
  SAMPLES_16 = 0b100,
  SAMPLES_32 = 0b101,
  SAMPLES_64 = 0b110,
  SAMPLES_128 = 0b111,
}Burst_Sample;

typedef enum  {
  NORMAL = 0x00,
  SHUTDOWN = 0x01,
  BURST = 0x02,
}Shutdown_Mode;

/**
 * Return values of some as7341 functions. If function was executed
 * successfully it returns 0 otherwise it returns <0.
 */
typedef enum{
	MCP9600_ERROR_NO	=  0,   /**< Operation completed successfully */
}PCM9600_ReturnError_t;

/**
 * @brief struct of module PCA9685_t
 */
typedef struct {
	//sensor ID by default 0x80;
	uint8_t				channel_ID;
	uint16_t			val_on;
	uint16_t 			val_off;
}TEMP_t;

/**
 * @brief struct of module PCA9685_t
 */
typedef struct {
	//I2C definition
	I2C_HandleTypeDef 	hi2c;

	//sensor ID by default 0x80;
	uint8_t				sensor_ID;
	TEMP_t			channel;
}PCM9600_t;

  //Device status
  I2C_HandleTypeDef hi2c;
  /*!
   *    @brief  Sets up the hardware and initializes I2C
   *    @param  hi2c1
   *            I2C handle Structure definition
   *    @param  prescale
   *            prescale value
   *    @return True.
   */
  bool PCM9600begin(PCM9600_t *module, I2C_HandleTypeDef hi2c1);

  /*!
     *    @brief  //Returns true if the thermocouple (hot) junction temperature has been updated since we last checked. Also referred to as the data ready bit.
     *    @return True.
     */
  bool available();

  /*!
     *    @brief  //Returns true if the thermocouple will acknowledge over I2C, and false otherwise
     *    @return True.
     */
  bool isConnected();

  /*!
     *    @brief  //Returns the contents of the device ID register. The upper 8 bits are constant, but the lower contain revision data.
     *    @return True.
     */
  uint16_t deviceID();

  /*!
     *    @brief  //Returns true if the constant upper 8 bits in the device ID register are what they should be according to the datasheet.
     *    @return True.
     */
  bool checkDeviceID();

  /*!
     *    @brief  //Resets all device parameters to their default values. Returns 1 if there was an error, zero otherwise.
     *    @return True.
     */
  bool resetToDefaults();

  //Sensor measurements
  /*!
     *    @brief  //Returns the thermocouple temperature, and clears the data ready bit. Set units to true for Celcius, or false for freedom units (Fahrenheit)
     *    @return True.
     */
  uint8_t getThermocoupleTemp(PCM9600_t *module, bool units);

  /*!
     *    @brief   //Returns the ambient (IC die) temperature. Set units to true for Celcius, or false for freedom units (Fahrenheit)
     *    @return True.
     */
  uint8_t getAmbientTemp( bool units);

  /*!
     *    @brief  //Returns the difference in temperature between the thermocouple and ambient junctions. Set units to true for Celcius, or false for freedom units (Fahrenheit)

     *    @return True.
     */
  uint8_t getTempDelta(bool units);

  /*!
     *    @brief  //Returns the raw contents of the raw ADC register
     *    @return True.
     */
  signed long getRawADC();

  /*!
     *    @brief  Returns true if the MCP9600's EMF range has been exceeded, and false otherwise.
     *    @return True.
     */
  bool isInputRangeExceeded();

  //Measurement configuration
  /*!
     *    @brief  Changes the resolution on the cold (ambient) junction, for either 0.0625 or 0.25 degree C resolution. Lower resolution reduces conversion time.
     *    @return True.
     */

  bool setAmbientResolution(Ambient_Resolution res);

  /*!
     *    @brief  Returns the resolution on the cold (ambient) junction, for either 0.0625 or 0.25 degree C resolution. Lower resolution reduces conversion time.
     *    @return True.
     */
  Ambient_Resolution getAmbientResolution();

  /*!
     *    @brief  Changes the resolution on the hot (thermocouple) junction, for either 18, 16, 14, or 12-bit resolution. Lower resolution reduces conversion time.
     *    @return True.
     */
  bool setThermocoupleResolution(Thermocouple_Resolution res);

  /*!
     *    @brief  Returns the resolution on the hot (thermocouple) junction, for either 18, 16, 14, or 12-bit resolution. Lower resolution reduces conversion time.
     *    @return True.
     */
  Thermocouple_Resolution getThermocoupleResolution();

  /*!
     *    @brief  Changes the type of thermocouple connected to the MCP9600. Supported types are KJTNSEBR.
     *    @return True.
     */
  uint8_t setThermocoupleType(Thermocouple_Type type);


  /*!
     *    @brief  Returns the type of thermocouple connected to the MCP9600 as found in its configuration register. Supported types are KJTNSEBR.
     *    @return True.
     */
  Thermocouple_Type getThermocoupleType();

  /*!
     *    @brief  Changes the weight of the on-chip exponential moving average filter. Set this to 0 for no filter, 1 for minimum filter, and 7 for maximum filter.
     *    @return True.
     */
  uint8_t setFilterCoefficient(uint8_t coefficient);

  /*!
     *    @brief  Returns the weight of the on-chip exponential moving average filter.
     *    @return True.
     */
  uint8_t getFilterCoefficient();

  /*!
     *    @brief  Changes the amount of samples to take in burst mode. Returns 0 if set sucessfully, 1 otherwise.
     *    @return True.
     */
  bool setBurstSamples(Burst_Sample samples);

  /*!
     *    @brief  Returns the amount of samples to take in burst mode, according to the device's configuration register.
     *    @return True.
     */
  Burst_Sample getBurstSamples();

  /*!
     *    @brief  Returns true if all the burst samples have been taken and the results are ready. Returns false otherwise.
     *    @return True.
     */
  bool burstAvailable();

  /*!
     *    @brief  Initiates a burst on the MCP9600.
     *    @return True.
     */
  bool startBurst();

  /*!
     *    @brief  Changes the shutdown "operating" mode of the MCP9600. Configurable to Normal, Shutdown, and Burst. Returns 0 if properly set, 1 otherwise.
     *    @return True.
     */
  bool setShutdownMode(Shutdown_Mode mode);

  /*!
     *    @brief  Returns the shutdown "operating" mode of the MCP9600. Configurable to Normal, Shutdown, and Burst.
     *    @return True.
     */
  Shutdown_Mode getShutdownMode();


  //Temperature Alerts
  /*!
     *    @brief  Configures the temperature at which to trigger the alert for a given alert number.
     *    @return True.
     */
  bool configAlertTemp(uint8_t number, float temp);

  /*!
     *    @brief  Configures the junction to monitor the temperature of to trigger the alert. Set to zero for the thermocouple (hot) junction, or one for the ambient (cold) junction.
     *    @return True.
     */
  bool configAlertJunction(uint8_t number, bool junction);

  /*!
     *    @brief  Configures the hysteresis to use around the temperature set point, in degrees Celcius.
     *    @return True.
     */
  bool configAlertHysteresis(uint8_t number, uint8_t hysteresis);

  /*!
     *    @brief  Configures whether to trigger the alert on the rising (cold -> hot) or falling (hot -> cold) edge of the temperature change. Set to 1 for rising, 0 for falling.
     *    @return True.
     */
  bool configAlertEdge(uint8_t number, bool edge);

  /*!
     *    @brief  Configures whether the hardware alert pin is active-high or active-low. Set to 1 for active-high, 0 for active-low.
     *    @return True.
     */
  bool configAlertLogicLevel(uint8_t number, bool level);

  /*!
     *    @brief  Configures whether the MCP9600 treats the alert like a comparator or an interrrupt. Set to 1 for interrupt, 0 for comparator. More information is on pg. 34 of the datasheet.
     *    @return True.
     */
  bool configAlertMode(uint8_t number, bool mode);

  /*!
     *    @brief  Configures whether or not the interrupt is enabled or not. Set to 1 to enable, or 0 to disable.
     *    @return True.
     */
  bool configAlertEnable(uint8_t number, bool enable);

  /*!
     *    @brief  Clears the interrupt on the specified alert channel, resetting the value of the pin.
     *    @return True.
     */
  bool clearAlertPin(uint8_t number);

  /*!
     *    @brief  Returns true if the interrupt has been triggered, false otherwise
     *    @return True.
     */
  bool isTempGreaterThanLimit(uint8_t number);


  //debug
  /*!
     *    @brief  Attempts to read a single register, will keep trying for retryAttempts amount of times
     *    @return True.
     */
  uint8_t readSingleRegister(MCP9600_Register reg);

  /*!
     *    @brief  Attempts to read two registers, will keep trying for retryAttempts amount of times
     *    @return True.
     */
  uint16_t readDoubleRegister(PCM9600_t *module, MCP9600_Register reg);

  /*!
     *    @brief  Attempts to write data into a single 8-bit register. Does not check to make sure it was written successfully. Returns 0 if there wasn't an error on I2C transmission, and 1 otherwise.
     *    @return True.
     */
  bool writeSingleRegister(MCP9600_Register reg, uint8_t data);

  /*!
     *    @brief  Attempts to write data into a double (two 8-bit) registers. Does not check to make sure it was written successfully. Returns 0 if there wasn't an error on I2C transmission, and 1 otherwise.
     *    @return True.
     */
  bool writeDoubleRegister(MCP9600_Register reg, uint16_t data);

  //Internal I2C Abstraction
  //private:

  //TwoWire *_i2cPort;                                                //Generic connection to user's chosen I2C port
  uint8_t _deviceAddress;                                           //I2C address of the MCP9600



#endif /* INC_MCP9600_H_ */
