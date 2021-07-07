/*
INA226.h - Header file for the Bi-directional Current/Power Monitor Arduino Library.
(c) 2014 Korneliusz Jarzebski, modified 2020 by Peter Buchegger
www.jarzebski.pl
This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef INC_INA226_H_
#define INC_INA226_H_
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"



#define INA226_ADDRESS              (0x40)


#define INA226_BIT_SOL              (0x8000)
#define INA226_BIT_SUL              (0x4000)
#define INA226_BIT_BOL              (0x2000)
#define INA226_BIT_BUL              (0x1000)
#define INA226_BIT_POL              (0x0800)
#define INA226_BIT_CNVR             (0x0400)
#define INA226_BIT_AFF              (0x0010)
#define INA226_BIT_CVRF             (0x0008)
#define INA226_BIT_OVF              (0x0004)
#define INA226_BIT_APOL             (0x0002)
#define INA226_BIT_LEN              (0x0001)

typedef enum {
	INA226_REG_CONFIG = 0x00,
	INA226_REG_SHUNTVOLTAGE = 0x01,
	INA226_REG_BUSVOLTAGE = 0x02,
	INA226_REG_POWER = 0x03,
	INA226_REG_CURRENT= 0x04,
	INA226_REG_CALIBRATION= 0x05,
	INA226_REG_MASKENABLE= 0x06,
	INA226_REG_ALERTLIMIT = 0x07,
}INA226_Register;

typedef struct {
	//I2C definition
I2C_HandleTypeDef 	hi2c;

	//sensor ID by default 0x80;
	uint8_t				sensor_ID;
}INA226_t;

typedef enum
{
    INA226_AVERAGES_1             = 0b000,
    INA226_AVERAGES_4             = 0b001,
    INA226_AVERAGES_16            = 0b010,
    INA226_AVERAGES_64            = 0b011,
    INA226_AVERAGES_128           = 0b100,
    INA226_AVERAGES_256           = 0b101,
    INA226_AVERAGES_512           = 0b110,
    INA226_AVERAGES_1024          = 0b111
} ina226_averages_t;

typedef enum
{
    INA226_BUS_CONV_TIME_140US    = 0b000,
    INA226_BUS_CONV_TIME_204US    = 0b001,
    INA226_BUS_CONV_TIME_332US    = 0b010,
    INA226_BUS_CONV_TIME_588US    = 0b011,
    INA226_BUS_CONV_TIME_1100US   = 0b100,
    INA226_BUS_CONV_TIME_2116US   = 0b101,
    INA226_BUS_CONV_TIME_4156US   = 0b110,
    INA226_BUS_CONV_TIME_8244US   = 0b111
} ina226_busConvTime_t;

typedef enum
{
    INA226_SHUNT_CONV_TIME_140US   = 0b000,
    INA226_SHUNT_CONV_TIME_204US   = 0b001,
    INA226_SHUNT_CONV_TIME_332US   = 0b010,
    INA226_SHUNT_CONV_TIME_588US   = 0b011,
    INA226_SHUNT_CONV_TIME_1100US  = 0b100,
    INA226_SHUNT_CONV_TIME_2116US  = 0b101,
    INA226_SHUNT_CONV_TIME_4156US  = 0b110,
    INA226_SHUNT_CONV_TIME_8244US  = 0b111
} ina226_shuntConvTime_t;

typedef enum
{
    INA226_MODE_POWER_DOWN      = 0b000,
    INA226_MODE_SHUNT_TRIG      = 0b001,
    INA226_MODE_BUS_TRIG        = 0b010,
    INA226_MODE_SHUNT_BUS_TRIG  = 0b011,
    INA226_MODE_ADC_OFF         = 0b100,
    INA226_MODE_SHUNT_CONT      = 0b101,
    INA226_MODE_BUS_CONT        = 0b110,
    INA226_MODE_SHUNT_BUS_CONT  = 0b111,
} ina226_mode_t;

//Device status
I2C_HandleTypeDef hi2c;

	bool INA226begin(INA226_t *module, I2C_HandleTypeDef hi2c1);
	bool INA226configure(ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode);
	bool INA226calibrate(float rShuntValue, float iMaxCurrentExcepted);

	ina226_averages_t getAverages(void);
	ina226_busConvTime_t getBusConversionTime(void);
	ina226_shuntConvTime_t getShuntConversionTime(void);
	ina226_mode_t getMode(void);

	bool enableShuntOverLimitAlert(void);
	bool enableShuntUnderLimitAlert(void);
	bool enableBusOvertLimitAlert(void);
	bool enableBusUnderLimitAlert(void);
	bool enableOverPowerLimitAlert(void);
	bool enableConversionReadyAlert(void);

	bool setBusVoltageLimit(float voltage);
	bool setShuntVoltageLimit(float voltage);
	bool setPowerLimit(float watts);

	bool setAlertInvertedPolarity(bool inverted);
	bool setAlertLatch(bool latch);

	bool isMathOverflow(void);
	bool isAlert(void);

	float readShuntCurrent(void);
	float readShuntVoltage(void);
	float readBusPower(void);
	float readBusVoltage(void);

	float getMaxPossibleCurrent(void);
	float getMaxCurrent(void);
	float getMaxShuntVoltage(void);
	float getMaxPower(void);

	int8_t inaAddress;
	float currentLSB, powerLSB;
	float vShuntMax, vBusMax, rShunt;

	bool setMaskEnable(uint16_t mask);
	uint16_t getMaskEnable(void);

	bool writeRegister16(INA226_Register reg, uint16_t val);
	uint8_t readRegister(INA226_Register reg);

#endif
