/*
 * INA226.c
 *
 *  Created on: 7 juil. 2021
 *      Author: sajl9
 */


/* Class file for the INA226 Bi-directional Current/Power Monitor Arduino Library.

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


#include <INA226.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <main.h>
#include <math.h>



bool INA226begin(INA226_t *module, I2C_HandleTypeDef hi2c1)
{
	module->hi2c = hi2c1;
	module->sensor_ID = 0x40;
    return INA226configure(
        INA226_AVERAGES_1,
        INA226_BUS_CONV_TIME_1100US,
        INA226_SHUNT_CONV_TIME_1100US,
        INA226_MODE_SHUNT_BUS_CONT
    );
}

bool INA226configure(ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode)
{
    uint16_t config = 0;

    config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);

    vBusMax = 36;
    vShuntMax = 0.08192f;

    return writeRegister16(INA226_REG_CONFIG, config);
}

bool INA226calibrate(float rShuntValue, float iMaxCurrentExcepted)
{
    uint16_t calibrationValue;
    rShunt = rShuntValue;

    float iMaxPossible, minimumLSB;

    iMaxPossible = vShuntMax / rShunt;

    minimumLSB = iMaxCurrentExcepted / 32767;

    currentLSB = (uint32_t)(minimumLSB * 100000000);
    currentLSB /= 100000000;
    currentLSB /= 0.0001;
    currentLSB = ceil(currentLSB);
    currentLSB *= 0.0001;

    powerLSB = currentLSB * 25;

    calibrationValue = (uint16_t)((0.00512) / (currentLSB * rShunt));

    return writeRegister16(INA226_REG_CALIBRATION, calibrationValue);
}

float getMaxPossibleCurrent(void)
{
    return (vShuntMax / rShunt);
}

float getMaxCurrent(void)
{
    float maxCurrent = (currentLSB * 32767);
    float maxPossible = getMaxPossibleCurrent();

    if (maxCurrent > maxPossible)
    {
        return maxPossible;
    } else
    {
        return maxCurrent;
    }
}

float getMaxShuntVoltage(void)
{
    float maxVoltage = getMaxCurrent() * rShunt;

    if (maxVoltage >= vShuntMax)
    {
        return vShuntMax;
    } else
    {
        return maxVoltage;
    }
}

float getMaxPower(void)
{
    return (getMaxCurrent() * vBusMax);
}

float readBusPower(void)
{
    return (readRegister(INA226_REG_POWER) * powerLSB);
}

float readShuntCurrent(void)
{
    return (readRegister(INA226_REG_CURRENT) * currentLSB);
}

float readShuntVoltage(void)
{
    float voltage;

    voltage = readRegister(INA226_REG_SHUNTVOLTAGE);

    return (voltage * 0.0000025);
}

float readBusVoltage(void)
{
    int16_t voltage;

    voltage = readRegister(INA226_REG_BUSVOLTAGE);

    return (voltage * 0.00125);
}

ina226_averages_t getAverages(void)
{
    uint16_t value;

    value = readRegister(INA226_REG_CONFIG);
    value &= 0b0000111000000000;
    value >>= 9;

    return (ina226_averages_t)value;
}

ina226_busConvTime_t getBusConversionTime(void)
{
    uint16_t value;

    value = readRegister(INA226_REG_CONFIG);
    value &= 0b0000000111000000;
    value >>= 6;

    return (ina226_busConvTime_t)value;
}

ina226_shuntConvTime_t getShuntConversionTime(void)
{
    uint16_t value;

    value = readRegister(INA226_REG_CONFIG);
    value &= 0b0000000000111000;
    value >>= 3;

    return (ina226_shuntConvTime_t)value;
}

ina226_mode_t getMode(void)
{
    uint16_t value;

    value = readRegister(INA226_REG_CONFIG);
    value &= 0b0000000000000111;

    return (ina226_mode_t)value;
}

bool setMaskEnable(uint16_t mask)
{
    return writeRegister16(INA226_REG_MASKENABLE, mask);
}

uint16_t getMaskEnable(void)
{
    return readRegister(INA226_REG_MASKENABLE);
}

bool enableShuntOverLimitAlert(void)
{
    return writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SOL);
}

bool enableShuntUnderLimitAlert(void)
{
    return writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SUL);
}

bool enableBusOvertLimitAlert(void)
{
    return writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BOL);
}

bool enableBusUnderLimitAlert(void)
{
    return writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BUL);
}

bool enableOverPowerLimitAlert(void)
{
    return writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_POL);
}

bool enableConversionReadyAlert(void)
{
    return writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_CNVR);
}

bool setBusVoltageLimit(float voltage)
{
    uint16_t value = voltage / 0.00125;
    return writeRegister16(INA226_REG_ALERTLIMIT, value);
}

bool setShuntVoltageLimit(float voltage)
{
    uint16_t value = voltage / 0.0000025;
    return writeRegister16(INA226_REG_ALERTLIMIT, value);
}

bool setPowerLimit(float watts)
{
    uint16_t value = watts / powerLSB;
    return writeRegister16(INA226_REG_ALERTLIMIT, value);
}

bool setAlertInvertedPolarity(bool inverted)
{
    uint16_t temp = getMaskEnable();

    if (inverted)
    {
        temp |= INA226_BIT_APOL;
    } else
    {
        temp &= ~INA226_BIT_APOL;
    }

    return setMaskEnable(temp);
}

bool setAlertLatch(bool latch)
{
    uint16_t temp = getMaskEnable();

    if (latch)
    {
        temp |= INA226_BIT_LEN;
    } else
    {
        temp &= ~INA226_BIT_LEN;
    }

    return setMaskEnable(temp);
}

bool isMathOverflow(void)
{
    return ((getMaskEnable() & INA226_BIT_OVF) == INA226_BIT_OVF);
}

bool isAlert(void)
{
    return ((getMaskEnable() & INA226_BIT_AFF) == INA226_BIT_AFF);
}


uint8_t readRegister(INA226_Register reg)
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

bool writeRegister16(INA226_Register reg, uint16_t data)
{
	uint8_t highByte = data >> 8;
	uint8_t lowByte = data & 0xFF;
    uint8_t initStruct[3]={reg, highByte, lowByte};

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c, INA226_ADDRESS, initStruct, sizeof(initStruct), 1);
	return (status == HAL_OK);
}
