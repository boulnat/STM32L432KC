/*
 * PCA9685.h
 *
 *  Created on: Apr 1, 2021
 *      Author: boulnat
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_
// REGISTER ADDRESSES
#define PCA9685_ADR	  0x80
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1  0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

/**
 * Return values of some as7341 functions. If function was executed
 * successfully it returns 0 otherwise it returns <0.
 */
typedef enum{
	PCA9685_ERROR_NO	=  0,   /**< Operation completed successfully */
}PCA9685_ReturnError_t;

typedef enum{
	CH1,
	CH2
}CHANNEL;

/**
 * @brief struct of module PCA9685_t
 */
typedef struct {
	//sensor ID by default 0x80;
	uint8_t				channel_ID;
	uint16_t			val_on;
	uint16_t 			val_off;
}CHANNEL_t;

/**
 * @brief struct of module PCA9685_t
 */
typedef struct {
	//I2C definition
	I2C_HandleTypeDef 	hi2c;

	//sensor ID by default 0x80;
	uint8_t				sensor_ID;
	CHANNEL_t			channel;
}PCA9685_t;


/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  hi2c1
 *            I2C handle Structure definition
 *    @param  prescale
 *            prescale value
 *    @return True.
 */
bool PCA9685begin(PCA9685_t *module, I2C_HandleTypeDef hi2c1, uint8_t prescale);

/*!
 *    @brief  read a register of PCA9685
 *    @param  reg
 *            address of the register to read
 *    @return value of the register.
 */
uint8_t PCA9685_read(PCA9685_t module, unsigned char reg);

/*!
 *    @brief  initialise the PCA9685
 *    @param  address
 *            address of the PCA9685
 */
void pca9685_init(PCA9685_t *module);

/*!
 *    @brief  write output of pwm channel value
 *    @param  num
 *            the number of the channel 0-15
 *    @param  on
 *            the start position of the on
 *    @param  off
 *            the end position
 *    		  ex: on:0 -> off:4095 = LED OFF
 */

void pca9685_pwm(PCA9685_t *module, uint8_t num, uint16_t on, uint16_t off);

/*!
 *    @brief  write output of multiple pwm channels value at the same time
 *    @param  num
 *            the number of the channel 0-15
 *    @param  on
 *            the start position of the on
 *    @param  off
 *            the end position
 *    		  ex: on:0 -> off:4095 = LED OFF
 */
void pca9685_mult_pwm(PCA9685_t *module, uint16_t num, uint16_t on, uint16_t off);

/*!
 *    @brief  write output of all pwm channels value at the same time
 *    @param  on
 *            the start position of the on
 *    @param  off
 *            the end position
 *    		  ex: on:0 -> off:4095 = LED OFF
 */
HAL_StatusTypeDef pca9685_all_pwm(PCA9685_t *module, uint16_t on, uint16_t off);

/*!
 *    @brief  write output of multiple pwm channels to off
 *    @param  num
 *            the number of the channel 0-15
 */
void all_led_off(PCA9685_t *module);

/*!
 *    @brief  to be implemented
 */
void reset();
/*!
 *    @brief  to be implemented
 */
void sleep();
/*!
 *    @brief  to be implemented
 */
void wakeup();
/*!
 *    @brief  to be implemented
 */
void setExtClk(uint8_t prescale);
/*!
 *    @brief  to be implemented
 */
void setPWMFreq(float freq);
/*!
 *    @brief  to be implemented
 */
void setOutputMode(bool totempole);
/*!
 *    @brief  to be implemented
 */
uint8_t getPWM(uint8_t num);
/*!
 *    @brief  to be implemented
 */
void setPWM(uint8_t num, uint16_t on, uint16_t off);
/*!
 *    @brief  to be implemented
 */
uint8_t readPrescale(void);
/*!
 *    @brief  to be implemented
 */
void writeMicroseconds(uint8_t num, uint16_t Microseconds);

/*!
 *    @brief  to be implemented
 */
void setOscillatorFrequency(uint32_t freq);
/*!
 *    @brief  to be implemented
 */
uint32_t getOscillatorFrequency(void);


#endif /* INC_PCA9685_H_ */
