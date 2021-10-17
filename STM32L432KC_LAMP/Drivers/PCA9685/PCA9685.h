/*
 * PCA9685.h
 *
 *  Created on: Apr 1, 2021
 *      Author: boulnat
 *
 *  PCA9685 Version 2.0 based on #https://github.com/NachtRaveVL/PCA9685-Arduino/blob/master/src/PCA9685.h
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_
// REGISTER ADDRESSES
#define PCA9685_I2C_BASE_MODULE_ADDRESS 0x40
#define PCA9685_I2C_BASE_MODULE_ADRMASK 0x3F
#define PCA9685_I2C_BASE_PROXY_ADDRESS  0xE0
#define PCA9685_I2C_BASE_PROXY_ADRMASK  0xFE

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

// Default proxy addresser i2c addresses
#define PCA9685_I2C_DEF_ALLCALL_PROXYADR    0xE0      // V2.0 Default AllCall i2c proxy address
#define PCA9685_I2C_DEF_SUB1_PROXYADR       0xE2      // V2.0 Default Sub1 i2c proxy address
#define PCA9685_I2C_DEF_SUB2_PROXYADR       0xE4      // V2.0 Default Sub2 i2c proxy address
#define PCA9685_I2C_DEF_SUB3_PROXYADR       0xE8      // V2.0 Default Sub3 i2c proxy address

/**
 * @brief Output driver control mode (see datasheet Table 12 and Fig 13, 14, and 15 concerning correct
 *	usage of OUTDRV).
 *NOTE: Totem-pole mode should be used when an external N-type or P-type driver is in
 * use, which provides actual sourcing current while open-drain mode doesn't. At max
 * channel capacity, the sink current limit is 25mA@5v per channel while the source
 * current limit, in totem-pole mode, is 10mA@5v per channel. However, from datasheet
 * Table 6. subnote [1]: "Some newer LEDs include integrated Zener diodes to limit
 * voltage transients, reduce EMI, and protect the LEDs, and these -MUST- be driven only
 * in the open-drain mode to prevent over-heating the IC." Also from datasheet, Section
 * 10. question 5: "in the push-pull architecture there is a low resistance path to GND
 * through the Zener and this [causes] the IC to overheat."
 */
typedef enum {
	PCA9685_OutputDriverMode_OpenDrain, // Module outputs in an open-drain (aka direct connection) style structure with 400mA @5v total sink current, useful for LEDs and low-power Servos
	PCA9685_OutputDriverMode_TotemPole, // Module outputs in a totem-pole (aka push-pull) style structure with 400mA @5v total sink current and 160mA total source current, useful for external drivers (default)

	PCA9685_OutputDriverMode_Count,             // Internal use only
	PCA9685_OutputDriverMode_Undefined = -1     // Internal use only
} PCA9685_OutputDriverMode;

/**
 * @brief Output-enabled/active-low-OE-pin=LOW driver output mode (see datasheet Table 12 and
 * Fig 13, 14, and 15 concerning correct usage of INVRT).
 *
 * NOTE: Polarity inversion is often set according to if an external N-type driver
 * (should not use INVRT) or external P-type driver/direct connection (should use INVRT)
 * is used. Most breakouts have just a 220Ω resistor between the individual channel
 * outputs of the IC and PWM output pins, which is useful when powering LEDs. The V+ rail
 * of most breakouts can connect through a 10v 1000μF decoupling capacitor, typically
 * already installed on most breakouts, which can reduce voltage spikes and ground bounce
 * during phase shifts at the start/end of the PWM high phase when many channel devices
 * are connected together. See https://forums.adafruit.com/viewtopic.php?f=8&t=127421 and
 * https://forums.adafruit.com/viewtopic.php?f=8&t=162688 for information on installing
 * a decoupling capacitor if need arises.
 */
typedef enum {
	PCA9685_OutputEnabledMode_Normal, // When OE is enabled/LOW, channels output a normal signal, useful for N-type external drivers (default)
	PCA9685_OutputEnabledMode_Inverted, // When OE is enabled/LOW, channels output an inverted signal, useful for P-type external drivers or direct connection

	PCA9685_OutputEnabledMode_Count,            // Internal use only
	PCA9685_OutputEnabledMode_Undefined = -1    // Internal use only
} PCA9685_OutputEnabledMode;

/**
 *  @brief Output-not-enabled/active-low-OE-pin=HIGH driver output mode (see datasheet Section
 *  7.4 concerning correct usage of OUTNE).
 *  NOTE: Active-low-OE pin is typically used to synchronize multiple PCA9685 devices
 *  together, but can also be used as an external dimming control signal.
 */
typedef enum {
	PCA9685_OutputDisabledMode_Low, // When OE is disabled/HIGH, channels output a LOW signal (default)
	PCA9685_OutputDisabledMode_High, // When OE is disabled/HIGH, channels output a HIGH signal (only available in totem-pole mode)
	PCA9685_OutputDisabledMode_Floating, // When OE is disabled/HIGH, channel outputs go into a floating (aka high-impedance/high-Z) state, which may be further refined via external pull-up/pull-down resistors

	PCA9685_OutputDisabledMode_Count,           // Internal use only
	PCA9685_OutputDisabledMode_Undefined = -1   // Internal use only
} PCA9685_OutputDisabledMode;

/**
 * @brief Channel update strategy used when multiple channels are being updated in batch.
 */
typedef enum {
	PCA9685_ChannelUpdateMode_AfterStop, // Channel updates commit after full-transmission STOP signal (default)
	PCA9685_ChannelUpdateMode_AfterAck, // Channel updates commit after individual channel update ACK signal

	PCA9685_ChannelUpdateMode_Count,            // Internal use only
	PCA9685_ChannelUpdateMode_Undefined = -1    // Internal use only
} PCA9685_ChannelUpdateMode;

/**
 * @brief Software-based phase balancing scheme.
 * NOTE: Software-based phase balancing attempts to further mitigate ground bounce and
 * voltage spikes during phase shifts at the start/end of the PWM high phase by shifting
 * the leading edge of each successive PWM high phase by some amount. This helps make
 * the current sinks occur over the entire duty cycle range instead of all together at
 * once. Software-based phase balancing can be useful in certain situations, but in
 * practice has been the source of many problems, including the case whereby the PCA9685
 * will skip a cycle between PWM changes when the leading/trailing edge is shifted past a
 * certain point. While we may revisit this idea in the future, for now we're content on
 * leaving None as the default, and limiting the shift that Linear applies.
 */
typedef enum {
	PCA9685_PhaseBalancer_None, // Disables software-based phase balancing, relying on installed hardware to handle current sinkage (default)
	PCA9685_PhaseBalancer_Linear, // Uses linear software-based phase balancing, with each channel being a preset 16 steps (out of the 4096/12-bit value range) away from previous channel (may cause LED flickering/skipped-cycles on PWM changes)

	PCA9685_PhaseBalancer_Count,                // Internal use only
	PCA9685_PhaseBalancer_Undefined = -1        // Internal use only
} PCA9685_PhaseBalancer;

/**
 * Return values of some as7341 functions. If function was executed
 * successfully it returns 0 otherwise it returns <0.
 */
typedef enum {
	PCA9685_ERROR_NO = 0, /**< Operation completed successfully */
} PCA9685_ReturnError_t;

typedef enum {
	CH1, CH2
} CHANNEL;

/**
 * @brief struct of module PCA9685_t
 */
typedef struct {
	//sensor ID by default 0x80;
	uint8_t channel_ID;
	uint16_t val_on;
	uint16_t val_off;
} CHANNEL_t;

/**
 * @brief struct of module PCA9685_t
 */
typedef struct {
	//I2C definition
	I2C_HandleTypeDef hi2c;

	//sensor ID by default 0x80;
	uint8_t sensor_ID;
	CHANNEL_t channel;
} PCA9685_t;

typedef enum {
	PCA9685_CHANNEL_0,
	PCA9685_CHANNEL_1,
	PCA9685_CHANNEL_2,
	PCA9685_CHANNEL_3,
	PCA9685_CHANNEL_4,
	PCA9685_CHANNEL_5,
	PCA9685_CHANNEL_6,
	PCA9685_CHANNEL_7,
	PCA9685_CHANNEL_8,
	PCA9685_CHANNEL_9,
	PCA9685_CHANNEL_10,
	PCA9685_CHANNEL_11,
	PCA9685_CHANNEL_12,
	PCA9685_CHANNEL_13,
	PCA9685_CHANNEL_14,
	PCA9685_CHANNEL_15,
} PCA9685_color_channel_t;
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
 *  @brief Initializes module. Typically called in setup().
 *  See individual enums for more info.
 */
void init(PCA9685_OutputDriverMode driverMode,
		PCA9685_OutputEnabledMode enabledMode,
		PCA9685_OutputDisabledMode disabledMode,
		PCA9685_ChannelUpdateMode updateMode,
		PCA9685_PhaseBalancer phaseBalancer);
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
void pca9685_mult_pwm(PCA9685_t *module, uint16_t num, uint16_t on,
		uint16_t off);

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
 *    @brief Resets modules. Typically called in setup(), before any init()'s. Calling will
 *     perform a software reset on all PCA9685 devices on the Wire instance, ensuring
 *    that all PCA9685 devices on that line are properly reset.
 */
void pca9685_resetDevices();

/*! Initializes module as a proxy addresser. Typically called in setup(). Used when
 * instance talks through to AllCall/Sub1-Sub3 instances as a proxy object. Using
 * this method will disable any method that performs a read or conflicts with certain
 * states. Proxy addresser i2c addresses must be >= 0xE0, with defaults provided via
 * PCA9685_I2C_DEF_[ALLCALL|SUB[1-3]]_PROXYADR defines.
 */
void pca9685_initAsProxyAddresser();

// Mode accessors
/*!
 *    @brief  get I2C address
 */
uint8_t pca9685_getI2CAddress();
/*!
 *    @brief  get I2C speed
 */
uint32_t pca9685_getI2CSpeed();
/*!
 *    @brief  get output driver mode
 *    @return PCA9685_OutputDriverMode
 */
PCA9685_OutputDriverMode pca9685_getOutputDriverMode();
/*!
 *    @brief  get output enabled mode
 *    @return PCA9685_OutputEnabledMode
 */
PCA9685_OutputEnabledMode pca9685_getOutputEnabledMode();
/*!
 *    @brief  get output disable mode
 *    @return PCA9685_OutputDisabledMode
 */
PCA9685_OutputDisabledMode pca9685_getOutputDisabledMode();
/*!
 *    @brief  get channel update mode
 *    @return PCA9685_ChannelUpdateMode
 */
PCA9685_ChannelUpdateMode pca9685_getChannelUpdateMode();
/*!
 *    @brief  get phase balancer
 *    @return PCA9685_PhaseBalancer
 */
PCA9685_PhaseBalancer pca9685_getPhaseBalancer();

/*!
 *    @brief  Min: 24Hz, Max: 1526Hz, Default: 200Hz. As Hz increases channel resolution
 * diminishes, as raw pre-scaler value, computed per datasheet, starts to require
 * much larger frequency increases for single-digit increases of the raw pre-scaler
 * value that ultimately controls the PWM frequency produced.
 */
void pca9685_setPWMFrequency(float pwmFrequency);

/*!
 * @brief set channel full on
 * @param channel number
 */
void pca9685_setChannelOn(int channel);
/*!
 * @brief set channel full off
 * @param channel number
 */
void pca9685_setChannelOff(int channel);
/*!
 * @brief PWM amounts 0 - 4096, 0 full off, 4096 full on
 * @param   channel: number of the channel to set pwm
 * 			pwmAmout: pwm value
 */
void pca9685_setChannelPWM(int channel, uint16_t pwmAmount);

/*!
 *    @brief  to be implemented
 *    Allows external clock line to be utilized (power reset required to disable)
 */
void enableExtClockLine();
/*!
 *    @brief  to be implemented
 */
void setChannelsPWM(int begChannel, int numChannels, const uint16_t *pwmAmounts);
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

PCA9685_OutputDriverMode _driverMode;                   // Output driver mode
PCA9685_OutputEnabledMode _enabledMode;                // OE enabled output mode
PCA9685_OutputDisabledMode _disabledMode;             // OE disabled output mode
PCA9685_ChannelUpdateMode _updateMode;                  // Channel update mode
PCA9685_PhaseBalancer _phaseBalancer;                   // Phase balancer scheme
bool _isProxyAddresser; // Proxy addresser flag (disables certain functionality)

void writeChannelBegin(int channel);
void writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd);
void writeChannelEnd();

//void writeRegister(uint8_t regAddress, uint8_t value);
//byte readRegister(byte regAddress);

#endif /* INC_PCA9685_H_ */
