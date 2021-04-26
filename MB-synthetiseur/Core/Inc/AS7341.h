/*
 * AS7341.h
 *
 *  Created on: Mar 30, 2021
 *      Author: boulnat
 */

#ifndef INC_AS7341_H_
#define INC_AS7341_H_
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define AS7341_I2CADDR_DEFAULT 0x39 ///< AS7341 default i2c address
#define AS7341_CHIP_ID 0x09         ///< AS7341 default device id from WHOAMI

#define AS7341_WHOAMI 0x92 ///< Chip ID register

//#define AS7341_ASTATUS 0x60    	///< AS7341_ASTATUS (unused)
//#define AS7341_CH0_DATA_L 0x61 	///< AS7341_CH0_DATA_L (unused)
//#define AS7341_CH0_DATA_H 0x62 	///< AS7341_CH0_DATA_H (unused)
#define AS7341_ITIME_L 0x63   		///< AS7341_ITIME_L (unused)
#define AS7341_ITIME_M 0x64   		///< AS7341_ITIME_M (unused)
#define AS7341_ITIME_H 0x65    		///< AS7341_ITIME_H (unused)
#define AS7341_CONFIG 0x70 			///< Enables LED control and sets light sensing mode
#define AS7341_STAT 0x71   			///< AS7341_STAT (unused)
#define AS7341_EDGE 0x72   			///< AS7341_EDGE (unused)
#define AS7341_GPIO 0x73   			///< Connects photo diode to GPIO or INT pins
#define AS7341_LED 0x74    			///< LED Register; Enables and sets current limit
#define AS7341_ENABLE  0x80 		///< Main enable register. Controls SMUX, Flicker Detection, Spectral
       ///< Measurements and Power
#define AS7341_ATIME 0x81       	///< Sets ADC integration step count
#define AS7341_WTIME 0x83       	///< AS7341_WTIME (unused)
#define AS7341_SP_LOW_TH_L 0x84 	///< Spectral measurement Low Threshold low byte
#define AS7341_SP_LOW_TH_H   0x85 	///< Spectral measurement Low Threshold high byte
#define AS7341_SP_HIGH_TH_L  0x86 	///< Spectral measurement High Threshold low byte
#define AS7341_SP_HIGH_TH_H  0x87                    ///< Spectral measurement High Threshold low byte
#define AS7341_AUXID 0x90 			///< AS7341_AUXID (unused)
#define AS7341_REVID 0x91 			///< AS7341_REVID (unused)
#define AS7341_ID 0x92    			///< AS7341_ID (unused)
#define AS7341_STATUS  0x93 		///< Interrupt status registers. Indicates the occourance of an interrupt
#define AS7341_ASTATUS 0x94    		///< AS7341_ASTATUS (unused)
#define AS7341_CH0_DATA_L 0x95 		///< ADC Channel Data
#define AS7341_CH0_DATA_H 0x96 		///< ADC Channel Data
#define AS7341_CH1_DATA_L 0x97 		///< ADC Channel Data
#define AS7341_CH1_DATA_H 0x98 		///< ADC Channel Data
#define AS7341_CH2_DATA_L 0x99 		///< ADC Channel Data
#define AS7341_CH2_DATA_H 0x9A 		///< ADC Channel Data
#define AS7341_CH3_DATA_L 0x9B 		///< ADC Channel Data
#define AS7341_CH3_DATA_H 0x9C 		///< ADC Channel Data
#define AS7341_CH4_DATA_L 0x9D 		///< ADC Channel Data
#define AS7341_CH4_DATA_H 0x9E 		///< ADC Channel Data
#define AS7341_CH5_DATA_L 0x9F 		///< ADC Channel Data
#define AS7341_CH5_DATA_H 0xA0 		///< ADC Channel Data
#define AS7341_STATUS2 0xA3 		///< Measurement status flags; saturation, validity
#define AS7341_STATUS3 0xA4 		///< Spectral interrupt source, high or low threshold
#define AS7341_STATUS5 0xA6 		///< AS7341_STATUS5 (unused)
#define AS7341_STATUS6 0xA7			///< AS7341_STATUS6 (unused)
#define AS7341_CFG0 0xA9 			///< Sets Low power mode, Register bank, and Trigger lengthening
#define AS7341_CFG1 0xAA 			///< Controls ADC Gain
#define AS7341_CFG3 0xAC 			///< AS7341_CFG3 (unused)
#define AS7341_CFG6 0xAF 			///< Used to configure Smux
#define AS7341_CFG8 0xB1 			///< AS7341_CFG8 (unused)
#define AS7341_CFG9 0xB2 			///< Enables flicker detection and smux command completion system
       ///< interrupts
#define AS7341_CFG10 0xB3 ///< AS7341_CFG10 (unused)
#define AS7341_CFG12 0xB5 ///< Spectral threshold channel for interrupts, persistence and auto-gain
#define AS7341_PERS  0xBD ///< Number of measurement cycles outside thresholds to trigger an
       ///< interupt
#define AS7341_GPIO2 0xBE ///< GPIO Settings and status: polarity, direction, sets output, reads
       ///< input
#define AS7341_ASTEP_L 0xCA      ///< Integration step size ow byte
#define AS7341_ASTEP_H 0xCB      ///< Integration step size high byte
#define AS7341_AGC_GAIN_MAX 0xCF ///< AS7341_AGC_GAIN_MAX (unused)
#define AS7341_AZ_CONFIG 0xD6    ///< AS7341_AZ_CONFIG (unused)
#define AS7341_FD_TIME1 0xD8 ///< Flicker detection integration time low byte
#define AS7341_FD_TIME2 0xDA ///< Flicker detection gain and high nibble
#define AS7341_FD_CFG0 0xD7  ///< AS7341_FD_CFG0 (unused)
#define AS7341_FD_STATUS 0xDB ///< Flicker detection status; measurement valid, saturation, flicker
       ///< type
#define AS7341_INTENAB 0xF9  ///< Enables individual interrupt types
#define AS7341_CONTROL 0xFA  ///< Auto-zero, fifo clear, clear SAI active
#define AS7341_FIFO_MAP 0xFC ///< AS7341_FIFO_MAP (unused)
#define AS7341_FIFO_LVL 0xFD ///< AS7341_FIFO_LVL (unused)
#define AS7341_FDATA_L 0xFE  ///< AS7341_FDATA_L (unused)
#define AS7341_FDATA_H 0xFF  ///< AS7341_FDATA_H (unused)

#define AS7341_SPECTRAL_INT_HIGH_MSK 0b00100000 ///< bitmask to check for a high threshold interrupt
#define AS7341_SPECTRAL_INT_LOW_MSK  0b00010000 ///< bitmask to check for a low threshold interrupt

/**
 * @brief Allowable gain multipliers for `setGain`
 *
 */
typedef enum {
  AS7341_GAIN_0_5X,
  AS7341_GAIN_1X,
  AS7341_GAIN_2X,
  AS7341_GAIN_4X,
  AS7341_GAIN_8X,
  AS7341_GAIN_16X,
  AS7341_GAIN_32X,
  AS7341_GAIN_64X,
  AS7341_GAIN_128X,
  AS7341_GAIN_256X,
  AS7341_GAIN_512X,
} as7341_gain_t;

/**
 * @brief Available SMUX configuration commands
 *
 */
typedef enum {
  AS7341_SMUX_CMD_ROM_RESET, ///< ROM code initialization of SMUX
  AS7341_SMUX_CMD_READ,      ///< Read SMUX configuration to RAM from SMUX chain
  AS7341_SMUX_CMD_WRITE, ///< Write SMUX configuration from RAM to SMUX chain
} as7341_smux_cmd_t;
/**
 * @brief ADC Channel specifiers for configuration
 *
 */
typedef enum {
  AS7341_ADC_CHANNEL_0,
  AS7341_ADC_CHANNEL_1,
  AS7341_ADC_CHANNEL_2,
  AS7341_ADC_CHANNEL_3,
  AS7341_ADC_CHANNEL_4,
  AS7341_ADC_CHANNEL_5,
} as7341_adc_channel_t;
/**
 * @brief Spectral Channel specifiers for configuration and reading
 *
 */
typedef enum {
  AS7341_CHANNEL_415nm_F1,
  AS7341_CHANNEL_445nm_F2,
  AS7341_CHANNEL_480nm_F3,
  AS7341_CHANNEL_515nm_F4,
  AS7341_CHANNEL_555nm_F5,
  AS7341_CHANNEL_590nm_F6,
  AS7341_CHANNEL_630nm_F7,
  AS7341_CHANNEL_680nm_F8,
  AS7341_CHANNEL_CLEAR,
  AS7341_CHANNEL_NIR,
} as7341_color_channel_t;

/**
 * @brief The number of measurement cycles with spectral data outside of a
 * threshold required to trigger an interrupt
 *
 */
typedef enum {
  AS7341_INT_COUNT_ALL, ///< 0
  AS7341_INT_COUNT_1,   ///< 1
  AS7341_INT_COUNT_2,   ///< 2
  AS7341_INT_COUNT_3,   ///< 3
  AS7341_INT_COUNT_5,   ///< 4
  AS7341_INT_COUNT_10,  ///< 5
  AS7341_INT_COUNT_15,  ///< 6
  AS7341_INT_COUNT_20,  ///< 7
  AS7341_INT_COUNT_25,  ///< 8
  AS7341_INT_COUNT_30,  ///< 9
  AS7341_INT_COUNT_35,  ///< 10
  AS7341_INT_COUNT_40,  ///< 11
  AS7341_INT_COUNT_45,  ///< 12
  AS7341_INT_COUNT_50,  ///< 13
  AS7341_INT_COUNT_55,  ///< 14
  AS7341_INT_COUNT_60,  ///< 15
} as7341_int_cycle_count_t;

/**
 * @brief Pin directions to set how the GPIO pin is to be used
 *
 */
typedef enum {
  AS7341_GPIO_OUTPUT, ///< THhe GPIO pin is configured as an open drain output
  AS7341_GPIO_INPUT,  ///< The GPIO Pin is set as a high-impedence input
} as7341_gpio_dir_t;

/**
 * @brief Wait states for async reading
 */
typedef enum {
  AS7341_WAITING_START, //
  AS7341_WAITING_LOW,   //
  AS7341_WAITING_HIGH,  //
  AS7341_WAITING_DONE,  //
} as7341_waiting_t;

/**
 * Object for one entry with specific index in @ref CO_SDO_objectDictionary.
 */
typedef struct {
	//I2C definition
	I2C_HandleTypeDef 	hi2c;

	//sensor ID by default 0x80;
	uint8_t				sensor_ID;

	//for integration
	//number of step
	uint8_t 			astep;
	//time
	uint8_t 			atime;
	//gain of integration
	as7341_gain_t 		gain;

	//pointer to buffer
	void				*preadings_buffer;
}as7341_t;

  /*!
   *    @brief  Sets up the hardware and initializes I2C
   *    @param  hi2c1
   *            I2C handle Structure definition
   *    @return True.
   */
  bool AS7341begin(I2C_HandleTypeDef hi2c1);


  /*!
   *    @brief  Sets up ASTEP Addr: 0xCA, 0xCB
   *    @param  astep_value
   *            Sets the integration time per step in increments of
   *			2.78µs. The default value is 999.
   *			Value	Step Size
   *			0 		2.78µs
   *			n 		2.78µs x (n+1)
   *			ex: 599 -> 1.67ms
   *    @return True.
   */
  bool setASTEP(uint8_t astep_value);

  /*!
   *    @brief  Sets up ATIME Addr: 0x81
   *    @param  atime_value
   *            Sets the number of integration steps from 1 to 256
   *            Value	Integration Time
   *            0 		ASTEP
   *            n 		ASTEP x (n+1)
   *            ex: 255 -> 256 x ASTEP
   *    @return True if initialization was successful, otherwise false.
   */
  bool setATIME(uint8_t atime_value);

  /*!
   *    @brief  Spectral engines gain setting. Addr: 0xAA
   *    @param  gain_value
   *            Sets the spectral sensitivity.
   *            Value	Gain
   *            0 		0.5x
   *            1		1x
   *            .		.
   *            10 		512x
   *    @return True if initialization was successful, otherwise false.
   */
  bool setGain(as7341_gain_t gain_value);

  /*!
   *    @brief  get integration time per step.
   *    @return Astep value.
   */
  uint16_t getASTEP();
  /*!
   *    @brief  Sets the number of integration steps
   *    @return Astep value.
   */
  uint8_t getATIME();
  /*!
   *    @brief  get Spectral engines gain setting.
   *    @return gain.
   */
  as7341_gain_t getGain();

  /**
   * @brief Returns the integration time
   *
   * The integration time is `(ATIME + 1) * (ASTEP + 1) * 2.78µS`
   *
   * @return long The current integration time in ms
   */
  long getTINT();

  /**
   * @brief Converts raw ADC values to basic counts
   *
   * The basic counts are `RAW/(GAIN * TINT)`
   *
   * @param raw The raw ADC values to convert
   *
   * @return float The basic counts
   */
  float toBasicCounts(uint16_t raw);

  /**
   * @brief fills the provided buffer with the current measurements for Spectral
   * channels F1-8, Clear and NIR
   *
   * @param readings_buffer Pointer to a buffer of length 10 or more to fill with
   * sensor data
   * @return true: success false: failure
   */
  uint16_t readAllChannels(uint16_t *readings_buffer);

  /**
   * @brief Delay while waiting for data, with option to time out and recover
   *
   * @param waitTime the maximum amount of time to wait
   * @return none
   */
  void delayForData(int waitTime);

  /**
   * @brief Returns the ADC data for a given channel
   *
   * @param channel The ADC channel to read
   * @return uint16_t The measured data for the currently configured sensor
   */
  uint16_t readChannel(as7341_adc_channel_t channel);

  /**
   * @brief Returns the reading data for the specified color channel
   *
   *  call `readAllChannels` before reading to update the stored readings
   *
   * @param channel The color sensor channel to read
   * @return uint16_t The measured data for the selected sensor channel
   */
  uint16_t getChannel(as7341_color_channel_t channel);

  /**
   * @brief starts the process of getting readings from all channels without using
   * delays
   *
   * @return true: success false: failure (a bit arbitrary)
   */
  bool startReading(void);

  /**
   * @brief runs the process of getting readings from all channels without using
   * delays.  Should be called regularly (ie. in loop()) Need to call
   * startReading() to initialise the process Need to call getAllChannels() to
   * transfer the data into an external buffer
   *
   * @return true: reading is complete false: reading is incomplete (or failed)
   */
  bool checkReadingProgress();

  /**
   * @brief transfer all the values from the private result buffer into one
   * nominated
   *
   * @param readings_buffer Pointer to a buffer of length 12 (THERE IS NO ERROR
   * CHECKING, YE BE WARNED!)
   *
   * @return true: success false: failure
   */
  bool getAllChannels(uint16_t *readings_buffer);

  /**
   * @brief Detect a flickering light
   * @return The frequency of a detected flicker or 1 if a flicker of
   * unknown frequency is detected
   */
  uint16_t detectFlickerHz(void);

  /**
   * @brief Configure SMUX for sensors F1-4, Clear and NIR
   *
   */
  void setup_F1F4_Clear_NIR();

  /**
   * @brief Configure SMUX for sensors F5-8, Clear and NIR
   *
   */
  void setup_F5F8_Clear_NIR();

  /**
   * @brief Sets the power state of the sensor
   *
   * @param enable_power true: on false: off
   */
  void powerEnable(bool enable_power);

  /**
   * @brief Enables measurement of spectral data
   *
   * @param enable_measurement true: enabled false: disabled
   * @return true: success false: failure
   */
  bool enableSpectralMeasurement(bool enable_measurement);

  /**
   * @brief Sets the threshold above which spectral measurements will trigger
   * interrupts when the APERS count is reached
   *
   * @param high_threshold
   * @return true: success false: failure
   */
  bool setHighThreshold(uint16_t high_threshold);

  /**
   * @brief Sets the threshold below which spectral measurements will trigger
   * interrupts when the APERS count is reached
   *
   * @param low_threshold the new threshold
   * @return true: success false: failure
   */
  bool setLowThreshold(uint16_t low_threshold);

  /**
   * @brief Returns the current high thighreshold for spectral measurements
   *
   * @return int16_t The current high threshold
   */
  uint16_t getHighThreshold(void);

  /**
   * @brief Returns the current low thighreshold for spectral measurements
   *
   * @return int16_t The current low threshold
   */
  uint16_t getLowThreshold(void);

  /**
   * @brief Enable Interrupts based on spectral measurements
   *
   * @param enable_int true: enable false: disable
   * @return true: success false: falure
   */
  bool enableSpectralInterrupt(bool enable_int);

  /**
   * @brief Enabled system interrupts
   *
   * @param enable_int Set to true to enable system interrupts
   * @return true: success false: failure
   */
  bool enableSystemInterrupt(bool enable_int);

  /**
   * @brief Sets the number of times an interrupt threshold must be exceeded
   * before an interrupt is triggered
   *
   * @param cycle_count The number of cycles to trigger an interrupt
   * @return true: success false: failure
   */
  bool setAPERS(as7341_int_cycle_count_t cycle_count);

  /**
   * @brief Set the ADC channel to use for spectral thresholds including
   * interrupts, automatic gain control, and persistance settings
   *
   * @param channel The channel to use for spectral thresholds. Must be a
   * as7341_adc_channel_t **except for** `AS7341_ADC_CHANNEL_5`
   * @return true: success false: failure
   */
  bool setSpectralThresholdChannel(as7341_adc_channel_t channel);

  /**
   * @brief Returns the current value of the Interupt status register
   *
   * @return uint8_t
   */
  uint8_t getInterruptStatus(void);

  /**
   * @brief Clear the interrupt status register
   *
   * @return true: success false: failure
   */
  bool clearInterruptStatus(void);

  /**
   * @brief Returns the status of the spectral measurement threshold interrupts
   *
   * @return true: interrupt triggered false: interrupt not triggered
   */
  bool spectralInterruptTriggered(void);

  /**
   * @brief The current state of the spectral measurement interrupt status
   * register
   *
   * @return uint8_t The current status register
   */
  uint8_t spectralInterruptSource(void);

  /**
   * @brief The status of the low threshold interrupt
   *
   * @return true: low interrupt triggered false: interrupt not triggered
   */
  bool spectralLowTriggered(void);

  /**
   * @brief The status of the high threshold interrupt
   *
   * @return true: high interrupt triggered false: interrupt not triggered
   */
  bool spectralHighTriggered(void);


  bool enableLED(bool enable_led);
  bool setLEDCurrent(uint16_t led_current_ma);

  void disableAll(void);

  bool getIsDataReady();
  bool setBank(bool low); // low true gives access to 0x60 to 0x74

  as7341_gpio_dir_t getGPIODirection(void);
  bool setGPIODirection(as7341_gpio_dir_t gpio_direction);
  bool getGPIOInverted(void);
  bool setGPIOInverted(bool gpio_inverted);
  bool getGPIOValue(void);
  bool setGPIOValue(bool);


  bool AS7341init(int32_t sensor_id);
  //uint8_t last_spectral_int_source = 0; ///< The value of the last reading of the spectral interrupt source
         ///< register



  bool enableSMUX();
  bool enableFlickerDetection(bool enable_fd);
  void FDConfig(void);
  int8_t getFlickerDetectStatus(void);
  bool setSMUXCommand(as7341_smux_cmd_t command);
  void writeRegister(uint8_t addr, uint8_t val);
  void setSMUXLowChannels(bool f1_f4);
  uint16_t _channel_readings[12];
  as7341_waiting_t _readingState;



#endif /* INC_AS7341_H_ */
