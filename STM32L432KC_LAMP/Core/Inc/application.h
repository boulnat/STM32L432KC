/*
 * application.h
 *
 *  Created on: Apr 17, 2021
 *      Author: boulnat
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

  /*!
   *    @brief  state of the lamp
   */
typedef enum {
	ERROR_STATE,
	INIT_SENSOR,
	INIT_CAN,
	WAIT,
	CONFIG,
	RUN
}SystemState;

typedef enum {
	ST,
	CHANNEL_1,
	T,
	PWM
}pidRegister_t;

/*!
 *    @brief  main application
 */
uint8_t application(void);

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  hi2c1
 *            I2C handle Structure definition
 *    @return True.
 */
uint8_t initCan(void);

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  hi2c1
 *            I2C handle Structure definition
 *    @return True.
 */
uint8_t initSensor(void);

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  hi2c1
 *            I2C handle Structure definition
 *    @return True.
 */
void errorHandler(void);

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  hi2c1
 *            I2C handle Structure definition
 *    @return True.
 */
uint8_t idle(void);

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  hi2c1
 *            I2C handle Structure definition
 *    @return True.
 */
uint8_t run(void);

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  hi2c1
 *            I2C handle Structure definition
 *    @return True.
 */
uint8_t config(void);

/**
 * @defgroup CO_application Application interface
 * @ingroup CO_CANopen
 * @{
 *
 * Application interface for CANopenNode stack. Function is called
 * from file main_xxx.c (if implemented).
 *
 * ###Main program flow chart
 *
 * @code
               (Program Start)
                      |
                      V
    +------------------------------------+
    |           programStart()           |
    +------------------------------------+
                      |
                      |<-------------------------+
                      |                          |
                      V                          |
             (Initialze CANopen)                 |
                      |                          |
                      V                          |
    +------------------------------------+       |
    |        communicationReset()        |       |
    +------------------------------------+       |
                      |                          |
                      V                          |
         (Enable CAN and interrupts)             |
                      |                          |
                      |<----------------------+  |
                      |                       |  |
                      V                       |  |
    +------------------------------------+    |  |
    |           programAsync()           |    |  |
    +------------------------------------+    |  |
                      |                       |  |
                      V                       |  |
        (Process CANopen asynchronous)        |  |
                      |                       |  |
                      +- infinite loop -------+  |
                      |                          |
                      +- reset communication ----+
                      |
                      V
    +------------------------------------+
    |            programEnd()            |
    +------------------------------------+
                      |
                      V
              (delete CANopen)
                      |
                      V
                (Program end)
   @endcode
 *
 *
 * ###Timer program flow chart
 *
 * @code
        (Timer interrupt 1 millisecond)
                      |
                      V
              (CANopen read RPDOs)
                      |
                      V
    +------------------------------------+
    |           program1ms()             |
    +------------------------------------+
                      |
                      V
              (CANopen write TPDOs)
   @endcode
 *
 *
 * ###Receive and transmit high priority interrupt flow chart
 *
 * @code
           (CAN receive event or)
      (CAN transmit buffer empty event)
                      |
                      V
       (Process received CAN message or)
   (copy next message to CAN transmit buffer)
   @endcode
 */


/**
 * Called after microcontroller reset.
 */
void programStart(void);


/**
 * Called after communication reset.
 */
void communicationReset(void);


/**
 * Called before program end.
 */
void programEnd(void);


/**
 * Called cyclically from main.
 *
 * @param timer1msDiff Time difference since last call
 */
void programAsync(uint16_t timer1msDiff);


/**
 * Called cyclically from 1ms timer task.
 */
void program1ms(void);

void calibration(void);
void scenario(void);
void spectro(void);
void temperature(void);
void multimetre(void);
void ventilator(uint32_t fan);

uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax);

#endif /* INC_APPLICATION_H_ */
