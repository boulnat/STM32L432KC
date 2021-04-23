/*
 * application.c
 *
 *  Created on: Apr 17, 2021
 *      Author: boulnat
 */

#include "CANopen.h"
#include "i2c.h"
#include "AS7341.h"
#include "MCP9600.h"
#include "PCA9685.h"

#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */


volatile uint16_t   CO_timer1ms = 0U;   /* variable increments each millisecond */

/*******************************************************************************/
void programStart(void){
	  CO_NMT_reset_cmd_t reset = CO_RESET_NOT;

	  /* Configure microcontroller. */
	  /* initialize EEPROM */
	  /* increase variable each startup. Variable is stored in EEPROM. */
	  OD_powerOnCounter++;

	  while(reset != CO_RESET_APP){
	  /* CANopen communication reset - initialize CANopen objects *******************/
	          CO_ReturnError_t err;
	          uint16_t timer1msPrevious;

	          /* disable CAN and CAN interrupts */
	  	    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 1, 0);    //added by me
	  	    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);			 //added by me
	  	    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);	 //added by me
	  	    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);			 //added by me


	          /* initialize CANopen */
	          //err = CO_init(0/* CAN module address */, 10/* NodeID */, 20 /* bit rate */);
	          //if(err != CO_ERROR_NO){
	          //    while(1);
	              /* CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err); */
	          //}


	          /* Configure Timer interrupt function for execution every 1 millisecond */

	  	      //AS7341begin(hi2c1);
	  	      //setATIME(100);
	  	      //setASTEP(999);
	  	      //setGain(AS7341_GAIN_256X);

	  	      //PCM9600begin(hi2c1);
	  	      PCA9685begin(hi2c1, 0);
	  	      pca9685_init(0x80);

	          /* Configure CAN transmit and receive interrupt */
	          err = CO_init((uint32_t)&hcan1, 2, 20);

	          if(err != CO_ERROR_NO)
	             {
	            	 //TODO behavior in a case of the stack error. Currently not defined.
	            	 //_Error_Handler(0, 0);
	             }

	          /* start CAN */
	          CO_CANsetNormalMode(CO->CANmodule[0]);


	          reset = CO_RESET_NOT;
	          timer1msPrevious = CO_timer1ms;  //added by me
	          //put the device in preoperational waiting for master to put in operational
	          //cansend can0 000#010(0)
	          //CO->NMT->operatingState = CO_NMT_OPERATIONAL;//added by me
	          //CO_OD_ROM.producerHeartbeatTime = 0x50;//added by me

	          while(reset == CO_RESET_NOT){
	                  	  /* loop for normal program execution ******************************************/
	        	  	  	  	  INCREMENT_1MS(CO_timer1ms);
	                        uint16_t timer1msCopy, timer1msDiff;

	                        timer1msCopy = CO_timer1ms;
	                        timer1msDiff = timer1msCopy - timer1msPrevious;
	                        timer1msPrevious = timer1msCopy;

	                        /* CANopen process */

	                        reset = CO_process(CO, timer1msDiff, NULL);

	                        //INCREMENT_1MS(CO_timer1ms);//added by me
	                        /* Nonblocking application code may go here. */
	                        if(CO->CANmodule[0]->CANnormal)
	                        {
	                             bool_t syncWas;

	                             /* Process Sync and read inputs */
	                             //CO->RPDO[0]->synchronous=1; //added by me
	                             syncWas = CO_process_SYNC_RPDO(CO, 1000);

	                             /* Further I/O or nonblocking application code may go here. */

	                             uint16_t buff[12];
	                             //readAllChannels(buff);

	                             /* Write outputs */
	                             //CO->TPDO[0]->CANtxBuff[0].data[0]=getChannel(AS7341_CHANNEL_415nm_F1); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[1]=getChannel(AS7341_CHANNEL_415nm_F1); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[2]=getChannel(AS7341_CHANNEL_480nm_F3); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[3]=getChannel(AS7341_CHANNEL_515nm_F4); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[4]=getChannel(AS7341_CHANNEL_555nm_F5); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[5]=getChannel(AS7341_CHANNEL_590nm_F6); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[6]=getChannel(AS7341_CHANNEL_630nm_F7); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[7]=getChannel(AS7341_CHANNEL_680nm_F8); //added by me

	                             //cansend can0 602#3B00180510000000 ask for PDO every 10s
	                             //cansend can0 602#4001640100000000
	                             //CO_OD_RAM.readAnalogueInput16Bit[0] = getChannel(AS7341_CHANNEL_415nm_F1); //added by me set the value of an object
	                             //CO_OD_RAM.readAnalogueInput16Bit[1] = getChannel(AS7341_CHANNEL_445nm_F2);
	                             //CO_OD_RAM.readAnalogueInput16Bit[2] = getChannel(AS7341_CHANNEL_480nm_F3);

	                             for(int i=0; i<4096/16; i++){
	                            	 pca9685_pwm(0x80, 0, 1, 4095-(16*i));
	                            	 pca9685_pwm(0x80, 0, 2, 4095-(16*i));
	                             			//pca9685_pwm(&hi2c1, I2C_address, 15, 0, 4095-(sharedvar*i));
	                             			//osDelay(shareddelay);
	                             }
	                             pca9685_mult_pwm(0x80, 0, 0, 4095);

	                             //can be read with cansend can0 60(2)#40 20 21 00 00 00 00 00
	                             //cansend can0 602#3F006201AF000000
	                             //cansend can0 602#4000620100000000

	                             CO_process_TPDO(CO, syncWas, 1000);

	                             CO_CANpolling_Tx(CO->CANmodule[0]);

	                             /* verify timer overflow */
	                             if(0) {
	                                 CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
	                             }
	                         }
	                        /* Process EEPROM */
	                    }
	                }
	            	  /* program exit ***************************************************************/
	                /* stop threads */


	                /* delete objects from memory */
	                CO_delete((uint32_t)&hcan1/* CAN module address */);


	                /* reset */
	                //return 0;
}


/*******************************************************************************/
void communicationReset(void){

}


/*******************************************************************************/
void programEnd(void){

}


/*******************************************************************************/
void programAsync(uint16_t timer1msDiff){

}


/*******************************************************************************/
void program1ms(void){
    uint16_t sharedvar=16;
    uint16_t sharedchannel=0xFFFF;
    uint16_t shareddelay = 5;

  	 uint8_t I2C_address = 0x80;
  	 PCA9685begin(hi2c1, 0);
  	 pca9685_init(I2C_address);

  		 for(int i=0; i<4096/sharedvar; i++){
  			pca9685_mult_pwm(I2C_address, sharedchannel, 0, 4095-(sharedvar*i));
  			//pca9685_pwm(&hi2c1, I2C_address, 15, 0, 4095-(sharedvar*i));
  			//osDelay(shareddelay);
  		 }

  	 	 for(int i=0; i<4096/sharedvar; i++){
  	 		pca9685_mult_pwm(I2C_address, sharedchannel, 0, (sharedvar*i));
  	 		//pca9685_pwm(&hi2c1, I2C_address, 15 ,0, 4095-(sharedvar*i));
  	 		//osDelay(shareddelay);
  	 	 }
}

/* timer thread executes in constant intervals ********************************/
static void tmrTask_thread(void){

    for(;;) {

        /* sleep for interval */

        INCREMENT_1MS(CO_timer1ms);

        if(CO->CANmodule[0]->CANnormal) {
            bool_t syncWas;

            /* Process Sync and read inputs */
            syncWas = CO_process_SYNC_RPDO(CO, TMR_TASK_INTERVAL);

            /* Further I/O or nonblocking application code may go here. */

            /* Write outputs */
            CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL);

            /* verify timer overflow */
            if(0) {
                CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
            }
        }
    }
}
/* CAN interrupt function *****************************************************/
void /* interrupt */ CO_CAN1InterruptHandler(void){
    CO_CANinterrupt(CO->CANmodule[0]);


    /* clear interrupt flag */
}
