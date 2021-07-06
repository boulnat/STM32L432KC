/*
 * application.c
 *
 *  Created on: Apr 17, 2021
 *      Author: Sajl
 */

#include <PID.h>
#include "CANopen.h"
#include "i2c.h"
#include "AS7341.h"
#include "MCP9600.h"
#include "PCA9685.h"
#include "application.h"

#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */


volatile uint16_t   CO_timer1ms = 0U;   /* variable increments each millisecond */
PCM9600_t module_PCM9600_t;
PCA9685_t module_PCA9685_t;
PID_t module_PID_t;

CO_ReturnError_t err;
uint16_t timer1msPrevious;
CO_NMT_reset_cmd_t reset_co = CO_RESET_NOT;

uint8_t initSensor(){
	uint8_t status = 0;
	I2C_Module i2cm;
	i2cm.instance=hi2c1;
	i2cm.sdaPin=GPIO_PIN_6;
	i2cm.sdaPort=GPIOB;
	i2cm.sclPin=GPIO_PIN_7;
	i2cm.sclPort=GPIOB;

	I2C_ClearBusyFlagErratum(&i2cm);

	do{

	status = PCM9600begin(&module_PCM9600_t, hi2c1);

	status = PCA9685begin(&module_PCA9685_t,hi2c1,3);
	pca9685_init(&module_PCA9685_t);
	pca9685_pwm(&module_PCA9685_t, 0, 0, 4095);//turn off pwm1
	pca9685_pwm(&module_PCA9685_t, 1, 0, 4095);//turn off pwm2


	status = AS7341init(hi2c1, 0x80);
	/*  Tint = (ATIME + 1) × (ASTEP + 1) × 2.78µs
	 *  Tint = 50ms
	 * */
	status = setASTEP(999);
	status = setATIME(100);
	status = setGain(AS7341_GAIN_256X);

	osDelay(5000);

	}while(status!=0);

	return status;
}


void spectro(){
      //startReading(); /* reading in a loop */

      uint16_t buff[12];
      //do{
		  if(!readAllChannels(buff)){


			  //cansend can0 602#3B00180510000000 ask for PDO every 10s
			  //cansend can0 602#4001640100000000
			  //!!!!weird number if scan is too fast
			  //CO_OD_RAM.readAnalogueInput16Bit[0] = getChannel(AS7341_CHANNEL_415nm_F1);//getChannel(AS7341_CHANNEL_415nm_F1); //added by me set the value of an object
			  //CO_OD_RAM.readAnalogueInput16Bit[1] = getChannel(AS7341_CHANNEL_445nm_F2);
			  //CO_OD_RAM.readAnalogueInput16Bit[2] = getChannel(AS7341_CHANNEL_480nm_F3);
			  CO_OD_RAM.spectroRegister[CHANNEL_1] = getChannel(AS7341_CHANNEL_590nm_F6);

			  //CO_OD_RAM.readAnalogueInput16Bit[3] = getThermocoupleTemp(&module,0);
			  //scenario();
			  //startReading();
		  }
      //}while(1);
}

void temperature(void){
	CO_OD_RAM.pidRegister[T] = getThermocoupleTemp(&module_PCM9600_t,0);
}
void scenario(void){
    uint16_t sharedvar=16;
    //uint16_t sharedchannel=0xFFFF;
    //uint16_t shareddelay = 5;

  	 //uint8_t I2C_address = 0x80;
  	 PCA9685_t module;
  	 PCA9685begin(&module,hi2c1,3);
  	 pca9685_init(&module);
	 pca9685_pwm(&module, 0, 0, 4095);//turn off pwm1
	 pca9685_pwm(&module, 1, 0, 4095);//turn off pwm2
	 for(;;){
	         for(int i=0; i<1024/sharedvar; i++){
	        	 pca9685_pwm(&module, 0, 0,  4095-(sharedvar*i));//turn off pwm1
	        	 pca9685_pwm(&module, 1, 0,  4095-(sharedvar*i));//turn off pwm1

	        	 //HAL_Delay(10);
	        	 //pca9685_mult_pwm(0x80, 1, 0, 4095-(16*i));
	        	 //pca9685_pwm(0x80, 1, 0, 4095-(16*i));
	         }
  	  }
}
/*******************************************************************************/
void programStart(void){
	  CO_NMT_reset_cmd_t reset = CO_RESET_NOT;

	  /* Configure microcontroller. */
	  initSensor();
	  //Define Variables we'll be connecting to

	  //Specify the links and initial tuning parameters
	  uint16_t Kp=10, Ki=2, Kd=2, offset_spectro=0;
	  double Input, Output, Setpoint;

	  CO_OD_RAM.pidRegister[0] = Kp;

	  PID(&module_PID_t, &Input, &Output, &Setpoint, Kp,  Ki,  Kd,  P_ON_M,  DIRECT);
	  SetMode(&module_PID_t, AUTOMATIC);
	  Setpoint = 10000;

	  spectro();
	  offset_spectro = getChannel(AS7341_CHANNEL_590nm_F6);

	  //PIDInit(&module_PID_t, Kp, Ki, Kd, 0.1, 0, 65535, AUTOMATIC, DIRECT);
	  //module_PID_t.setpoint = 55000;

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
	          /* Configure CAN transmit and receive interrupt */
	          err = CO_init((uint32_t)&hcan1, 2, 20);

	          //CO_errorReset();
	          for (int i = 0; i < ODL_errorStatusBits_stringLength; i++) {
	        	  OD_errorStatusBits[i] = 0;
	          }

	          if(err != CO_ERROR_NO)
	             {
	            	 //TODO behavior in a case of the stack error. Currently not defined.
	            	 //_Error_Handler(0, 0);
	             }

	          /* start CAN */
	          CO_CANsetNormalMode(CO->CANmodule[0]);


	          reset_co = CO_RESET_NOT;
	          timer1msPrevious = CO_timer1ms;  //added by me
	          //put the device in preoperational waiting for master to put in operational
	          //cansend can0 000#010(0)
	          //CO->NMT->operatingState = CO_NMT_OPERATIONAL;//added by me
	          //CO_OD_ROM.producerHeartbeatTime = 0x50;//added by me
	          CO_OD_RAM.errorRegister=0;

	          while(reset_co == CO_RESET_NOT){
	                  	  /* loop for normal program execution ******************************************/
	        	  	  	  	INCREMENT_1MS(CO_timer1ms);
	                        uint16_t timer1msCopy, timer1msDiff;

	                        timer1msCopy = CO_timer1ms;
	                        timer1msDiff = timer1msCopy - timer1msPrevious;
	                        timer1msPrevious = timer1msCopy;

	                        /* CANopen process */

	                        reset_co = CO_process(CO,100, NULL);

	                        /* Nonblocking application code may go here. */
	                        //if(CO->CANmodule[0]->CANnormal)
	                        if(CO->NMT->operatingState)
	                        {

	                        	bool_t syncWas;

	                             /* Process Sync and read inputs */


	                             syncWas = CO_process_SYNC_RPDO(CO, 1000);

	                             //Kp = CO_OD_RAM.pidRegister[0];

	                             /* Further I/O or nonblocking application code may go here. */
	                             /* 16 bit to 12 bit */
	                             //module_PID_t.input = getChannel(AS7341_CHANNEL_590nm_F6);
	                             //Input = getChannel(AS7341_CHANNEL_590nm_F6);


	                             /* hysteresis */
	                             /*
	                             if((Input+10000)>Setpoint||(Input-10000)<Setpoint){
	                            	 Compute(&module_PID_t);
	                            	 Output = MAP(Input, 0, (65535-offset_spectro), 0, 4095);
	                            	 pca9685_pwm(&module_PCA9685_t, CH1, 0, Output);
	                             }
	                              */
	                             /* 0x2500 */
	                             //if(CO_OD_RAM.pidRegister[ST]){
	                            	 pca9685_pwm(&module_PCA9685_t, CH1, 0, CO_OD_RAM.pidRegister[PWM]);
	                             //}
	                             spectro();
	                             temperature();

	                             /* Write outputs */
	                             //CO->TPDO[0]->CANtxBuff[0].data[0]=getChannel(AS7341_CHANNEL_415nm_F1); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[1]=getChannel(AS7341_CHANNEL_415nm_F1); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[2]=getChannel(AS7341_CHANNEL_480nm_F3); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[3]=getChannel(AS7341_CHANNEL_515nm_F4); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[4]=getChannel(AS7341_CHANNEL_555nm_F5); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[5]=getChannel(AS7341_CHANNEL_590nm_F6); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[6]=getChannel(AS7341_CHANNEL_630nm_F7); //added by me
	                             //CO->TPDO[0]->CANtxBuff[0].data[7]=getChannel(AS7341_CHANNEL_680nm_F8); //added by me




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
	//scenario();
}

uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

/* timer thread executes in constant intervals ********************************/
/*
static void tmrTask_thread(void){

    for(;;) {



        INCREMENT_1MS(CO_timer1ms);

        if(CO->CANmodule[0]->CANnormal) {
            bool_t syncWas;


            syncWas = CO_process_SYNC_RPDO(CO, TMR_TASK_INTERVAL);


            CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL);


            if(0) {
                CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
            }
        }
    }
}
*/
/* CAN interrupt function *****************************************************/
/*
void  CO_CAN1InterruptHandler(void){
    CO_CANinterrupt(CO->CANmodule[0]);
}
*/
