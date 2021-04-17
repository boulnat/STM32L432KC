/*
 * application.c
 *
 *  Created on: Apr 17, 2021
 *      Author: boulnat
 */

#include "CANopen.h"
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
	  //OD_powerOnCounter++;
	  while(reset != CO_RESET_APP){
	  /* CANopen communication reset - initialize CANopen objects *******************/
	          CO_ReturnError_t err;
	          uint16_t timer1msPrevious;

	          /* disable CAN and CAN interrupts */


	          /* initialize CANopen */
	          //err = CO_init(0/* CAN module address */, 10/* NodeID */, 125 /* bit rate */);
	          //if(err != CO_ERROR_NO){
	          //    while(1);
	              /* CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err); */
	          //}


	          /* Configure Timer interrupt function for execution every 1 millisecond */


	          /* Configure CAN transmit and receive interrupt */
	          err = CO_init((uint32_t)&hcan1, 4, 20);
	          if(err != CO_ERROR_NO)
	             {
	            	 //TODO behavior in a case of the stack error. Currently not defined.
	            	 //_Error_Handler(0, 0);
	             }

	          /* start CAN */
	          CO_CANsetNormalMode(CO->CANmodule[0]);

	          reset = CO_RESET_NOT;
	          timer1msPrevious = CO_timer1ms;

	          while(reset == CO_RESET_NOT){
	                  	  /* loop for normal program execution ******************************************/
	                        uint16_t timer1msCopy, timer1msDiff;

	                        timer1msCopy = CO_timer1ms;
	                        timer1msDiff = timer1msCopy - timer1msPrevious;
	                        timer1msPrevious = timer1msCopy;


	                        /* CANopen process */
	                        reset = CO_process(CO, timer1msDiff, NULL);

	                        /* Nonblocking application code may go here. */
	                        if(CO->CANmodule[0]->CANnormal)
	                        {
	                             bool_t syncWas;

	                             /* Process Sync and read inputs */
	                             syncWas = CO_process_SYNC_RPDO(CO, 1000);

	                             /* Further I/O or nonblocking application code may go here. */
	                             CO_TPDO_t              *TPDO;
	                             CO_EM_t                *em;
	                             CO_SDO_t               *SDO;
	                             uint8_t                *operatingState;
	                             uint8_t                 nodeId;
	                             uint16_t                defaultCOB_ID;
	                             uint8_t                 restrictionFlags;
	                             const CO_TPDOCommPar_t *TPDOCommPar;
	                             const CO_TPDOMapPar_t  *TPDOMapPar;
	                             uint16_t                idx_TPDOCommPar;
	                             uint16_t                idx_TPDOMapPar;
	                             CO_CANmodule_t         *CANdevTx;
	                             uint16_t                CANdevTxIdx;
	                             CO_TPDO_init(TPDO,em,SDO,operatingState,nodeID,defaultCOB_ID,restrictionFlags,TPDOCommPar,TPDOMapPar,idx_TPDOCommPar,idx_TPDOMapPar,CANdevTx,CANdevTxIdx);
								 CO_TPDOsend();

	                             /* Write outputs */
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
	                CO_delete(0/* CAN module address */);


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
