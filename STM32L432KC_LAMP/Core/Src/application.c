/*
 * application.c
 *
 *  Created on: Apr 17, 2021
 *      Author: Boulet Nathan
 */

#include "PID.h"
#include "canOPEN.h"
#include "i2c.h"
#include "AS7341.h"
#include "MCP9600.h"
#include "PCA9685.h"
#include "INA226.h"
#include "application.h"
#include "dac.h"

#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */

volatile uint16_t CO_timer1ms = 0U; /* variable increments each millisecond */
PCM9600_t module_PCM9600_t;
PCA9685_t module_PCA9685_t;
INA226_t module_INA226_t;
PID_t module_PID_t;

//Specify the links and initial tuning parameters
uint16_t Kp = 0, Ki = 15, Kd = 0, offset_spectro = 0;
double Input, Output, Setpoint;
double desire;
double outputcorrected;

double consigne = 0;
float percent = 100;  //calcul the value of the spectro for each 10%
float cnst = 409.5;
float ref = 4095;
int go = 0;

//test PID to get the desired value
uint16_t colorvalue;
uint16_t yellowsaturation = 65535;

CO_ReturnError_t err;
uint16_t timer1msPrevious;
CO_NMT_reset_cmd_t reset_co = CO_RESET_NOT;

uint16_t sharedvar = 16;
int i = 0;

uint8_t initSensor() {
	uint8_t status = 0;
	/*
	 I2C_Module i2cm;
	 i2cm.instance=hi2c1;
	 i2cm.sdaPin=GPIO_PIN_6;
	 i2cm.sdaPort=GPIOB;
	 i2cm.sclPin=GPIO_PIN_7;
	 i2cm.sclPort=GPIOB;

	 I2C_ClearBusyFlagErratum(&i2cm);
	 */
	do {

		//status = PCM9600begin(&module_PCM9600_t, hi2c1);

		status = PCA9685begin(&module_PCA9685_t, hi2c1, 3);
		pca9685_init(&module_PCA9685_t);
		pca9685_pwm(&module_PCA9685_t, 0, 0, 4095);  //turn off pwm1
		pca9685_pwm(&module_PCA9685_t, 1, 0, 4095);  //turn off pwm2
		pca9685_pwm(&module_PCA9685_t, 2, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 3, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 4, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 5, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 6, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 7, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 8, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 9, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 10, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 11, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 12, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 13, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 14, 0, 4095);
		pca9685_pwm(&module_PCA9685_t, 15, 0, 4095);

		//status = AS7341init(hi2c1, 0x80);
		/*  Tint = (ATIME + 1) × (ASTEP + 1) × 2.78µs
		 *  Tint = 50ms
		 * */
		//status = setASTEP(999);
		//status = setATIME(100);
		//status = setGain(AS7341_GAIN_256X);
		//status = INA226begin(&module_INA226_t, hi2c1);
		//status = INA226configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
		// Calibrate INA226. Rshunt = 0.01 ohm, Max excepted current = 4A
		//status = INA226calibrate(0.01, 4);
		//osDelay(5000);
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_0] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_1] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_2] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_3] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_4] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_5] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_6] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_7] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_8] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_9] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_10] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_11] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_12] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_13] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_14] = 4095;
		CO_OD_RAM.pidRegister[PCA9685_CHANNEL_15] = 4095;

	} while (status != 0);

	return status;
}

void spectro() {
	//startReading(); /* reading in a loop */

	uint16_t buff[12];
	//do{
	if (!readAllChannels(buff)) {
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_415nm_F1] = getChannel(
				AS7341_CHANNEL_415nm_F1);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_445nm_F2] = getChannel(
				AS7341_CHANNEL_445nm_F2);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_480nm_F3] = getChannel(
				AS7341_CHANNEL_480nm_F3);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_515nm_F4] = getChannel(
				AS7341_CHANNEL_515nm_F4);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_555nm_F5] = getChannel(
				AS7341_CHANNEL_555nm_F5);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_590nm_F6] = getChannel(
				AS7341_CHANNEL_590nm_F6);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_630nm_F7] = getChannel(
				AS7341_CHANNEL_630nm_F7);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_680nm_F8] = getChannel(
				AS7341_CHANNEL_680nm_F8);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_CLEAR] = getChannel(
				AS7341_CHANNEL_CLEAR);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_NIR] = getChannel(
				AS7341_CHANNEL_NIR);
	}
	//}while(1);
}

void calibration() {
	uint16_t buff[12];
	//do{

	if (!readAllChannels(buff)) {
		//cansend can0 602#3B00180510000000 ask for PDO every 10s
		//cansend can0 602#4001640100000000
		//!!!!weird number if scan is too fast
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_415nm_F1] = getChannel(
				AS7341_CHANNEL_415nm_F1);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_445nm_F2] = getChannel(
				AS7341_CHANNEL_445nm_F2);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_480nm_F3] = getChannel(
				AS7341_CHANNEL_480nm_F3);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_515nm_F4] = getChannel(
				AS7341_CHANNEL_515nm_F4);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_555nm_F5] = getChannel(
				AS7341_CHANNEL_555nm_F5);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_590nm_F6] = getChannel(
				AS7341_CHANNEL_590nm_F6);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_630nm_F7] = getChannel(
				AS7341_CHANNEL_630nm_F7);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_680nm_F8] = getChannel(
				AS7341_CHANNEL_680nm_F8);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_CLEAR] = getChannel(
				AS7341_CHANNEL_CLEAR);
		CO_OD_RAM.spectroRegister[AS7341_CHANNEL_NIR] = getChannel(
				AS7341_CHANNEL_NIR);

	}
}
void temperature(void) {
	CO_OD_RAM.temperatureRegister = getThermocoupleTemp(&module_PCM9600_t, 0);
}

void multimetre(void) {
	CO_OD_RAM.elecRegister[0] = getMaxCurrent();
	CO_OD_RAM.elecRegister[2] = getMaxPower();
	//CO_OD_RAM.elecRegister[1]=(float)CO_OD_RAM.elecRegister[2]/CO_OD_RAM.elecRegister[0];
}

void ventilator(uint32_t fan) {
	DAC1->DHR12R1 = fan;
}

void scenario(void) {
	uint16_t sharedvar = 16;
	//uint16_t sharedchannel=0xFFFF;
	//uint16_t shareddelay = 5;

	//uint8_t I2C_address = 0x80;
	PCA9685_t module;
	PCA9685begin(&module, hi2c1, 3);
	pca9685_init(&module);
	pca9685_pwm(&module, 0, 0, 4095);    //turn off pwm1
	pca9685_pwm(&module, 1, 0, 4095);    //turn off pwm2
	for (;;) {
		for (int i = 0; i < 1024 / sharedvar; i++) {
			pca9685_pwm(&module, 0, 0, 4095 - (sharedvar * i));  //turn off pwm1
			pca9685_pwm(&module, 1, 0, 4095 - (sharedvar * i));  //turn off pwm1
			//HAL_Delay(10);
			//pca9685_mult_pwm(0x80, 1, 0, 4095-(16*i));
			//pca9685_pwm(0x80, 1, 0, 4095-(16*i));

		}
	}
}

void test_cycle(void) {
	for(int a = 0; a<16; a++){
		pca9685_pwm(&module_PCA9685_t, a, 0, 4000);
		HAL_Delay(500);

		pca9685_pwm(&module_PCA9685_t, a, 0, 4095);
		HAL_Delay(500);

	}
	/*
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_0, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_0]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_1, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_1]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_2, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_2]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_3, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_3]);


	HAL_Delay(500);

	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_0, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_1, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_2, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_3, 0, 3000);



	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_4, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_4]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_5, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_5]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_6, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_6]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_7, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_7]);

	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_8, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_8]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_9, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_9]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_10, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_10]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_11, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_11]);

	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_12, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_12]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_13, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_13]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_14, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_14]);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_15, 0,
			CO_OD_RAM.pidRegister[PCA9685_CHANNEL_15]);

	HAL_Delay(500);


	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_4, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_5, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_6, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_7, 0, 3000);

	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_8, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_9, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_10, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_11, 0, 3000);

	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_12, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_13, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_14, 0, 3000);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_15, 0, 3000);

	HAL_Delay(500);

	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_0, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_1, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_2, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_3, 0, 3);

	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_4, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_5, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_6, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_7, 0, 3);

	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_8, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_9, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_10, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_11, 0, 3);

	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_12, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_13, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_14, 0, 3);
	pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_15, 0, 3);

	HAL_Delay(500);
	*/
}

/*******************************************************************************/
void programStart(void) {
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT;

	/* Configure microcontroller. */
	initSensor();
	//Define Variables we'll be connecting to

	//multimetre();

	/*
	 * PID test
	 *
	 CO_OD_RAM.pidRegister[0] = Kp;

	 PID(&module_PID_t, &Input, &Output, &Setpoint, Kp,  Ki,  Kd,  P_ON_E,  DIRECT);
	 SetMode(&module_PID_t, AUTOMATIC);
	 consigne=(yellowsaturation*0.05);
	 module_PID_t.mySetpoint=&consigne;
	 */

	/* initialize EEPROM */
	/* increase variable each startup. Variable is stored in EEPROM. */
	OD_powerOnCounter++;

	while (reset != CO_RESET_APP) {
		/* CANopen communication reset - initialize CANopen objects *******************/
		CO_ReturnError_t err;
		uint16_t timer1msPrevious;

		/* disable CAN and CAN interrupts */
		HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);    //added by me
		HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);			 //added by me
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);	 //added by me
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);			 //added by me
		//NVIC_SystemReset(); //added by me
		bool_t syncWas;

		/* initialize CANopen */

		/* Configure Timer interrupt function for execution every 1 millisecond */
		/* Configure CAN transmit and receive interrupt */
		err = CO_init((uint32_t) &hcan1, 2, 125);

		for (int i = 0; i < ODL_errorStatusBits_stringLength; i++) {
			OD_errorStatusBits[i] = 0;
		}

		if (err != CO_ERROR_NO) {
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
		//CO->NMT->HBproducerTimer = 0xFF;

		/* CAN1 interrupt Init */
		//HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
		//HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
		//HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
		//HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		while (reset_co == CO_RESET_NOT) {
			/* loop for normal program execution ******************************************/
			INCREMENT_1MS(CO_timer1ms);
			uint16_t timer1msCopy, timer1msDiff;

			timer1msCopy = CO_timer1ms;
			timer1msDiff = timer1msCopy - timer1msPrevious;
			timer1msPrevious = timer1msCopy;

			/* CANopen process */

			reset_co = CO_process(CO, 1, NULL);

			/* Nonblocking application code may go here. */
			if (CO->CANmodule[0]->CANnormal) {
				/* Process Sync and read inputs */
				syncWas = CO_process_SYNC_RPDO(CO, TMR_TASK_INTERVAL);

				switch (CO->NMT->operatingState) {
				case CO_NMT_OPERATIONAL:
					//Further I/O or nonblocking application code may go here. //
					/*
					 * Program to control the lamp
					 *
					 * cansend can0 602#2300250110000000
					 * cansend can0 602#23002501A00F0000
					 * cansend can0 602#23002501B80B0000
					 *
					 */
					//pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_0, 0, 4095 - (sharedvar * i));	//turn off pwm1
					//pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_0, 0, 4095 - (sharedvar * i));	//turn off pwm1
					//pca9685_pwm(&module_PCA9685_t, PCA9685_CHANNEL_1, 0, 4095 - (sharedvar * i));
					//ventilator(255);
					//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,4096);
					/*
					 * 602#4302250100000000
					 */
					//temperature();
					//all_led_off(&module_PCA9685_t);
					//temperature();
					test_cycle();
					break;
				case CO_NMT_STOPPED:
					pca9685_pwm(&module_PCA9685_t, 0, 0, 4095);	//turn off pwm1
					pca9685_pwm(&module_PCA9685_t, 1, 0, 4095);	//turn off pwm2
					pca9685_pwm(&module_PCA9685_t, 2, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 3, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 4, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 5, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 6, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 7, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 8, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 9, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 10, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 11, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 12, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 13, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 14, 0, 4095);
					pca9685_pwm(&module_PCA9685_t, 15, 0, 4095);
					break;
				case CO_NMT_INITIALIZING:
					initSensor();
					break;
				case CO_NMT_PRE_OPERATIONAL:

					break;

				}
			}

			//can be read with cansend can0 60(2)#40 20 21 00 00 00 00 00
			//cansend can0 602#3F006201AF000000
			//cansend can0 602#4000620100000000

			CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL);
			CO_CANpolling_Tx(CO->CANmodule[0]);

			/* verify timer overflow */
			if (0) {
				CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW,
				CO_EMC_SOFTWARE_INTERNAL, 0U);
			}

			/* Process EEPROM */
		}
	}

	/* program exit ***************************************************************/
	/* stop threads */

	/* delete objects from memory */
	CO_delete((uint32_t) &hcan1);/* CAN module address */

	/* reset */
//return 0;
}

/*******************************************************************************/
void communicationReset(void) {

}

/*******************************************************************************/
void programEnd(void) {

}

/*******************************************************************************/
void programAsync(uint16_t timer1msDiff) {

}

/*******************************************************************************/
void program1ms(void) {
//scenario();
}

uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax,
		uint32_t au32_OUTmin, uint32_t au32_OUTmax) {
	return ((((au32_IN - au32_INmin) * (au32_OUTmax - au32_OUTmin))
			/ (au32_INmax - au32_INmin)) + au32_OUTmin);
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
