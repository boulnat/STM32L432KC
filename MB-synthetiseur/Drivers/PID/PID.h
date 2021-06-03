/*
 * PID.h
 *
 *  Created on: May 18, 2021
 *      Author: boulnat
 */

#ifndef PID_PID_H_B_
#define PID_PID_H_B_
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define LIBRARY_VERSION	1.2.1


//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1

typedef struct {
	double dispKp;				// * we'll hold on to the tuning parameters in user-entered
	double dispKi;				//   format for display purposes
	double dispKd;				//

	double kp;                  // * (P)roportional Tuning Parameter
	double ki;                  // * (I)ntegral Tuning Parameter
	double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

	double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
	double *myOutput;             //   This creates a hard link between the variables and the
	double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
	                                  //   what these values are.  with pointers we'll just know.

	unsigned long lastTime;
	double outputSum, lastInput;

	unsigned long SampleTime;
	double outMin, outMax;
}PID_t;

void beginPIF(PID_t *module);

//commonly used functions **************************************************************************
void PID(PID_t *pid, double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and
        double, double, double, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

//PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and
//        double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here

void SetMode(PID_t *pid, int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

bool Compute(PID_t *pid);                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

void SetOutputLimits(PID_t *pid, double, double); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application

//available but not commonly used functions ********************************************************
//void SetTunings(double, double,       // * While most users will set the tunings once in the
//                    double);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
void SetTunings(PID_t *pid, double, double,       // * overload for specifying proportional mode
                    double, int);

void SetControllerDirection(PID_t *pid, int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
void SetSampleTime(PID_t *pid, int);              // * sets the frequency, in Milliseconds, with which
                                          //   the PID calculation is performed.  default is 100



//Display functions ****************************************************************
double GetKp(PID_t *pid);						  // These functions query the pid for interal values.
double GetKi(PID_t *pid);						  //  they were created mainly for the pid front-end,
double GetKd(PID_t *pid);						  // where it's important to know what is actually
int GetMode(PID_t *pid);						  //  inside the PID.
int GetDirection(PID_t *pid);					  //


void Initialize(PID_t *pid);

bool inAuto, pOnE;


#endif /* PID_PID_H_B_ */
