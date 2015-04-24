/*
 * lab.c
 *
 *  Created on: Jan 26, 2015
 *      Author: hanley6
 */

/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*--------------- Includes ---------------*/
#include "lab.h"
#include "math.h"
/*------------- End Includes -------------*/

/*---------- Function Prototypes ---------*/
void lab2(void);
/*-------- End Function Prototypes -------*/


/*--------------- Globals ----------------*/
struct imuSensor imusensor;
struct U u;

/////////// HUMMINGBIRD PARAMETERS /////////////
float mass = 1.5;  	// TO BE ENTERED!			
float kF = 1.607e-5; 	// TO BE ENTERED!		
float kM = 3.2475e-7; 	// TO BE ENTERED!		
float l = 0.215;    	// TO BE ENTERED!		
float MAXPHI2 = powf(779.5638,2.0);	// TO BE ENTERED!
float MINPHI2 = powf(112.705875,2.0);	// TO BE ENTERED!
////////////////////////////////////////////

// Other Declarations
float Winv[4*4];
float cnt_u[4];
float omega_cmd2[4];
float omega_cmd[4];
float cmd[4];
float roll_desired;
float pitch_desired;
float yaw_desired;
float x_nom[4];
float g = 9.80665;	// Standard Gravity m/s^2
float z[3];		// Sensor measurement for Kalman Filter
float dt;

/*------------- End Globals --------------*/

/*----------------------------------------------------------------------*/
/*---------------------------- End Preamble ----------------------------*/
/*----------------------------------------------------------------------*/


/*----------------------------------------------------------------------*/
/*------------------ Main Loop (called at 1 kHz) -----------------------*/
/*----------------------------------------------------------------------*/
void lab(void)
{
	// Lab 2
	lab2();

	// Convert Controller Outputs to Motor Inputs
	Command();
	
}
/*----------------------------------------------------------------------*/
/*---------------- End Main Loop (called at 1 kHz) ---------------------*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/*------------------------------ Helpers -------------------------------*/
/*----------------------------------------------------------------------*/

/*-------------- Controller --------------*/
void lab2() {
	
	// Desired	
	pitch_desired = 0.0;
	
	// INNER LOOP
		// Hummingbird
	float Ktx_P = 1.0;	// Proportional Roll
	float Ktx_D = 0.3;	// Derivative Roll
	float Kty_P = 1.0;	// Proportional Pitch
	float Kty_D = 0.3;	// Derivative Pitch
	float Ktz_P = 0.08;	// Proportional Yaw
	float Ktz_D = 0.04;	// Derivative Yaw
		
	// Inner Loop PD 
	// Note: Lab 2 only tests Pitch control. Do not uncomment roll and yaw lines.
	cnt_u[0] = 0.0; // Ktx_P*(roll_desired-imusensor.dThetax)-Ktx_D*(imusensor.dOmegax);
	cnt_u[1] = Kty_P*(pitch_desired-imusensor.dThetay)-Kty_D*(imusensor.dOmegay);
	cnt_u[2] = 0.0;//Ktz_P*(yaw_desired-imusensor.dThetaz)-Ktz_D*(imusensor.dOmegaz);

	// For Keeping data
	u.u1 = cnt_u[0];
	u.u2 = cnt_u[1];
	u.u3 = cnt_u[2];
	u.u4 = cnt_u[3];
}
/*------------ End Controller ------------*/

/*---------------- Command ---------------*/
void Command() {
	/////////////// Controller Settings ////////////
	WO_SDK.ctrl_mode=0x00;  //0x00: direct individual motor control (individual commands for motors 0...3)
				//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw 
				//      and thrust inputs; no attitude controller active
				//0x02: attitude and throttle control: commands are input for standard attitude controller
				//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;	//0: disable control by HL processor
				//1: enable control by HL processor
	////////////////////////////////////////////////

	//////// Translate commanded torques and thrust into rotor speed and commands ////////////
	// NOTE METHOD BELOW ASSUMES THAT CG IS IN THE SAME PLANE AS THE ROTORS
	float twolkF = 1.0/(2.0*l*kF);
	float fourkF = 1.0/(4.0*kF);
	float fourkM = 1.0/(4.0*kM);

	Winv[0] = 0;
	Winv[1] = twolkF;
	Winv[2] = -fourkM;
	Winv[3] = fourkF;
	Winv[1*4+0] = -twolkF;
	Winv[1*4+1] = 0;
	Winv[1*4+2] = fourkM;
	Winv[1*4+3] = fourkF;
	Winv[2*4+0] = 0;
	Winv[2*4+1] = -twolkF;
	Winv[2*4+2] = -fourkM;
	Winv[2*4+3] = fourkF;
	Winv[3*4+0] = twolkF;
	Winv[3*4+1] = 0;
	Winv[3*4+2] = fourkM;
	Winv[3*4+3] = fourkF;

	matrix_multiply(4,4,1,Winv,cnt_u,omega_cmd2);

	int i;
	for (i=0; i<4; i++) {
		if (omega_cmd2[i] > MAXPHI2) {
			omega_cmd2[i] = MAXPHI2;
		}
		else if (omega_cmd2[i] < MINPHI2) {
			omega_cmd2[i] = MINPHI2;
		}
		omega_cmd[i] = sqrt(omega_cmd2[i]);
		// Translate Desired Rotor Speed into Motor Commands
		// NOTE: THIS IS FOR THE PELICAN
		cmd[i] = 0.238432*omega_cmd[i] - 25.872642;	// Verify

		// Below is a safety measure. We want to make sure the motor 
		// commands are never 0 so that the motors will always keep 
		// spinning. Also makes sure that motor commands stay within range.
		// NOTE: THIS SHOULD BE UNNECESSARY. I IMPLEMENTED THIS AS AN EXTRA 
		// SAFETY MEASURE
		if (cmd[i] < 1.0) {
			cmd[i] = 1.0;
		}
		else if (cmd[i] > 200.0) {
			cmd[i] = 200.0;
		} 
	}
	/////////////////////////////////////////////////////////////////////////////////////////////

	/////// Send Motor Commands ///////////
	WO_Direct_Individual_Motor_Control.motor[0] = cmd[0];
	WO_Direct_Individual_Motor_Control.motor[3] = cmd[1];
	WO_Direct_Individual_Motor_Control.motor[1] = cmd[2];
	WO_Direct_Individual_Motor_Control.motor[2] = cmd[3];
	///////////////////////////////////////
}
/*-------------- End Command -------------*/

/*----------------------------------------------------------------------*/
/*---------------------------- End Helpers -----------------------------*/
/*----------------------------------------------------------------------*/

