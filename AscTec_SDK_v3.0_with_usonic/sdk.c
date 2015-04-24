/*

AscTec SDK 3.0

Copyright (c) 2011, Ascending Technologies GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

 */

#include "main.h"
#include "sdk.h"
#include "LL_HL_comm.h"
#include "gpsmath.h"
#include "sdk_telemetry.h"
#include "uart.h"
#include "system.h"
#include "lpc_aci_eeprom.h"
#include <stdio.h>
#include "LPC214x.h"
#include "hardware.h"
#include "irq.h"
#include "lab.h"

#ifdef MATLAB
#include "..\custom_mdl\onboard_matlab_ert_rtw\onboard_matlab.h"
#endif

// Dan Block Add
#define USONIC_INIT_DELAY 2000
#define SWITCH_INIT_DELAY 2000
#define SDCARD_START_DELAY 4000
// End Dan Block Add

struct WO_SDK_STRUCT WO_SDK;
struct WO_CTRL_INPUT WO_CTRL_Input;
struct RO_RC_DATA RO_RC_Data;
struct RO_ALL_DATA RO_ALL_Data;
struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;
struct WO_DIRECT_INDIVIDUAL_MOTOR_CONTROL WO_Direct_Individual_Motor_Control;

//waypoint example global variables for jeti display
unsigned char wpExampleWpNr=0;
unsigned char wpExampleActive=0;

//emergency mode variables
unsigned char emergencyMode;
unsigned char emergencyModeUpdate=0;

#ifdef MATLAB
unsigned char xbee_send_flag=0;
unsigned char triggerSaveMatlabParams=0; //trigger command to save matlab parameters to flash
struct MATLAB_DEBUG matlab_debug;
struct MATLAB_UART matlab_uart, matlab_uart_tmp;
struct MATLAB_PARAMS matlab_params, matlab_params_tmp;

void SDK_matlabMainLoop(void);
#endif
void SDK_EXAMPLE_direct_individual_motor_commands(void);
void SDK_EXAMPLE_direct_motor_commands_with_standard_output_mapping(void);
void SDK_EXAMPLE_attitude_commands(void);
void SDK_EXAMPLE_gps_waypoint_control(void);
int SDK_EXAMPLE_turn_motors_on(void);
int SDK_EXAMPLE_turn_motors_off(void);


// Start Added by Dan Block
unsigned long my_sdkloop_counter = 0;
static int timer = 1;
int firebit = 1;
int UsonicTimer = 0;
unsigned int SPI0data = 0;
unsigned int SPI0command = 0;
int f28027_ready = 0;
int UsonicData = 0;
int NewUsonicData = 0;
int USMaxBot_range1 = 0;

int SwitchTimer = 0;
int readSwitchbit = 0;
int MagnetSwitch = 0;
int NewMagnetSwitch = 0;
int GotMagnet = 0;

#define MAX_SPI_SEND_SHORTS  50
int SPItxCount = 0;
int SPItxSize = 0;
unsigned short int SPItxArray[MAX_SPI_SEND_SHORTS];
int	SPItxDone = 1;
int numMissedTx = 0;

typedef union float2short_tag {
	float fl;
	unsigned short int sh[2];
} float2short;

float2short f2s;

int testcount = 0;
float testarray[5];

void SPI0Handler(void) __irq
{
	IENABLE;

	IOSET0 = (1<<EXT_NCS); // set CS back high
	if ((S0SPSR&0x80) == 0x80) {
		SPI0data = S0SPDR; // read data
	}

	if (SPI0command == 0xCC01) {  // command to tell f28027 to fire ultrasonic sensor
		if ((SPI0data == 0xBABE) || (SPI0data == 0xCABB)) {
			firebit = 0;
		} else { // error go back to not ready
			f28027_ready = 0;
		}
	} else if (SPI0command == 0xCC02) {  // command to read date from f28027 ultrasonic reading
		//mytest = SPI0data & 0xFF00;
		if ( (SPI0data & 0xFF00) == 0xDD00) {
			UsonicData = SPI0data & 0xFF;
			NewUsonicData = 1;
			firebit = 1;
			UsonicTimer = 0;
		} else {  // error go back to not ready
			f28027_ready = 0;
		}

	} else if (SPI0command == 0xCC03) {  // command to just check if board attached to SPI
		if (SPI0data == 0xCABB) {
			f28027_ready = 1;
			firebit = 1;
			readSwitchbit = 1;
			UsonicTimer = 0;
			SwitchTimer = 0;
		}
	} else if (SPI0command == 0xCC04) { // command to beaglebone to store data to SD card
		if (SPItxCount < SPItxSize) {
			IOCLR0 = (1<<EXT_NCS);
			S0SPDR = SPItxArray[SPItxCount];
			SPItxCount++;
		} else {
			SPItxDone = 1;
		}
	} else if (SPI0command == 0xCC05) {  // command to tell f28027 to transfer magnet switch state
		if ((SPI0data == 0xBABE) || (SPI0data == 0xCABB)) {
			readSwitchbit = 0;
		} else { // error go back to not ready
			f28027_ready = 0;
		}
	} else if (SPI0command == 0xCC06) {  // command to read magnet switch state
		if ( (SPI0data & 0xFF00) == 0xDD00) {
			MagnetSwitch = SPI0data & 0xFF;
			NewMagnetSwitch = 1;
			readSwitchbit = 1;
			SwitchTimer = 0;
		} else {  // error go back to not ready
			f28027_ready = 0;
		}

	}


	S0SPINT = 0x1;  // clear interrupt

	IDISABLE;
	VICVectAddr = 0;		/* Acknowledge Interrupt */
}



void SDK_init(void) {

//	ultrasound_timer = 0;
//	ultrasound_z = 0;
//	ultrasound_vz = 0;
//	ultrasound_z_prev = 0;
//	ultrasound_z_error_sum = 0;

	my_sdkloop_counter = 0;

}

// End Dan Block Added

void SDK_mainloop(void)
{
//#ifdef MATLAB
	//SDK_matlabMainLoop(); //this runs only in combination with the AscTec Simulink Toolkit

	//jeti telemetry can always be activated. You may deactivate this call if you don't have the AscTec Telemetry package.
	//SDK_jetiAscTecExampleRun();

//#else //write your own C-cod e within this function

//	lab();

	//SDK_jetiAscTecExampleRun();

	//if (wpExampleActive) //this is used to activate the waypoint example via the jeti telemetry display
	//	SDK_EXAMPLE_gps_waypoint_control();

//#endif

// Added By Dan Block
	if (NewUsonicData == 1) {
		USMaxBot_range1 = UsonicData;
		//filter_ultrasound_z();
		NewUsonicData = 0;
	}
	if (NewMagnetSwitch == 1) {
		GotMagnet = ~MagnetSwitch & 0x1;
		NewMagnetSwitch = 0;
	}
// End Add


	////////////// Call Lab Code Here !  /////////////
	lab();

//	 Add by Dan Block
//	 Code to read magnet switch from attached 28027 board
//	if (timer > SWITCH_INIT_DELAY) { //wait SWITCH_INIT_DELAY ms before talking to 28027
//		if (f28027_ready == 1) {
//			SwitchTimer++;
//			if (readSwitchbit == 1) {
//				if (SwitchTimer == 25) {  // read every 25 ms
//					IOCLR0 = (1<<EXT_NCS);
//					SPI0command = 0xCC05;
//					S0SPDR = SPI0command;  // send Sample Switch Command
//				}
//			} else if (SwitchTimer == 26) {
//				IOCLR0 = (1<<EXT_NCS);
//				SPI0command = 0xCC06;
//				S0SPDR = SPI0command;  // send transfer Switch State Command
//			}
//		} else {
//			IOCLR0 = (1<<EXT_NCS);
//			SPI0command = 0xCC03;
//			S0SPDR = SPI0command;  // send Are you Alive command
//		}
//	}

// This code was communicating with BBB
//	if (timer > SDCARD_START_DELAY) {
//		if(0==(timer%500)) {
//			if (SPItxDone == 1) {
//				SPItxDone = 0;
//				testarray[0] = (float)(testcount);
//				testarray[1] = (float)(5*testcount + 1);
//				testarray[2] = (float)(5*testcount + 2);
//				testarray[3] = (float)(5*testcount + 3);
//				testarray[4] = (float)(5*testcount + 4);
//				testcount++;
//				for (i=0;i<5;i++) {
//					f2s.fl = testarray[i];
//					SPItxArray[2*i] = f2s.sh[0];
//					SPItxArray[2*i + 1] = f2s.sh[1];
//				}
//				SPItxCount = 0;
//				SPItxSize = 10;
//
//				IOCLR0 = (1<<EXT_NCS);
//				SPI0command = 0xCC04;
//				S0SPDR = SPI0command;  // send Command to beaglebone to store floats to SDCard
//			} else {
//				numMissedTx++;
//			}
//		}
//	}


	// Code for reading Ultrasonic sensor if 28027 board attached
	if (timer > USONIC_INIT_DELAY) { //wait USONIC_INIT_DELAY ms before talking to 28027
		if (f28027_ready == 1) {
			UsonicTimer++;
			if (firebit == 1) {
				if (UsonicTimer == 5) {
					IOCLR0 = (1<<EXT_NCS);
					SPI0command = 0xCC01;
					S0SPDR = SPI0command;  // send Start Measurement Usonic Command
				}
			} else if (UsonicTimer == 75) {
				IOCLR0 = (1<<EXT_NCS);
				SPI0command = 0xCC02;
				S0SPDR = SPI0command;  // send Read Measurement Usonic Command
			}
		} else {
			IOCLR0 = (1<<EXT_NCS);
			SPI0command = 0xCC03;
			S0SPDR = SPI0command;  // send Are you Alive command
		}
	}

	timer++;

	my_sdkloop_counter++;

// End Add by Dan Block

}


// Dan Block Add
//void filter_ultrasound_z(void) {
//	// take finite difference to estimate z velocity
//	float dt = ((float)(my_sdkloop_counter - ultrasound_timer))/1000.0;
//    float alpha = 0.95;
//	ultrasound_z = (float)UsonicData/100.0;
//	if (ultrasound_z_prev > 0.0) {
//        ultrasound_z = alpha * ultrasound_z + (1-alpha)* ultrasound_z_prev;
//		ultrasound_vz = (ultrasound_z - ultrasound_z_prev)/dt;
//	}
//
//	g_state.dZ = ultrasound_z;
//	g_state.dVz = ultrasound_vz;
//
//	ultrasound_z_prev = ultrasound_z;
//	ultrasound_timer = my_sdkloop_counter;
//}
// End Dan Block Add




/*
 *
 * Sets emergency mode on LowLevel processor. Select one of the EM_ defines as mode option. See EM_ defines for details
 */

void SDK_SetEmergencyMode(unsigned char mode) {
	if ((mode != EM_SAVE_EXTENDED_WAITING_TIME) && (mode != EM_SAVE) && (mode
			!= EM_RETURN_AT_MISSION_SUMMIT) && (mode
			!= EM_RETURN_AT_PREDEFINED_HEIGHT))
		return;
	emergencyMode = mode;
	emergencyModeUpdate = 1;
}

/*
 * the following example shows the direct motor command usage by mapping the stick directly to the motor outputs (do NOT try to fly ;-) )
 */
void SDK_EXAMPLE_direct_individual_motor_commands(void)
{

	WO_SDK.ctrl_mode=0x00;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	WO_SDK.disable_motor_onoff_by_stick=0;

	unsigned int i;

	//scale throttle stick to [0..200] and map it to all motors
	WO_Direct_Individual_Motor_Control.motor[0]=RO_ALL_Data.channel[2]/21;
	WO_Direct_Individual_Motor_Control.motor[1]=RO_ALL_Data.channel[2]/21;
	WO_Direct_Individual_Motor_Control.motor[2]=RO_ALL_Data.channel[2]/21;
	WO_Direct_Individual_Motor_Control.motor[3]=RO_ALL_Data.channel[2]/21;
	WO_Direct_Individual_Motor_Control.motor[4]=RO_ALL_Data.channel[2]/21;
	WO_Direct_Individual_Motor_Control.motor[5]=RO_ALL_Data.channel[2]/21;

	if (RO_ALL_Data.channel[6]>2500)
	{
		WO_Direct_Individual_Motor_Control.motorReverseMask=0x01; //invert motor 0 if AUX switch is enabled
		//limit inverted speed (IMPORTANT! THIS IS NOT DONE AUTOMATICALLY!)
		if (WO_Direct_Individual_Motor_Control.motor[0]>80)
			WO_Direct_Individual_Motor_Control.motor[0]=80;
	}else
		WO_Direct_Individual_Motor_Control.motorReverseMask=0x00;


	//make sure commands are never 0 so that motors will always keep spinning
    //also make sure that commands stay within range
    for(i=0;i<6;i++)
    {
    	if(!WO_Direct_Individual_Motor_Control.motor[i]) WO_Direct_Individual_Motor_Control.motor[i]=1;
    	else if (WO_Direct_Individual_Motor_Control.motor[i]>200) WO_Direct_Individual_Motor_Control.motor[i]=200;
    }
}


void SDK_EXAMPLE_direct_motor_commands_with_standard_output_mapping(void)
{
	WO_SDK.ctrl_mode=0x01;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	/*
	 *  Stick commands directly mapped to motors, NO attitude control! Do NOT try to fly!
	 * */

	WO_Direct_Motor_Control.pitch=(4095-RO_ALL_Data.channel[0])/21;
	WO_Direct_Motor_Control.roll=RO_ALL_Data.channel[1]/21;
	WO_Direct_Motor_Control.thrust=RO_ALL_Data.channel[2]/21;
	WO_Direct_Motor_Control.yaw=(4095-RO_ALL_Data.channel[3])/21;

}


void SDK_EXAMPLE_attitude_commands(void)
{
	WO_SDK.ctrl_mode=0x02;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	//with this example the UAV will go to ~10% throttle when SDK control is activated
	WO_CTRL_Input.ctrl=0x08;	//0x08: enable throttle control by HL. Height control and GPS are deactivated!!
								//pitch, roll and yaw are still commanded via the remote control

	WO_CTRL_Input.thrust=400;	//10% throttle command


}



/* This function demonstrates a simple waypoint command generation. The switch on Channel 7 is used
 * to activate a 15m by 15m square. Therefore a waypoint is calculated from the current position and
 * height and is transmitted to the low level processor. The waypoint status is monitored to switch to
 * the next waypoint after the current one is reached.
 *
 * wpCtrlWpCmd is used to send a command to the low level processor. Different options like waypoint, launch, land, come home, set home
 * are available. See LL_HL_comm.h for WP_CMD_* defines
 *
 * wpCtrlWpCmdUpdated has to be set to 1 to send the command. When the cmd is sent it is set back to 0 automatically
 *
 * wpCtrlAckTrigger is set to 1 when the LL accepts the waypoint
 *
 * wpCtrlNavStatus gives you a navigation status. See WP_NAVSTAT_* defines in SDK.h for options
 *
 * wpCtrlDistToWp gives you the current distance to the current waypoint in dm (= 10 cm)
 */
void SDK_EXAMPLE_gps_waypoint_control()
{
	static unsigned char wpExampleState=0;
	static double originLat,originLon;


	WO_SDK.ctrl_mode=0x03;

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	switch (wpExampleState)
	{
		//prior to start, the lever on channel 7 has to be in "OFF" position
		case 0:
		if ((RO_ALL_Data.channel[6]<1600) || (wpExampleActive))
			wpExampleState=1;
		break;

		case 1:
		if ((RO_ALL_Data.channel[6]>2400) || (wpExampleActive))
		{
			double lat,lon;
			//lever was set to "ON" state -> calculate and send first waypoint and switch state

			//fill waypoint structure
			wpToLL.max_speed=100;
			wpToLL.pos_acc=3000; 	//3m accuracy
			wpToLL.time=400; 		//4 seconds waiting time at each waypoint
			wpToLL.wp_activated=1;

			//see LL_HL_comm.h for WPPROP defines
			wpToLL.properties=WPPROP_ABSCOORDS|WPPROP_AUTOMATICGOTO|WPPROP_HEIGHTENABLED|WPPROP_YAWENABLED;

			//use current height and yaw
			wpToLL.yaw=RO_ALL_Data.angle_yaw; //use current yaw
			wpToLL.height=RO_ALL_Data.fusion_height; //use current height

			originLat=(double)GPS_Data.latitude/10000000.0;
			originLon=(double)GPS_Data.longitude/10000000.0;

			//calculate a position 15m north of us
			xy2latlon(originLat,originLon,0.0,15.0,&lat,&lon);

			wpToLL.X=lon*10000000;
			wpToLL.Y=lat*10000000;

			//calc chksum
			wpToLL.chksum = 0xAAAA
									+ wpToLL.yaw
									+ wpToLL.height
									+ wpToLL.time
									+ wpToLL.X
									+ wpToLL.Y
									+ wpToLL.max_speed
									+ wpToLL.pos_acc
									+ wpToLL.properties
									+ wpToLL.wp_activated;

			//send waypoint
			wpCtrlAckTrigger=0;
			wpCtrlWpCmd=WP_CMD_SINGLE_WP;
			wpCtrlWpCmdUpdated=1;
			wpExampleWpNr=0;
			wpExampleState=2;

		}
		break;

		case 2:
			//wait until cmd is processed and sent to LL processor
			if ((wpCtrlWpCmdUpdated==0) && (wpCtrlAckTrigger))
			{
				//check if waypoint was reached and wait time is over
				if (wpCtrlNavStatus&(WP_NAVSTAT_REACHED_POS_TIME))
				{
					//new waypoint
					double lat,lon;

					//fill waypoint structure
					wpToLL.max_speed=100;
					wpToLL.pos_acc=3000; //3m accuracy
					wpToLL.time=400; //4 seconds wait time
					wpToLL.wp_activated=1;

					//see LL_HL_comm.h for WPPROP defines
					wpToLL.properties=WPPROP_ABSCOORDS|WPPROP_AUTOMATICGOTO|WPPROP_HEIGHTENABLED|WPPROP_YAWENABLED;

					//use current height and yaw
					wpToLL.yaw=RO_ALL_Data.angle_yaw; //use current yaw
					wpToLL.height=RO_ALL_Data.fusion_height; //use current height

					//calculate a position 15m north and 15m east of origin
					xy2latlon(originLat,originLon,15.0,15.0,&lat,&lon);

					wpToLL.X=lon*10000000;
					wpToLL.Y=lat*10000000;

					//calc chksum
					wpToLL.chksum = 0xAAAA
											+ wpToLL.yaw
											+ wpToLL.height
											+ wpToLL.time
											+ wpToLL.X
											+ wpToLL.Y
											+ wpToLL.max_speed
											+ wpToLL.pos_acc
											+ wpToLL.properties
											+ wpToLL.wp_activated;
					//send waypoint
					wpCtrlAckTrigger=0;
					wpCtrlWpCmd=WP_CMD_SINGLE_WP;
					wpCtrlWpCmdUpdated=1;
					wpExampleWpNr++;

					wpExampleState=3;
				}

				if (wpCtrlNavStatus&WP_NAVSTAT_PILOT_ABORT)
				{
					wpExampleActive=0;
					wpExampleState=0;
				}


			}
			if ((RO_ALL_Data.channel[6]<1600) && (wpExampleActive==0))
						wpExampleState=0;
		break;

		case 3:
			//wait until cmd is processed and sent to LL processor
			if ((wpCtrlWpCmdUpdated==0) && (wpCtrlAckTrigger))
			{
				//check if waypoint was reached and wait time is over
				if (wpCtrlNavStatus&(WP_NAVSTAT_REACHED_POS_TIME))
				{
					//new waypoint
					double lat,lon;

					//fill waypoint structure
					wpToLL.max_speed=100;
					wpToLL.pos_acc=3000; //3m accuracy
					wpToLL.time=400; //4 seconds wait time
					wpToLL.wp_activated=1;

					//see LL_HL_comm.h for WPPROP defines
					wpToLL.properties=WPPROP_ABSCOORDS|WPPROP_AUTOMATICGOTO|WPPROP_HEIGHTENABLED|WPPROP_YAWENABLED;

					//use current height and yaw
					wpToLL.yaw=RO_ALL_Data.angle_yaw; //use current yaw
					wpToLL.height=RO_ALL_Data.fusion_height; //use current height

					//calculate a position 15m east of origin
					xy2latlon(originLat,originLon,15.0,0.0,&lat,&lon);

					wpToLL.X=lon*10000000;
					wpToLL.Y=lat*10000000;

					//calc chksum
					wpToLL.chksum = 0xAAAA
											+ wpToLL.yaw
											+ wpToLL.height
											+ wpToLL.time
											+ wpToLL.X
											+ wpToLL.Y
											+ wpToLL.max_speed
											+ wpToLL.pos_acc
											+ wpToLL.properties
											+ wpToLL.wp_activated;

					//send waypoint
					wpCtrlAckTrigger=0;
					wpCtrlWpCmd=WP_CMD_SINGLE_WP;
					wpCtrlWpCmdUpdated=1;
					wpExampleWpNr++;

					wpExampleState=4;
				}

				if (wpCtrlNavStatus&WP_NAVSTAT_PILOT_ABORT)
				{
					wpExampleActive=0;
					wpExampleState=0;
				}


			}
			if ((RO_ALL_Data.channel[6]<1600) && (wpExampleActive==0))
						wpExampleState=0;
		break;

		case 4:
			//wait until cmd is processed and sent to LL processor
			if ((wpCtrlWpCmdUpdated==0) && (wpCtrlAckTrigger))
			{
				//check if waypoint was reached and wait time is over
				if (wpCtrlNavStatus&(WP_NAVSTAT_REACHED_POS_TIME))
				{

					//fill waypoint structure
					wpToLL.max_speed=100;
					wpToLL.pos_acc=3000; //3m accuracy
					wpToLL.time=400; //4 seconds wait time
					wpToLL.wp_activated=1;

					//see LL_HL_comm.h for WPPROP defines
					wpToLL.properties=WPPROP_ABSCOORDS|WPPROP_AUTOMATICGOTO|WPPROP_HEIGHTENABLED|WPPROP_YAWENABLED;

					//use current height and yaw
					wpToLL.yaw=RO_ALL_Data.angle_yaw; //use current yaw
					wpToLL.height=RO_ALL_Data.fusion_height; //use current height

					//go to the beginning

					wpToLL.X=originLon*10000000;
					wpToLL.Y=originLat*10000000;

					//calc chksum
					wpToLL.chksum = 0xAAAA
											+ wpToLL.yaw
											+ wpToLL.height
											+ wpToLL.time
											+ wpToLL.X
											+ wpToLL.Y
											+ wpToLL.max_speed
											+ wpToLL.pos_acc
											+ wpToLL.properties
											+ wpToLL.wp_activated;

					//send waypoint
					wpCtrlAckTrigger=0;
					wpCtrlWpCmd=WP_CMD_SINGLE_WP;
					wpCtrlWpCmdUpdated=1;

					wpExampleWpNr++;

					wpExampleState=0;
					wpExampleActive=0;
				}

				if (wpCtrlNavStatus&WP_NAVSTAT_PILOT_ABORT)
				{
					wpExampleActive=0;
					wpExampleState=0;
				}


			}
			if ((RO_ALL_Data.channel[6]<1600) && (wpExampleActive==0))
						wpExampleState=0;
		break;

		default:
			wpExampleState=0;
		break;
	}

}

int SDK_EXAMPLE_turn_motors_on(void) //hold throttle stick down and yaw stick fully left to turn motors on
{
	static int timeout=0;

	WO_SDK.ctrl_mode=0x02;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	WO_SDK.disable_motor_onoff_by_stick=0; //make sure stick command is accepted

	if(++timeout>=1000)
	{
		timeout=0;
		return(1); //1 => start sequence completed => motors running => user can stop calling this function
	}
	else if(timeout>500) //neutral stick command for 500 ms
	{
		WO_CTRL_Input.ctrl=0x0C;	//0x0C: enable throttle control and yaw control
		WO_CTRL_Input.thrust=0;
		WO_CTRL_Input.yaw=0;
		return(0);
	}
	else //hold stick command for 500 ms
	{
		WO_CTRL_Input.ctrl=0x0C;	//0x0C: enable throttle control and yaw control
		WO_CTRL_Input.thrust=0;
		WO_CTRL_Input.yaw=-2047;
		return(0);
	}

}

int SDK_EXAMPLE_turn_motors_off(void) //hold throttle stick down and yaw stick fully right to turn motors off
{
	static int timeout=0;

	WO_SDK.ctrl_mode=0x02;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	WO_SDK.disable_motor_onoff_by_stick=0; //make sure stick command is accepted

	if(++timeout>=1000)
	{
		timeout=0;
		return(1); //1 => stop sequence completed => motors turned off => user can stop calling this function
	}
	else if(timeout>500) //neutral stick command for 500 ms
	{
		WO_CTRL_Input.ctrl=0x0C;	//0x0C: enable throttle control and yaw control
		WO_CTRL_Input.thrust=0;
		WO_CTRL_Input.yaw=0;
		return(0);
	}
	else //hold stick command for 500 ms
	{
		WO_CTRL_Input.ctrl=0x0C;	//0x0C: enable throttle control and yaw control
		WO_CTRL_Input.thrust=0;
		WO_CTRL_Input.yaw=2047;
		return(0);
	}
}


#ifdef MATLAB

void SDK_matlabMainLoop()
{
	static unsigned short uart_count = 1; //counter for uart communication


		/* put your own c-code here */

		rt_OneStep(); //call RTW function rt_OneStep
		//ctrl_mode is set in rt_one_step

		/* put your own c-code here */


		//don't touch anything below here

		//debug packet handling
		if (uart_count==0 && xbee_send_flag) //call function for uart transmission with 50 Hz
		{
			matlab_debug.cpu_load = HL_Status.cpu_load;
			matlab_debug.battery_voltage = HL_Status.battery_voltage_1;

			UART_Matlab_SendPacket(&matlab_debug, sizeof(matlab_debug), 'd');
		}
		uart_count++;
		uart_count%=ControllerCyclesPerSecond/50;

		//save parameters only while not flying
		if ((!RO_ALL_Data.flying) && (triggerSaveMatlabParams))
		{
			triggerSaveMatlabParams=0;
			lpc_aci_SavePara();
			lpc_aci_WriteParatoFlash();
		}

}


#endif
