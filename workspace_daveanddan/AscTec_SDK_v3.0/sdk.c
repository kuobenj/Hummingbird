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
#include "lab.h"
#include "Utilites.h"

#ifdef MATLAB
#include "..\custom_mdl\onboard_matlab_ert_rtw\onboard_matlab.h"
#endif

struct WO_SDK_STRUCT WO_SDK;
struct WO_CTRL_INPUT WO_CTRL_Input;
struct RO_RC_DATA RO_RC_Data;
struct RO_ALL_DATA RO_ALL_Data;
struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;
struct WO_DIRECT_INDIVIDUAL_MOTOR_CONTROL WO_Direct_Individual_Motor_Control;
struct imuSensor imusensor;

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


/*----------------------------------------------------------------------*/
/*------------------------- Initialization -----------------------------*/
/*----------------------------------------------------------------------*/
void SDK_Init(void) { 	// Used in lab.c
	// initialize forces and torques structure
	u.u1 = 0.0;
	u.u2 = 0.0;
	u.u3 = 0.0;
	float mass = 1.9; // kg	
	u.u4 = mass*9.81;
}
/*----------------------------------------------------------------------*/
/*----------------------- End Initialization ---------------------------*/
/*----------------------------------------------------------------------*/



void SDK_mainloop(void)
{
//#ifdef MATLAB
	//SDK_matlabMainLoop(); //this runs only in combination with the AscTec Simulink Toolkit

	//jeti telemetry can always be activated. You may deactivate this call if you don't have the AscTec Telemetry package.
	//SDK_jetiAscTecExampleRun();

//#else //write your own C-cod e within this function

	imusensor.dT = (float) RO_ALL_Data.flight_time;
	imusensor.dThetax = (3.14159/180.0)*(((float) RO_ALL_Data.angle_roll) / 1000.0);
	imusensor.dThetay = (3.14159/180.0)*(((float) RO_ALL_Data.angle_pitch) / 1000.0);
	imusensor.dThetaz = angle_diff((3.14159 / 180.0) * (((float) RO_ALL_Data.angle_yaw) / 1000.0),0);
	imusensor.dOmegax = (3.1415 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_roll;
	imusensor.dOmegay = (3.1415 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_nick;
	imusensor.dOmegaz = (3.1415 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_yaw;

	lab();

	//SDK_jetiAscTecExampleRun();

	//if (wpExampleActive) //this is used to activate the waypoint example via the jeti telemetry display
	//	SDK_EXAMPLE_gps_waypoint_control();

//#endif
}


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
