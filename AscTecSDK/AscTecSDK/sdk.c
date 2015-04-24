/*

AscTec AutoPilot HL SDK v2.0

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
#include "uart.h"
#include "mymath.h"
#include <stdio.h>
//#include "utils.h"
#include "LPC214x.h"
#include "hardware.h"
#include "irq.h"
#include "lab.h"

// Added by Dan Block
#define USONIC_INIT_DELAY 2000
#define SWITCH_INIT_DELAY 2000
#define SDCARD_START_DELAY 4000
// End Add

struct WO_SDK_STRUCT WO_SDK;
struct WO_CTRL_INPUT WO_CTRL_Input;
struct RO_RC_DATA RO_RC_Data;
struct RO_ALL_DATA RO_ALL_Data;
struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;
struct WO_DIRECT_INDIVIDUAL_MOTOR_CONTROL WO_Direct_Individual_Motor_Control;

void SDK_EXAMPLE_direct_individual_motor_commands(void);
void SDK_EXAMPLE_direct_motor_commands_with_standard_output_mapping(void);
void SDK_EXAMPLE_attitude_commands(void);
void SDK_EXAMPLE_gps_waypoint_control(void);
int SDK_EXAMPLE_turn_motors_on(void);
int SDK_EXAMPLE_turn_motors_off(void);
void AE483_attitude_commands(void);
void filter_accelerations(void);
void toggleLED0(void);
int myBufPos = 0;
TFmQueue rdQue;  // fmmessage handling queue
float tmpthrust;
float R01[9];
float R10[9];
float accels0[3];
float accels1[3];
float gravity_vec[3] = {0.0, 0.0, 9.81};
float dt = 0.001;
int myLEDstate = 0;
unsigned long my_sdkloop_count = 0;


// Start Added by Dan Block
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

// Added by Dan Block
void SPI0Handler (void) __irq
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
// End Dan Block Added


void SDK_init(void) {

	fmInitMessageQ(&rdQue);

    home_x = 0.0;
    home_y = 0.0;
    home_z = 0.0;
    
    TotalBytesReceived = 0;
    
	// initialize global fmstateobject to all zeros
	g_state.dT = 0.0;
	g_state.dX = 0.0;
	g_state.dY = 0.0;
	g_state.dZ = 0.0;
	g_state.dPhi = 0.0;
	g_state.dTheta = 0.0;
	g_state.dPsi = 0.0;
	g_state.dVx = 0.0;
	g_state.dVy = 0.0;
	g_state.dVz = 0.0;
	g_state.dP = 0.0;
	g_state.dQ = 0.0;
	g_state.dR = 0.0;

	u_outer.roll_desired = 0.0;
	u_outer.pitch_desired = 0.0;
	u_outer.yaw_desired = 0.0;
	u_outer.p_desired = 0.0;
	u_outer.q_desired = 0.0;
	u_outer.r_desired = 0.0;
	u_outer.thrust_desired = 0.01;

	ultrasound_timer = 0;
	ultrasound_z = 0;
	ultrasound_vz = 0;
	ultrasound_z_prev = 0;
	ultrasound_z_error_sum = 0;


    g_pinfo.homeLat = 40.114888978;
    g_pinfo.homeLong = -88.22726329;
    g_pinfo.homeHeight = 0.2;

	my_sdkloop_counter = 0;

}



/* SDK_mainloop(void) is triggered @ 1kHz.
 *
 * RO_(Read Only) data is updated before entering this function
 * and can be read to obtain information for supervision or control
 *
 * WO_(Write Only) data is written to the LL processor after
 * execution of this function.
 *
 * WO_ and RO_ structs are defined in sdk.h
 *
 * The struct RO_ALL_Data (defined in sdk.h)
 * is used to read all sensor data, results of the data fusion
 * and R/C inputs transmitted from the LL-processor. This struct is
 * automatically updated at 1 kHz.
 * */

/* How to flash the high level processor
 *
 * The easiest way to get your code on the high level processor is to use the JTAG-adapter.
 *
 * It needs three steps to get your code on the high level processor.
 * 1. Build your code ("Build Project")
 * 2. Connect your JTAG adapter and start the JTAG connection ("OpenOCD Asctec-JTAG")
 * 3. Flash the processor ("Asctec JTAG Debug")
 *
 * In the menu "Run -> External Tools -> External Tools Configuration..." you
 * will find "OpenOCD Asctec-JTAG". If the JTAG connection was activated
 * correctly, the console will show only the following line:
 * "Info:    openocd.c:92 main(): Open On-Chip Debugger (2007-08-10 22:30 CEST)"
 *
 * Do not launch more than ONE JTAG-connection at the same time!
 *
 * In the menu "Run -> Debug Configurations..." you will find "Asctec JTAG Debug"
 * If the code was successfully flashed on the processor, the program will switch
 * to the Debug window.
 *************
 * If you want to flash the high level processor using a serial interface (AscTec USB adapter)
 * and bootloader software like "Flash Magic", you need to change
 * the following in the "makefile" (line 113):
 *
 * FORMAT = ihex
 * #FORMAT = binary
 *
 * After buidling your code you will find the main.hex in your workspace folder.
 *************
 * */

/* After flashing the HL, your code can be debugged online. The ARM7 supports ONE hardware breakpoint.
 * You can monitor the CPU-load by looking at HL_Status.cpu_load. As long as this value is 1000 your
 * code in SDK_mainloop() is executed at 1 kHz.
 *
 * To activate the SDK controls, the serial interface switch on your R/C (channel 5) needs to be in ON position.
 *
 * If your project needs communication over a serial link, you find an example of how to do so in main.c, line 226.
 * All sample functions for transmitting and receiving data over the UART (HL_serial_0) are in uart.c - you can use
 * these examples to code your own communication state machine.
 */


void SDK_mainloop(void) //write your own code within this function
{

	// Handle incoming communication over xbee
	SDK_HandleComm();

	// update g_sensor and g_rawsensor from IMU_RAWDATA and RO_ALL_Data
	//    NOTE:  I cannot find the IMU_RAWDATA structure which is described in the manual!
//	g_rawsensor.dT = my_sdkloop_counter;  // FIXME !!!
//	g_rawsensor.dP = (3.14159 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_roll;
//	g_rawsensor.dQ = (3.14159 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_nick;
//	g_rawsensor.dR = (3.14159 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_yaw;
//	g_rawsensor.dAx = ((float)IMU_CalcData.acc_x)/10000.0;
//	g_rawsensor.dAy = ((float)IMU_CalcData.acc_y)/10000.0;
//	g_rawsensor.dAz = ((float)IMU_CalcData.acc_z)/10000.0;
//	g_rawsensor.dMagx = (float)IMU_CalcData.Hx;
//	g_rawsensor.dMagy = (float)IMU_CalcData.Hy;
//	g_rawsensor.dMagz = (float)IMU_CalcData.Hz;

	g_sensor.dT = my_sdkloop_counter;  //FIXME!!!
	g_sensor.dPhi = (3.14159 / 180.0) * (((float) RO_ALL_Data.angle_roll) / 1000.0);
	g_sensor.dTheta = (3.14159 / 180.0) * (((float) RO_ALL_Data.angle_pitch) / 1000.0);
	g_sensor.dPsi = angle_diff((3.14159 / 180.0) * (((float) RO_ALL_Data.angle_yaw) / 1000.0), 0);
	//g_sensor.dVx = (float)IMU_CalcData.speed_x; // FIXME!!  integrate accelerations
	//g_sensor.dVy = (float)IMU_CalcData.speed_y; // FIXME!!
	//g_sensor.dVz = (float)IMU_CalcData.speed_z; // FIXME!!
	g_sensor.dP = (3.14159 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_roll;
	g_sensor.dQ = (3.14159 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_nick;
	g_sensor.dR = (3.14159 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_yaw;
	//g_sensor.dAx = 9.81*((float)IMU_CalcData.acc_x_calib)/10000.0; //((float)IMU_CalcData.acc_x)/10000.0;
	//g_sensor.dAy = 9.81*((float)IMU_CalcData.acc_y_calib)/10000.0; //((float)IMU_CalcData.acc_y)/10000.0;
	//g_sensor.dAz = 9.81*((float)IMU_CalcData.acc_z_calib)/10000.0; //((float)IMU_CalcData.acc_z)/10000.0;
	//g_sensor.dHx = (float)IMU_CalcData.Hx;
	//g_sensor.dHy = (float)IMU_CalcData.Hy;
	//g_sensor.dHz = (float)IMU_CalcData.Hz;
	//g_sensor.dPressure = (float)IMU_CalcData.height_reference;

//	g_angles.phi = g_sensor.dPhi; //(3.14159 / 180.0) * (((float) RO_ALL_Data.angle_roll)/ 1000.0);
//	g_angles.theta = g_sensor.dTheta; //(3.14159 / 180.0) * (((float) RO_ALL_Data.angle_pitch)/ 1000.0);
//	g_angles.psi = g_sensor.dPsi; //IMU_CalcData.angle_yaw;
//	euler2quat(&g_angles, &g_q);

	//g_rotor.dT = my_sdkloop_counter; // FIXME!!
	//g_rotor.dW1 = (float)RO_ALL_Data.motor_rpm[0];
	//g_rotor.dW2 = (float)RO_ALL_Data.motor_rpm[1];
	//g_rotor.dW3 = (float)RO_ALL_Data.motor_rpm[2];
	//g_rotor.dW4 = (float)RO_ALL_Data.motor_rpm[3];

	//g_motorinput.dT = my_sdkloop_counter;

	//g_state.dT = my_sdkloop_counter;  //FIXME!!!
	////g_state.dX = (float)RO_ALL_Data.GPS_longitude/10000000.0;
	////g_state.dY = (float)RO_ALL_Data.GPS_latitude/10000000.0;
    ////g_state.dZ = (float)RO_ALL_Data.GPS_height/1000.0;
	//g_state.dPhi = g_sensor.dPhi;
	//g_state.dTheta = g_sensor.dTheta;
	//g_state.dPsi = g_sensor.dPsi;
	//g_state.dVx = (float)RO_ALL_Data.fusion_speed_y/1000.0;
	//g_state.dVy = (float)RO_ALL_Data.fusion_speed_x/1000.0;
	////g_state.dVz = IMU_CalcData.speed_z;
	//g_state.dP = g_sensor.dP; //(3.14159 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_roll;
	//g_state.dQ = g_sensor.dQ; //(3.14159 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_nick;
	//g_state.dR = g_sensor.dR; //(3.14159 / 180.0) * 0.0154 * (float) IMU_CalcData.angvel_yaw;

	//g_gpssensor.latitude = GPS_Data.latitude;
	//g_gpssensor.longitude = GPS_Data.longitude;
	//g_gpssensor.height = GPS_Data.height;
	//g_gpssensor.speed_x = GPS_Data.speed_x;
	//g_gpssensor.speed_y = GPS_Data.speed_y;
	//g_gpssensor.heading = GPS_Data.heading;
	//g_gpssensor.horizontal_accuracy = GPS_Data.horizontal_accuracy;
	//g_gpssensor.vertical_accuracy = GPS_Data.vertical_accuracy;
	//g_gpssensor.speed_accuracy = GPS_Data.speed_accuracy;
	//g_gpssensor.numSV = GPS_Data.numSV;
	//g_gpssensor.status = GPS_Data.status;


/*
// Added By Dan Block
	if (NewUsonicData == 1) {
		USMaxBot_range1 = UsonicData;
		filter_ultrasound_z();
		NewUsonicData = 0;
	}
	if (NewMagnetSwitch == 1) {
		GotMagnet = ~MagnetSwitch & 0x1;
		NewMagnetSwitch = 0;
	}
// End Add
*/

	///// FILTER ACCLERATIONS ////
	//filter_accelerations();

	////////////// Call Lab Code Here !  /////////////
	lab();

// Add by Dan Block
// Code to read magnet switch from attached 28027 board
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
/*
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

// End Add by Dan Block
*/

	// AE483_attitude_commands();


	my_sdkloop_counter++;

}


void filter_ultrasound_z(void) {
	// take finite difference to estimate z velocity
	float dt = ((float)(my_sdkloop_counter - ultrasound_timer))/1000.0;
    float alpha = 0.95;
	ultrasound_z = (float)UsonicData/100.0;
	if (ultrasound_z_prev > 0.0) {
        ultrasound_z = alpha * ultrasound_z + (1-alpha)* ultrasound_z_prev;
		ultrasound_vz = (ultrasound_z - ultrasound_z_prev)/dt;
	}

	g_state.dZ = ultrasound_z;
	g_state.dVz = ultrasound_vz;

	ultrasound_z_prev = ultrasound_z;
	ultrasound_timer = my_sdkloop_counter;
}

void filter_accelerations(void){
	// Integrate accelerations to get velocities in the WORLD frame

	static float alpha = 0.8;

	// ROTATE BODY ACCELERATIONS
	// get rotation matrix
	euler2rotmat(g_sensor.dPhi, g_sensor.dTheta, g_sensor.dPsi, R01);
	matrix_transpose(3,3,R01,R10);
	// smooth accelerations
	accels1[0] = accels1[0] + alpha * (g_sensor.dAx - accels1[0]);
	accels1[1] = accels1[1] + alpha * (g_sensor.dAy - accels1[1]);
	accels1[2] = accels1[2] + alpha * (g_sensor.dAz - accels1[2]);
	matrix_multiply( 3, 3, 1 , R10, accels1, accels0 );

	// COMPENSATE FOR GRAVITY
	matrix_add(3,1,accels0,gravity_vec);

	// INTEGRATE (AND PULL TOWARDS ZERO)
	static float alpha2 = 0.95;
	g_state.dVx = alpha2 * (g_state.dVx + accels0[0] * dt); // + (1-alpha2)*g_state.dVx;
	g_state.dVy = alpha2 * (g_state.dVy + accels0[1] * dt); // + (1-alpha2)*g_state.dVy;
	g_state.dVz = alpha2 * (g_state.dVz + accels0[2] * dt); // + (1-alpha2)*g_state.dVz;


}

void SDK_HandleComm(void)
{
	TFmMessage* pMsg;
	myBufPos = 0;
	uint8 b;

	while ( UartChAvailable()) {
		b = UartGetch();
		myBuf[myBufPos] = b;
		if (myBufPos < 511) {
			myBufPos++;
		}
	}
	TotalBytesReceived += myBufPos;

	if (myBufPos > 0) {
		TFmOptitrackObject optitrackObj;
		TFmInnerloopObject innerloopObj;
		TFmControlObject ctrlObj;
        TFmRemoteSetting remoteSetting;
        TFmCommand fmcmd;
		fmReceiveDataQ(&rdQue, myBuf, myBufPos);
		pMsg = fmPopMessageQ(&rdQue);
		while (pMsg) {
			switch(pMsg->iMsgId) {
			case FMOPTITRACK_ID:
				onOptitrackReceived(fmOptitrackObject_frommessage(pMsg,&optitrackObj));
				break;
			case FMINNERLOOP_ID:
				onInnerloopReceived(fmInnerloopObject_frommessage(pMsg, &innerloopObj));
				break;
			case FMCONTROL_ID:
				onControlReceived(fmControlObject_frommessage(pMsg, &ctrlObj));
				break;
			case FMCONTROL2_ID:
				onControl2Received(fmControlObject2_frommessage(pMsg, &u_outer));
				break;
			case FMREMOTESETTING_ID:
				onRemoteSettingReceived(fmRemoteSetting_frommessage(pMsg, &remoteSetting));
				break;
			case FMCOMMAND_ID:
				onUserCommand(fmCommand_frommessage(pMsg, &fmcmd));
				break;
			}
			pMsg = fmPopMessageQ(&rdQue);
		}
	}
}

void onOptitrackReceived(TFmOptitrackObject* pObj)
{
	// update g_state with position
	g_state.dX = pObj->dX;
	g_state.dY = pObj->dY;
	g_state.dZ = pObj->dZ;
}

void onInnerloopReceived(TFmInnerloopObject* pObj)
{
	toggleLED0();
	u_outer.roll_desired = pObj->dRoll;
	u_outer.pitch_desired = pObj->dPitch;
	u_outer.yaw_desired = pObj->dYaw;
	u_outer.thrust_desired = pObj->dThrust;
}

void onControlReceived(TFmControlObject* pObj)
{
	toggleLED0();
	u_outer.roll_desired = pObj->roll_desired;
	u_outer.pitch_desired = pObj->pitch_desired;
	u_outer.yaw_desired = pObj->yaw_desired;
	u_outer.thrust_desired = pObj->thrust_desired;
}


void onControl2Received(TFmControlObject2* pObj)
{
	toggleLED0();
    // FIXME: double check that the below are already set!
	//u_outer.roll_desired = pObj->roll_desired;
	//u_outer.pitch_desired = pObj->pitch_desired;
	//u_outer.yaw_desired = pObj->yaw_desired;
    //u_outer.p_desired = pObj->p_desired;
    //u_outer.q_desired = pObj->q_desired;
    //u_outer.r_desired = pObj->r_desired;
	//u_outer.thrust_desired = pObj->thrust_desired;
}

void onRemoteSettingReceived(TFmRemoteSetting* pSet) {
    home_x = pSet->home_x;
    home_y = pSet->home_y;
    home_z = pSet->home_z;
}

void onUserCommand(TFmCommand* fmcmd) {
    switch(fmcmd->iType) {
    case FMCMD_SETGPSHOME:
        setGPSHome();
        break;
    };
}

void setGPSHome(void) {
    // set home latitude and longitude in floating point degrees
    g_pinfo.homeLat = (float)((double)RO_ALL_Data.GPS_latitude/10000000.0);
    g_pinfo.homeLong = (float)((double)RO_ALL_Data.GPS_longitude/10000000.0);
    g_pinfo.homeHeight = (float)((double)RO_ALL_Data.GPS_height/1000.0);
}



void AE483_attitude_commands(void)
{
	WO_SDK.ctrl_mode=0x02;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	//with this example the UAV will go to ~10% throttle when SDK control is activated
	WO_CTRL_Input.ctrl= 0x08;	//0x08: enable throttle control by HL. Height control and GPS are deactivated!!
								//pitch, roll and yaw are still commanded via the remote control
//

	//	WO_CTRL_Input.pitch=(4095-RO_ALL_Data.channel[0]);
//	WO_CTRL_Input.roll=RO_ALL_Data.channel[1];
//	WO_CTRL_Input.thrust=RO_ALL_Data.channel[2];
//	WO_CTRL_Input.yaw=(4095-RO_ALL_Data.channel[3]);


//	tmpthrust = ((float)RO_ALL_Data.channel[2]);

//	WO_Direct_Motor_Control.pitch=(4095-RO_ALL_Data.channel[0])/21;
//	WO_Direct_Motor_Control.roll=RO_ALL_Data.channel[1]/21;\
//	WO_CTRL_Input.thrust = RO_ALL_Data.channel[2];
	float M=0.4;
	float X1=20;
	WO_CTRL_Input.thrust = throttle_scaling(RO_ALL_Data.channel[2],M,X1);
//	WO_Direct_Motor_Control.yaw=(4095-RO_ALL_Data.channel[3])/21;

}


void toggleLED0(void) {
	if (myLEDstate) {
		LED(0,OFF);
		myLEDstate = 0;
	}
	else {
		LED(0,ON);
		myLEDstate = 1;
	}
}


/* the following example shows the direct motor command usage by mapping the stick directly to the motor outputs (do NOT try to fly ;-) )
 *
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



/* This function demonstrates a simple waypoint command generation. Switch on Channel 7 is used
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
		if (RO_RC_Data.channel[6]<1600)
			wpExampleState=1;
		break;

		case 1:
		if (RO_RC_Data.channel[6]>2400)
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
			wpToLL.yaw=IMU_CalcData.angle_yaw; //use current yaw
			wpToLL.height=IMU_CalcData.height; //use current height

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
					wpToLL.yaw=IMU_CalcData.angle_yaw; //use current yaw
					wpToLL.height=IMU_CalcData.height; //use current height

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

					wpExampleState=3;
				}

				if (wpCtrlNavStatus&WP_NAVSTAT_PILOT_ABORT)
					wpExampleState=0;


			}
			if (RO_RC_Data.channel[6]<1600)
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
					wpToLL.yaw=IMU_CalcData.angle_yaw; //use current yaw
					wpToLL.height=IMU_CalcData.height; //use current height

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

					wpExampleState=4;
				}

				if (wpCtrlNavStatus&WP_NAVSTAT_PILOT_ABORT)
					wpExampleState=0;


			}
			if (RO_RC_Data.channel[6]<1600)
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
					wpToLL.yaw=IMU_CalcData.angle_yaw; //use current yaw
					wpToLL.height=IMU_CalcData.height; //use current height

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

					wpExampleState=0;
				}

				if (wpCtrlNavStatus&WP_NAVSTAT_PILOT_ABORT)
					wpExampleState=0;


			}
			if (RO_RC_Data.channel[6]<1600)
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
		WO_CTRL_Input.thrust=0;	//use R/C throttle stick input /2 to control thrust (just for testing)
		WO_CTRL_Input.yaw=0;
		return(0);
	}
	else //hold stick command for 500 ms
	{
		WO_CTRL_Input.ctrl=0x0C;	//0x0C: enable throttle control and yaw control
		WO_CTRL_Input.thrust=0;	//use R/C throttle stick input /2 to control thrust (just for testing)
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
		WO_CTRL_Input.thrust=0;	//use R/C throttle stick input /2 to control thrust (just for testing)
		WO_CTRL_Input.yaw=0;
		return(0);
	}
	else //hold stick command for 500 ms
	{
		WO_CTRL_Input.ctrl=0x0C;	//0x0C: enable throttle control and yaw control
		WO_CTRL_Input.thrust=0;	//use R/C throttle stick input /2 to control thrust (just for testing)
		WO_CTRL_Input.yaw=2047;
		return(0);
	}
}


