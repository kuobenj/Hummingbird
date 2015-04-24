
#include "lab.h"
#include "math.h"



//extern unsigned long my_sdkloop_count;

void do_communication(void);
void motor_rpm_calibration(void);
void clip_motor_control(float* cmd);
//float motor_calib_inv(float omega_des);
void u_to_rotorspeed(float* u, float* omega_cmd);
void motor_omega_to_cmd(float* omega_des, float* cmd);
void AE483_send_control(void);
void lab1(void);
void lab2(void);
void lab3(void);
void mm_check(void);
void potentialfield(void);
void GPSHold(void);
void ExternalOuterLoop(void);
void ExternalOuterLoopRate(void);
void ExternalOuterLoopAngleRate(void);
void OuterLoopGPSUltrasound(float x_desired, float y_desired, float z_desired, float* roll_desired, float* pitch_desired, float* yaw_desired);
void InnerLoop(float roll_desired, float pitch_desired, float yaw_desired);
void InnerLoopRate(float p_desired, float q_desired, float r_desired);
void InnerLoopAngleRate(float roll_desired, float pitch_desired, float yaw_desired, float p_desired, float q_desired, float r_desired);
void flatEarthl2x(void);
void SendMassData(void);
void countData(void);
// Declarations and defaults. You shouldn't have to change these.
unsigned long int output_timer = 0;
unsigned long int output_count = 1;
unsigned long int timer_tic = 10;
unsigned long int data_freq_1 = 1;
unsigned long int data_freq_2 = 40;
unsigned long int data_freq_3 = 60;
unsigned long int data_freq_4 = 80;
unsigned long int data_freq_5 = 100;
unsigned long int max_output_count = 0;
unsigned long int comm_start_count = 0;
unsigned long int my_lab_count = 0;
unsigned long int test_time_limit = 0; // milliseconds
unsigned char DataOutputsPerSecond = 1;  // frequency at which we send messages to ground station.
unsigned int outer_loop_timer = 0;
unsigned int trigger_stop = 0;
unsigned char OuterLoopFrequency = 50;  // frequency at which we call the OuterLoop().
unsigned int msg_flag = 0;  // flag that determines what objects get communicated over XBee.
unsigned int trigger_flag = 0;
unsigned int total_check_current = 0;
unsigned int total_check_previous = 0;
float state[2];
float K[2] = {0.32, 0.059};
float K3[12*4] = { -0.0000,    -0.0172,   0.0000,   0.0000,    -0.0321,   0.0000,    -0.2104,    0.0000,    0.0000,    -0.0677,    0.0000,    0.0000,  0.0172,    0.0000,    0.0000,   0.0321,   -0.0000,    0.0000,   -0.0000,    -0.2104,   -0.0000,   -0.0000,    -0.0677,   -0.0000,  0.0000,   0.0000,   -0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,   -0.0000,    -0.0725,    0.0000,   -0.0000,    -0.0797,   0.0000,   -0.0000,   0.3087,    0.0000,    0.0000,   0.6535,    0.0000,   -0.0000,    0.0000,    0.0000,   -0.0000,    0.0000};
float k_p = 0.0;
float k_d = 0.0;
float k_i = 0.0;
float nom[2];
float e[2] = {0,0};
float g_u[4];
float g_xerr[12];
float g_xnom[12] = {0,0,-1,0,0,0,0,0,0,0,0,0};
float omega_hover = 488.2;
float g_omega_cmd[4];
float g_cmd[4];
float W[4*4];
float R01[3*3];
float R10[3*3];
float R01_des[3*3];
float R10_des[3*3];
float tmpprod1[3*3];
float tmpprod2[3*3];
float e_R[3];
float e_w[3]; 
//////////// HUMMINGBIRD ///////////
float kF = 6.7e-6;
float kM = 1.7e-7;
/////////// PELICAN /////////////////
//float kF = 1.56788e-5;
//float kM = 2.453305e-7;
/////////////////////////////////////
float l = 0.17;
float Winv[4*4];
//float homeLat = 40.1148887;
//loat homeLong = -88.2272284;
//float homeLat = 40.1149701;
//float homeLong = -88.2269798;
//float desZ = 0.2;

void lab(void) {

    // Use message flag set in Lab 1.
    //lab1();

    // Use controller set in Lab 2. PLEASE COMMENT OUT AE483_attitude_commands() in sdk.c
    //lab2();

    // Use controller from lab 3.
    //lab3();
    //mm_check();

    //constant_altitude_control();
    // Ultrasound altitude hover (no x,y position control)
    //ultrasound_control();

    // THIS IS ONLY NEEDED FOR THE THRUST AND TORQUE TEST STAND
    //motor_rpm_calibration();
    
    // JUST RUN INNER LOOP, ASSUMING WE RECEIVE DESIRED ANGLES OVER MESSAGES
    ExternalOuterLoop();

    // JUST RUN INNER LOOP, ASSUMING WE RECEIVE DESIRED ANGULAR VELOCITY OVER MESSAGES
    // ExternalOuterLoopRate();

    // ANGLE AND ANGULAR VELOCITY TRACKING FROM Mellinger2011
    // ExternalOuterLoopAngleRate();

    //GPS pos hold
    //outerloopGPS();
    //GPSHold();
    //SendMassData();
    //Potential Field
    //potentialfield();
    //getData();
    //countData();
    // Call function which sends messages to Control PC
    do_communication();

    //my_lab_count++;



}
// added by Vishwa for testing
void SendMassData(void) { //{{{1
    // msg_flag = 1<<FMSTATUS_ID;
    // msg_flag = 1<<FMSTATE_ID;
    msg_flag = 1<<FMSENSOR_ID;

    

    // if (output_count <= (timer_tic*data_freq_1)){
    //     DataOutputsPerSecond = data_freq_1;
    // }
    // if (output_count > (timer_tic*data_freq_1) && output_count <= timer_tic*data_freq_2)
    // {
    //     DataOutputsPerSecond = data_freq_2;
    // }
    // if (output_count > (timer_tic*data_freq_2) && output_count <= timer_tic*data_freq_3)
    // {
    //     DataOutputsPerSecond = data_freq_3;
    // }
    // if (output_count > (timer_tic*data_freq_3) && output_count <= timer_tic*data_freq_4)
    // {
    //     DataOutputsPerSecond = data_freq_4;
    // }
    // if (output_count > (timer_tic*data_freq_4) && output_count <= timer_tic*data_freq_5)
    // {
    //     DataOutputsPerSecond = data_freq_5;
    // }
    // max_output_count = (timer_tic)*(data_freq_1 + data_freq_2 + data_freq_3 + data_freq_4 + data_freq_5);
    // if (output_count == timer_tic*data_freq_1 | output_count == timer_tic*data_freq_2 | output_count == timer_tic*data_freq_3 | output_count == timer_tic*data_freq_4 | output_count == timer_tic*data_freq_5){
    //     my_lab_count = 0;
    // }
    
    DataOutputsPerSecond = 50;
    // States Variables

    // g_state.dT = 0;
    // g_state.dX = 0;  
    // g_state.dY = 0;  
    // g_state.dZ = 0;    
    // g_state.dVx = 0;   
    // g_state.dVy = 0;   
    // g_state.dVz = 0;   
    // g_state.dPhi = 0;   
    // g_state.dTheta = 0; 
    // g_state.dPsi = 0; 
    // g_state.dP = 0; 
    // g_state.dQ = 0; 
    // g_state.dR = 0;

    // Sensor Variables

    g_sensor.dT = 0;
    g_sensor.dPhi = 0;
    g_sensor.dPsi = 0;
    g_sensor.dTheta = 0;
    g_sensor.dVx = 0;
    g_sensor.dVy = 0;
    g_sensor.dVz = 0;
    g_sensor.dP = 0;
    g_sensor.dQ = 0;
    g_sensor.dR = 0;
    g_sensor.dAx = 0;
    g_sensor.dAy = 0;
    g_sensor.dAz = 0;
    g_sensor.dHx = 0;
    g_sensor.dHy = 0;
    g_sensor.dHz = 0;
    g_sensor.dPressure = 0;

    // Status Variables

    // g_status.iType = 0;
    // g_status.iStatus = 0;
    // g_status.iStatus2 = 0;

}
// added for testing by Vishwa

void countData(void){ //{{{1
    // msg_flag = 1 << FMSTATUS_ID;
    // DataOutputsPerSecond = 1;
    TFmMessage msgObj;
    /*if (my_lab_count%2 == 0){total_check_current = TotalBytesReceived;}
    else {total_check_previous = TotalBytesReceived;}
    if (total_check_previous == total_check_current)
    {
        trigger_flag = 1;
    }
    if (((trigger_flag == 1) || (my_lab_count >= test_time_limit)) && trigger_stop == 0)
    {   g_status.iType = 0;
        g_status.iStatus = 0;
        g_status.iStatus2 = TotalBytesReceived;
        trigger_stop = 1;
        fmStatusObject_tomessage(&g_status, &msgObj);
        UART_SendMsg(msgObj.data, msgObj.iMsgSize); 
    }*/

    test_time_limit = (unsigned long int)((max_output_count/DataOutputsPerSecond)*1000 + comm_start_count + 7000);
    if ((my_lab_count >= test_time_limit) && trigger_stop == 0)
    {   g_status.iType = 0;
        g_status.iStatus = 0;
        g_status.iStatus2 = TotalBytesReceived;
        trigger_stop = 1;
        fmStatusObject_tomessage(&g_status, &msgObj);
        UART_SendMsg(msgObj.data, msgObj.iMsgSize); 
    }
}

void ExternalOuterLoop(void) { //{{{1
    DataOutputsPerSecond = 5;
    //msg_flag = (1<<FMGPSSENSOR_ID) | (1<<FMSTATE_ID) | (1<<FMMOTORINPUT_ID) | (1<<FMPLANNERINFO_ID);
    //msg_flag = (1<<FMSTATE_ID); //| (1<<FMMOTORINPUT_ID);
    msg_flag = 0;

    // PASS THROUGH DESIRED THRUST
    g_u[3] = u_outer.thrust_desired;

    // CALL INNER LOOP
    InnerLoop(u_outer.roll_desired, u_outer.pitch_desired, u_outer.yaw_desired);

    // SEND CONTROL TO MOTORS
    AE483_send_control();
}

void ExternalOuterLoopRate(void) { //{{{1
    DataOutputsPerSecond = 10;
    //msg_flag = (1<<FMGPSSENSOR_ID) | (1<<FMSTATE_ID) | (1<<FMMOTORINPUT_ID) | (1<<FMPLANNERINFO_ID);
    //msg_flag = (1<<FMSTATE_ID) | (1<<FMMOTORINPUT_ID);
    msg_flag = (1<<FMSENSOR_ID); 

    // PASS THROUGH DESIRED THRUST
    g_u[3] = u_outer.thrust_desired;

    // CALL INNER LOOP
    InnerLoopRate(u_outer.roll_desired, u_outer.pitch_desired, u_outer.yaw_desired);

    // SEND CONTROL TO MOTORS
    AE483_send_control();
}


void ExternalOuterLoopAngleRate(void) { //{{{1 Angle and Ang.Vel. tracking Mellinger2011
    DataOutputsPerSecond = 5;
    msg_flag = 0;

    // PASS THROUGH DESIRED THRUST
    g_u[3] = u_outer.thrust_desired;

    // CALL INNER LOOP
    InnerLoopAngleRate(u_outer.roll_desired, u_outer.pitch_desired, u_outer.yaw_desired,
                       u_outer.p_desired, u_outer.q_desired, u_outer.r_desired);

    // SEND CONTROL TO MOTORS
    AE483_send_control();
}



void GPSHold(void) { //{{{1

    DataOutputsPerSecond = 5;
    msg_flag = (1<<FMGPSSENSOR_ID) | (1<<FMSTATE_ID) | (1<<FMMOTORINPUT_ID) | (1<<FMPLANNERINFO_ID);

    static float roll_desired = 0.0;
    static float pitch_desired = 0.0;
    static float yaw_desired = 0.0;

    float x_desired = 0.0;
    float y_desired = 0.0;
    float z_desired = 0.2;

    // CALL OUTER LOOP
    if (outer_loop_timer++ == ControllerCyclesPerSecond / OuterLoopFrequency) {
        OuterLoopGPSUltrasound(x_desired, y_desired, z_desired, &roll_desired, &pitch_desired, &yaw_desired);
        outer_loop_timer = 0;
    }

    // CALL INNER LOOP
    InnerLoop(roll_desired, pitch_desired, yaw_desired);

    // SEND CONTROL TO MOTORS
    AE483_send_control();
}


void OuterLoopGPSUltrasound(float x_desired, float y_desired, float z_desired, float* roll_desired, float* pitch_desired, float* yaw_desired) { //{{{1

    //compute xyz from Latitude,Longitude
    flatEarthl2x();  // computes "xpos", "ypos"
    //ll2x();

    /////////////////// POTENTIAL FUNCTION ///////////////
    float kdescent = 50.0;
    float bdescent = 0.3;
    float k_att = 1.0;
    float b_att = 1.0;
    float delta_t = 0.02;
    float grad_fx;
    float grad_fy;

    float x_diff = g_state.dX - x_desired;
    float y_diff = g_state.dY - y_desired;
    //z_diff = (z_pos-z_des);
    float norm_diff = sqrt(pow(x_diff,2.0)+pow(y_diff,2.0));//+pow(z_diff,2.0));
    if (norm_diff <= b_att)
    {
        grad_fx = k_att*x_diff;
        grad_fy = k_att*y_diff;
        //grad_fz = k_att*z_diff;
    }
    else
    {
        grad_fx = k_att*b_att*x_diff/norm_diff;
        grad_fy = k_att*b_att*y_diff/norm_diff;
        //grad_fz = k_att*b_att*z_diff/norm_diff;
    }
    float dq[2];
    dq[0] = -delta_t*kdescent*grad_fx;
    dq[1] = -delta_t*kdescent*grad_fy;
    float dqnorm = sqrt(pow(dq[0],2) + pow(dq[1],2));
    if (dqnorm > bdescent) {
        dq[0] = bdescent * dq[0]/dqnorm;
        dq[1] = bdescent * dq[1]/dqnorm;
    }
    float xnom = g_state.dX + dq[0];
    float ynom = g_state.dY + dq[1];

    
    ////////////////// X Y  CONTROL ///////////////////////
    float Kp1 = 3.8; //0.5; 2.0 was not a good gain choice
    float Kd1 = 2.8;
    float Kp2 = 3.8; // 0.5; 2.0 was not a good gain choice
    float Kd2 = 2.8;

    float ax  = -Kp1*(g_state.dX - xnom) - Kd1*g_state.dVx;
    float ay = -Kp2*(g_state.dY - ynom) - Kd2*g_state.dVy;

    *yaw_desired = 0.0;
    *roll_desired = -(1.0/9.81)*(-ay * cos(*yaw_desired) + ax * sin(*yaw_desired));
    *pitch_desired = -(1.0/9.81) * (ay * sin(*yaw_desired) + ax * cos(*yaw_desired));
    /////////////////// END X,Y CONTROL ///////////////////


    ////////////////// Z  CONTROL ///////////////////////
    float Kpz = 5.3;
    float Kdz = 2.7;
    float Kiz = 0.08;
    float m = 0.69; // kg
    float g = 9.81; // gravity
    float z_error_sum;
    // Compute "integral error"
    if ((z_error_sum < .5) && (z_error_sum > -.5)) {
        z_error_sum += g_state.dZ - z_desired;
    }
    g_u[3] = m*g - Kpz * (g_state.dZ * cos(g_sensor.dPhi) * cos(g_sensor.dTheta) - z_desired) - Kdz * (g_state.dVz) - Kiz * (z_error_sum);
    if (g_u[3] < 0) {
        g_u[3] = 0;
    }
    if (g_u[3] > 10) {
        g_u[3] = 10;
    }
    ////////////////////// END Z CONTROL /////////////////
}


void InnerLoop(float roll_desired, float pitch_desired, float yaw_desired) { //{{{1
    // Attitude inner loops:
    float K00 = 0.88;
    float K01 = 0.13;
    float K10 = 0.88;
    float K11 = 0.13;
    float K20 = 0.31;
    float K21 = 0.12;
    // Attitude control
    g_u[0] = -K00 * (g_sensor.dPhi - roll_desired) - K01 * (g_sensor.dP);
    g_u[1] = -K10 * (g_sensor.dTheta - pitch_desired) - K11 * (g_sensor.dQ);
    g_u[2] = -K20 * (g_sensor.dPsi - yaw_desired) - K21 * (g_sensor.dR);

    // for logging purposes
    g_motorinput.dU1 = g_u[0];
    g_motorinput.dU2 = g_u[1];
    g_motorinput.dU3 = g_u[2];
    g_motorinput.dU4 = g_u[3];
}

void InnerLoopRate(float p_desired, float q_desired, float r_desired) { //{{{1
    // Attitude inner loops:
    float K00 = 0.88;
    float K01 = 0.13;
    float K10 = 0.88;
    float K11 = 0.13;
    float K20 = 0.31;
    float K21 = 0.12;
    // Attitude control
    g_u[0] = -K00 * angle_diff(g_sensor.dPhi,0) - K01 * (g_sensor.dP - p_desired);
    g_u[1] = -K10 * angle_diff(g_sensor.dTheta,0) - K11 * (g_sensor.dQ - q_desired);
    g_u[2] =  - K21 * (g_sensor.dR - r_desired);

    // for logging purposes
    g_motorinput.dU1 = g_u[0];
    g_motorinput.dU2 = g_u[1];
    g_motorinput.dU3 = g_u[2];
    g_motorinput.dU4 = g_u[3];
}


void InnerLoopAngleRate(float roll_desired, float pitch_desired, float yaw_desired,  //{{{1
        float p_desired, float q_desired, float r_desired) { 
    // GAINS
    float KR0 = 1.0857;
    float KR1 = 1.0857;
    float KR2 = 1.0857;
    float Kw0 = 0.1781;
    float Kw1 = 0.1781;
    float Kw2 = 0.1781;

    /*
    // CURRENT ROTATION MATRIX
    euler2rotmat(g_sensor.dPhi, g_sensor.dTheta, g_sensor.dPsi, R01);
    matrix_transpose(3,3,R01,R10);
    // DESIRED ROTATION MATRIX
    euler2rotmat(roll_desired, pitch_desired, yaw_desired, R01_des);
    //matrix_transpose(3,3,R01_des,R10_des);
    // ATTITUDE ERROR
    matrix_multiply(3,3,3,R01_des,R10,tmpprod1);
    //matrix_multiply(3,3,3,R01,R10_des,tmpprod2);
    matrix_transpose(3,3,tmpprod1,tmpprod2);
    //matrix_scalar_mult(3,3, tmpprod2, -1.0); // overwrites tmpprod2
    matrix_subtract(3,3, tmpprod1, tmpprod2); // in place!  result in first arg!
    unwedge(tmpprod1, e_R);    
    matrix_scalar_mult(3,1, e_R, 0.5); // in place!
    */
    e_R[0] = angle_diff(g_sensor.dPhi, roll_desired);
    e_R[1] = angle_diff(g_sensor.dTheta, pitch_desired);
    e_R[2] = angle_diff(g_sensor.dPsi, yaw_desired);

    // ANGULAR VELOCITY ERROR
    e_w[0] = g_sensor.dP - p_desired;
    e_w[1] = g_sensor.dQ - q_desired;
    e_w[2] = g_sensor.dR - r_desired;

    // Attitude control
    g_u[0] = -KR0 * e_R[0] - Kw0 * e_w[0];
    g_u[1] = -KR1 * e_R[1] - Kw1 * e_w[1]; 
    g_u[2] = -KR2 * e_R[2] - Kw2 * e_w[2];

}



//Potential Function: inputs current position and desired position, outputs next step
void outerloopGPS(void) { //{{{1

    DataOutputsPerSecond = 1;
    msg_flag = (1<<FMGPSSENSOR_ID);
    
    //compute xyz
    flatEarthl2x();
    //ll2x();
    //creating control around the next_position
    float Kd1 = 2.8;
    float Kp1 = 0.8; //0.5; 2.0 was not a good gain choice
    float Kd2 = 2.8;
    float Kp2 = 0.8; // 0.5; 2.0 was not a good gain choice
    float desRoll = -(Kp1*(g_state.dX) + Kd1*g_state.dVx)/9.81;
    float desPitch = (Kp2*(g_state.dY) + Kd2*g_state.dVy)/9.81;
    float desYaw = 0.0;
    // Attitude inner loops:
    float K00 = 0.88;
    float K01 = 0.13;
    float K10 = 0.88;
    float K11 = 0.13;
    float K20 = 0.31;
    float K21 = 0.12;
    // Attitude control
    g_u[0] = -K00 * (g_sensor.dPhi - desRoll) - K01 * (g_sensor.dP);
    g_u[1] = -K10 * (g_sensor.dTheta - desPitch) - K11 * (g_sensor.dQ);
    g_u[2] = -K20 * (g_sensor.dPsi - desYaw) - K21 * (g_sensor.dR);
    // Altitude control
    float Kpz = 4.0;
    float Kdz = 2.0;
    float Kiz = 0.05;
    float m = 0.69; // kg
    float g = 9.81; // gravity
    // centimeters from ground
    float z_desired = 0.2;
    float vz_desired = 0.0;
    float z_error_sum;
    // Compute "integral error"
    if ((z_error_sum < .5) && (z_error_sum > -.5)) {
        z_error_sum += g_state.dZ - z_desired;
    }
    g_u[3] = m*g - Kpz * (g_state.dZ * cos(g_sensor.dPhi) * cos(g_sensor.dTheta) - z_desired) - Kdz * (g_state.dVz - vz_desired) - Kiz * (z_error_sum);

    AE483_send_control();

}

// flat earth GPS coord transform
void flatEarthl2x(void) { //{{{1
    float r = 6378100.0;
    double lon = ((double)RO_ALL_Data.GPS_longitude/10000000.0)*3.14159/180.0;
    double lat = ((double)RO_ALL_Data.GPS_latitude/10000000.0)*3.14159/180.0;
    //float lat = g_state.dY*3.14159/180.0;
    //float lon = g_state.dX*3.14159/180.0;
    double lat0 = g_pinfo.homeLat*3.14159/180.0;
    double lon0 = g_pinfo.homeLong*3.14159/180.0;
    g_state.dX = (float)((lon-lon0)*r*cos(lat));
    g_state.dY = (float)(r*(lat-lat0));
}


// Altitude controlled by throttle
// roll, pitch, and yaw are controlled in the usual way
void constant_altitude_control(void) { //{{{1
    DataOutputsPerSecond = 10;
    msg_flag = (1<<FMSTATE_ID) | (1<<FMMOTORINPUT_ID);
    //msg_flag = (1<<FMGPSSENSOR_ID);
    ////////////////////////////////////////////////////////
    ///////// COMPUTE CONTROL VECTOR HERE //////////////////
    float desPitch = (3.14159/180)*((60.0/4096)*(float)(RO_ALL_Data.channel[0])-30);
    float desRoll = (3.14159/180)*((60.0/4096)*(float)(RO_ALL_Data.channel[1])-30);
    //float desYaw = (3.14159/180)*((float)(RO_ALL_Data.channel[3]));
    float desYawRate = (3.14159/180)*((60.0/4096)*(float)(RO_ALL_Data.channel[3])-30.0);
    float desYaw = 0.0;
    //desYaw = desYaw + (3.14159/180)*((4/4095)*RO_ALL_Data.channel[3]-2);
    // Attitude inner loops:
    //float K00 = 0.88;
    //float K01 = 0.13;
    //float K10 = 0.88;
    //float K11 = 0.13;
    //float K20 = 0.31;
    float K00 = 1.0;
    float K01 = 0.2;
    float K10 = 1.0;
    float K11 = 0.2;
    float K20 = 0.0;
    //float K21 = 0.12;
    float K21 = 0.3;

    float K02 = 1.0;
    float K12 = 1.0;

    // compute desired roll pitch rates based on angle error
    float desRollRate = -K02 * (g_sensor.dPhi - desRoll);
    float desPitchRate = -K12 * (g_sensor.dTheta - desPitch);

    // Attitude control
    g_u[0] = -K00 * (g_sensor.dPhi - desRoll) - K01 * (g_sensor.dP - desRollRate);
    g_u[1] = -K10 * (g_sensor.dTheta - desPitch) - K11 * (g_sensor.dQ - desPitchRate);
    g_u[2] = -K20 * (g_sensor.dPsi - desYaw) - K21 * (g_sensor.dR - desYawRate);
    //g_u[0] = 0;
    //g_u[1] = 0;
    //g_u[2] = 0;

    // Altitude control
    float Kpz = 10.0;
    float Kdz = 2.0;
    float Kiz = 0.1;
    //float m = 0.69; // kg
    float m = 2.2; // kg
    float g = 9.81; // gravity
    float z_desired = 0.25; // centimeters from ground
    //float z_desired = 0.2 + (0.4/4096)*(float)RO_ALL_Data.channel[2];
    float vz_desired = 0.0;
    // Compute "integral error"
    if ((ultrasound_z_error_sum < .5) && (ultrasound_z_error_sum > -.5)) {
        ultrasound_z_error_sum += g_state.dZ - z_desired;
    }
    g_u[3] = m*g - Kpz * (g_state.dZ * cos(g_sensor.dPhi) * cos(g_sensor.dTheta) - z_desired) - Kdz * (g_state.dVz - vz_desired) - Kiz * (ultrasound_z_error_sum);
    //g_u[3] = m*g;


    g_motorinput.dU1 = g_u[0];
    g_motorinput.dU2 = g_u[1];
    g_motorinput.dU3 = g_u[2];
    g_motorinput.dU4 = g_u[3];   //6.77 + g_u[3];


    /////////////////////////////////////////////////////////

    AE483_send_control();

}

void ultrasound_control(void) { //{{{1
    DataOutputsPerSecond = 5;
    msg_flag = (1<<FMGPSSENSOR_ID); // | (1<<FMMOTORINPUT_ID);
    ////////////////////////////////////////////////////////
    ///////// COMPUTE CONTROL VECTOR HERE //////////////////

    // Attitude inner loops:
    float K00 = 0.88;
    float K01 = 0.13;
    float K10 = 0.88;
    float K11 = 0.13;
    float K20 = 0.31;
    float K21 = 0.12;
    // Attitude control
    g_u[0] = -K00 * (g_sensor.dPhi - u_outer.roll_desired) - K01 * (g_sensor.dP);
    g_u[1] = -K10 * (g_sensor.dTheta - u_outer.pitch_desired) - K11 * (g_sensor.dQ);
    g_u[2] = -K20 * (g_sensor.dPsi - u_outer.yaw_desired) - K21 * (g_sensor.dR);

    // Altitude control
    float Kpz = 4.0;
    float Kdz = 2.0;
    float Kiz = 0.05;
    float m = 0.69; // kg
    float g = 9.81; // gravity
    float z_desired = 0.2; // centimeters from ground
    float vz_desired = 0.0;
    // Compute "integral error"
    if ((ultrasound_z_error_sum < .5) && (ultrasound_z_error_sum > -.5)) {
        ultrasound_z_error_sum += g_state.dZ - z_desired;
    }
    g_u[3] = m*g - Kpz * (g_state.dZ - z_desired) - Kdz * (g_state.dVz - vz_desired) - Kiz * (ultrasound_z_error_sum);
    //g_u[3] = u_outer.thrust_desired;
    //g_u[3] = 3.0;//7.8;

    g_motorinput.dU1 = g_u[0];
    g_motorinput.dU2 = g_u[1];
    g_motorinput.dU3 = g_u[2];
    g_motorinput.dU4 = g_u[3];   //6.77 + g_u[3];


    /////////////////////////////////////////////////////////

    AE483_send_control();

}



void lab3(void) { //{{{1
    DataOutputsPerSecond = 5;
    msg_flag = 0; //(1<<FMSTATE_ID) | (1<<FMMOTORINPUT_ID);
    ////////////////////////////////////////////////////////
    ///////// COMPUTE CONTROL VECTOR HERE //////////////////

//    // GET CURRENT ROTATION MATRIX
//    euler2rotmat(g_sensor.dPhi, g_sensor.dTheta, g_sensor.dPsi, R01);
//    //matrix_transpose(3,3,R01,R10);
//
//    // ROTATE POSITION AND DESIRED POSITION TO BODY FRAME
//
//    position0[0] = g_state.dX;  position0[1] = g_state.dY; position0[2] = g_state.dZ;
//    matrix_multiply( 3, 3, 1 , R01, position0, position1 );
//
//    // ROTATE VELOCITIES TO BODY FRAME
//
//    velocity0[0] = g_state.dVx;  velocity0[1] = g_state.dVy;  velocity0[2] = g_state.dVz;
//    matrix_multiply( 3, 3, 1 , R01, velocity0, velocity1 );
//
//    // OUTER LOOP
//    phi_desired = yControl(position1[1], velocity1[1], y_desired);
//    theta_desired = xControl(position1[0], velocity1[0], x_desired);
//
//    // INNER LOOP
//
//    g_u[0] = rollControl(g_sensor.dPhi, g_sensor.dP, phi_desired);
//    g_u[1] = pitchControl(g_sensor.dTheta, g_sensor.dQ, theta_desired);
//    g_u[2] = yawControl(g_sensor.dPsi, g_sensor.dR, psi_desired);
//    g_u[3] = zControl(position1[2], velocity1[2], z_desired, g_sensor.dPhi, g_sensor.dTheta);

        // LQR CONTROLLER
//    g_xerr[0] = g_state.dX - g_xnom[0];
//    g_xerr[1] = g_state.dY - g_xnom[1];
//    g_xerr[2] = g_state.dZ - g_xnom[2];
//    g_xerr[3] = g_state.dVx;
//    g_xerr[4] = g_state.dVy;
//    g_xerr[5] = g_state.dVz;
//    g_xerr[6] = g_state.dPhi;
//    g_xerr[7] = g_state.dTheta;
//    g_xerr[8] = g_state.dPsi;
//    g_xerr[9] = g_state.dP;
//    g_xerr[10] = g_state.dQ;
//    g_xerr[11] = g_state.dR;
//    matrix_multiply(4, 12, 1, K3, g_xerr, g_u);
//    g_u[3] = g_u[3] + 6.77;

    float K00 = 0.8831;
    float K01 = 0.08876;
    float K10 = 0.8828;
    float K11 = 0.08872;
    float K20 = 0.9359;
    float K21 = 0.0943;

    g_u[0] = -K00 * (g_sensor.dPhi - u_outer.roll_desired) - K01 * (g_sensor.dP);
    g_u[1] = -K10 * (g_sensor.dTheta - u_outer.pitch_desired) - K11 * (g_sensor.dQ);
    g_u[2] = -K20 * (g_sensor.dPsi - u_outer.yaw_desired) - K21 * (g_sensor.dR);
    g_u[3] = u_outer.thrust_desired;

    g_motorinput.dU1 = g_u[0];
    g_motorinput.dU2 = g_u[1];
    g_motorinput.dU3 = g_u[2];
    g_motorinput.dU4 = g_u[3];

    /////////////////////////////////////////////////////////

    AE483_send_control();
}


void lab2(void) { //{{{1
    // Note that g_sensor is a global variable which holds on-board filtered sensor data
    // (i.e. the "sensor" data object that you are used to.

    ///////////////////////////////////////////////////////
    ///////// YOUR LAB 2 CODE HERE ////////////////////////
    ///////////////////////////////////////////////////////
    float kp = 0.1;
    float kd = 0.03;
    // UPDATE u!
    g_u[0]=0; // roll control
    g_u[1]= -kp * (g_sensor.dTheta - 3.14159/4) - kd * g_sensor.dQ; // pitch control
    g_u[2]=0; // yaw control
    g_u[3]=3.0; // thrust control
    //////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////

    AE483_send_control();

    // TURN OFF MOTORS WE DONT NEED FOR PITCH CONTROL
    WO_Direct_Individual_Motor_Control.motor[2]=0;
    WO_Direct_Individual_Motor_Control.motor[3]=0;

}

void lab1(void) { //{{{1
    DataOutputsPerSecond = 5;
    msg_flag = (1<<FMSENSOR_ID) | (1<<FMROTOR_ID);
}


void motor_rpm_calibration(void) { //{{{1
    DataOutputsPerSecond = 5;
    msg_flag = 0;

    static int timer=0;

    WO_SDK.ctrl_mode=0x00;    //0x00: direct individual motor control: individual commands for motors 0..3
                            //0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
                            //0x02: attitude and throttle control: commands are input for standard attitude controller
                            //0x03: GPS waypoint control

    WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
                            //1: enable control by HL processor

    WO_SDK.disable_motor_onoff_by_stick=0;

    //scale throttle stick to [0..200] and map it to all motors
    WO_Direct_Individual_Motor_Control.motor[0]=0;
    WO_Direct_Individual_Motor_Control.motor[1]=0;
    WO_Direct_Individual_Motor_Control.motor[2]=0;
    WO_Direct_Individual_Motor_Control.motor[3]=1;

    if(++timer<2000) WO_Direct_Individual_Motor_Control.motor[3]=10;
    else if(timer<4000) WO_Direct_Individual_Motor_Control.motor[3]=20;
    else if(timer<6000) WO_Direct_Individual_Motor_Control.motor[3]=30;
    else if(timer<8000) WO_Direct_Individual_Motor_Control.motor[3]=40;
    else if(timer<10000) WO_Direct_Individual_Motor_Control.motor[3]=50;
    else if(timer<12000) WO_Direct_Individual_Motor_Control.motor[3]=60;
    else if(timer<14000) WO_Direct_Individual_Motor_Control.motor[3]=70;
    else if(timer<16000) WO_Direct_Individual_Motor_Control.motor[3]=80;
    else if(timer<18000) WO_Direct_Individual_Motor_Control.motor[3]=90;
    else if(timer<20000) WO_Direct_Individual_Motor_Control.motor[3]=100;
    else if(timer<22000) WO_Direct_Individual_Motor_Control.motor[3]=110;
    else if(timer<24000) WO_Direct_Individual_Motor_Control.motor[3]=120;
    else if(timer<26000) WO_Direct_Individual_Motor_Control.motor[3]=130;
    else if(timer<28000) WO_Direct_Individual_Motor_Control.motor[3]=140;
    else if(timer<30000) WO_Direct_Individual_Motor_Control.motor[3]=150;
    else if(timer<32000) WO_Direct_Individual_Motor_Control.motor[3]=160;
    else if(timer<34000) WO_Direct_Individual_Motor_Control.motor[3]=170;
    else if(timer<36000) WO_Direct_Individual_Motor_Control.motor[3]=180;
    else if(timer<38000) WO_Direct_Individual_Motor_Control.motor[3]=190;
    else if(timer<40000) WO_Direct_Individual_Motor_Control.motor[3]=200;

    else timer=0;


}



void do_communication(void) { //{{{1
    // This Function Sends Messages to Control PC
    
    // if ((my_lab_count >= comm_start_count) && (max_output_count == 0 || output_count <= max_output_count))
    // {

        // Check timer to see if we want to send a message.
        if (output_timer++ == ControllerCyclesPerSecond / DataOutputsPerSecond) {
    
            //toggleLED0();
    
            // Create an object of class TFmMessage (This is a class, i.e. datatype, that we created)
            TFmMessage msgObj;
    
            // msg_flag is a bit field. You can think of it as just an array of 1's and 0's.
            // The << operator is the bitshift operator. http://www.cs.umd.edu/class/sum2003/cmsc311/Notes/BitOp/bitshift.html
            // The & operator is the bitwise Boolean comparison. It compares two bit fields and
            // returns a bit field which is the AND of the input bits.
            // Finally, > 0 will convert the bit field to an integer and check for positivity.

            // Each variable (e.g. FMROTOR_ID) stores an integer. This is how we keep track of which
            // messages to send for a given message flag.
            
            // SEND GROUPS OF MESSAGES IN CYCLES 
            int sendmsg = 0;
#ifdef SENDMSGEVERYCYCLE
            sendmsg = 1;
#endif

            if ((output_count % 2 == 0) || sendmsg) { // EVEN
                if ((msg_flag & (1 << FMSTATUS_ID)) > 0) {
                    fmStatusObject_tomessage(&g_status, &msgObj);
                    UART_SendMsg(msgObj.data, msgObj.iMsgSize);
                }
                if ((msg_flag & (1 << FMSENSOR_ID)) > 0) {
                    fmSensorObject_tomessage(&g_sensor, &msgObj);
                    UART_SendMsg(msgObj.data, msgObj.iMsgSize);
                }
                if ((msg_flag & (1 << FMGPSSENSOR_ID)) > 0) {
                    fmGPSSensorObject_tomessage(&g_gpssensor, &msgObj);
                    UART_SendMsg(msgObj.data, msgObj.iMsgSize);
                }
                if ((msg_flag & (1 << FMMOTORINPUT_ID)) > 0) {
                    fmMotorInputObject_tomessage(&g_motorinput, &msgObj);
                    UART_SendMsg(msgObj.data, msgObj.iMsgSize);
                }
            }

            if ((output_count % 2 > 0) || sendmsg) { // ODD
                if ((msg_flag & (1 << FMSTATE_ID)) > 0) {
                    fmStateObject_tomessage(&g_state, &msgObj);
                    UART_SendMsg(msgObj.data, msgObj.iMsgSize);
                }
                if ((msg_flag & (1 << FMROTOR_ID)) > 0) {
                    fmRotorObject_tomessage(&g_rotor, &msgObj);
                    UART_SendMsg(msgObj.data, msgObj.iMsgSize);
                }
                if ((msg_flag & (1 << FMPLANNERINFO_ID)) > 0) {
                    fmPlannerInfo_tomessage(&g_pinfo, &msgObj);
                    UART_SendMsg(msgObj.data, msgObj.iMsgSize);
                }
                // added by Vishwa Shah for testing
                if ((msg_flag & (1 << FMINNERLOOP_ID)) > 0) {
                    fmInnerloopObject_tomessage(&u_outer,&msgObj);
                    UART_SendMsg(msgObj.data, msgObj.iMsgSize);
                }
            }

            output_timer = 0;
            output_count++;
        }
    // }
}

//float motor_calib_inv(float omega_des){
//    return (1.47*omega_des-24.4);
//}

void motor_omega_to_cmd(float* omega, float* cmd) { //{{{1
    int i;
    for (i = 0; i < 4; i++) {
        ///////// HUMMINGBIRD ////////////
        cmd[i] = 0.2352 * omega[i] - 24.6266;
        ///////// PELICAN ////////////////
        //cmd[i] = 0.2465 * omega[i] - 31.1405;
    }
}

void u_to_rotorspeed(float* u, float* omega_cmd) { //{{{1
    float twolkF = 1.0/(2.0*l*kF);
    float fourkF = 1.0/(4.0*kF);
    float fourkM = 1.0/(4.0*kM);
    Winv[0] = 0;
    Winv[1] = twolkF;
    Winv[2] = -fourkM;
    Winv[3] = fourkF;
    Winv[1*4+0]= - twolkF;
    Winv[1*4+1]= 0;
    Winv[1*4+2]= fourkM;
    Winv[1*4+3]= fourkF;
    Winv[2*4+0]= 0;
    Winv[2*4+1]= -twolkF;
    Winv[2*4+2]= - fourkM;
    Winv[2*4+3]= fourkF;
    Winv[3*4+0]= twolkF;
    Winv[3*4+1]= 0;
    Winv[3*4+2]= fourkM;
    Winv[3*4+3]= fourkF;

    matrix_multiply(4,4,1,Winv,u,omega_cmd);

    int i;
    for (i=0; i<4; i++) {
        if (omega_cmd[i] < 0)
            omega_cmd[i] = -sqrt(-omega_cmd[i]); //omega_hover - sqrt(-omega_cmd[i]);
        else
            omega_cmd[i] = sqrt(omega_cmd[i]); //omega_hover + sqrt(omega_cmd[i]);
    }

}


void clip_motor_control(float* cmd) { //{{{1
    //make sure commands are never 0 so that motors will always keep spinning
    //also make sure that commands stay within range
    unsigned int i;
    for (i = 0; i<4; i++) {
        if (cmd[i] < 1)
            cmd[i] = 1;
        else if (cmd[i] > 200)
            cmd[i] = 200;
    }
//    for (i = 0; i < 4; i++) {
//        if (!WO_Direct_Individual_Motor_Control.motor[i])
//            WO_Direct_Individual_Motor_Control.motor[i] = 1;
//        else if (WO_Direct_Individual_Motor_Control.motor[i] > 200)
//            WO_Direct_Individual_Motor_Control.motor[i] = 200;
//    }
}

void AE483_send_control(void) { //{{{1

    WO_SDK.ctrl_mode=0x00;    //0x00: direct individual motor control: individual commands for motors 0..3
                            //0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
                            //0x02: attitude and throttle control: commands are input for standard attitude controller
                            //0x03: GPS waypoint control

    WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
                            //1: enable control by HL processor

//    W[0]=1;
//    W[1]=0;
//    W[2]=1;
//    W[3]=-1;
//    W[1*4+0]=1;
//    W[1*4+1]=0;
//    W[1*4+2]=-1;
//    W[1*4+3]=-1;
//    W[2*4+0]=1;
//    W[2*4+1]=1;
//    W[2*4+2]=0;
//    W[2*4+3]=1;
//    W[3*4+0]=1;
//    W[3*4+1]=-1;
//    W[3*4+2]=0;
//    W[3*4+3]=1;

    //matrix_multiply(4,4,1,W,g_u,omega_des);

    u_to_rotorspeed(g_u, g_omega_cmd);
    motor_omega_to_cmd(g_omega_cmd, g_cmd);
    clip_motor_control(g_cmd);

    // ACTUAL MOTORS ARE NUMBERED DIFFERENTLY THAN IN LECTURE:
    // OUR WAY:  motor 0     front (+x)     motor 0   ASCTEC WAY
    //           motor 1     right (+y)     motor 3
    //           motor 2     rear (-x)      motor 1
    //           motor 3     left (-y)      motor 2
    WO_Direct_Individual_Motor_Control.motor[0] = g_cmd[0]; //motor_calib_inv(omega_des[0]);
    WO_Direct_Individual_Motor_Control.motor[3] = g_cmd[1]; //motor_calib_inv(omega_des[1]);
    WO_Direct_Individual_Motor_Control.motor[1] = g_cmd[2]; //motor_calib_inv(omega_des[2]);
    WO_Direct_Individual_Motor_Control.motor[2] = g_cmd[3]; //motor_calib_inv(omega_des[3]);

    //clip_motor_control();
}










