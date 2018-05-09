/*
 * Author: Tamara Spivey
 * Email: tjsxz4@mst.edu
 * Date: 03-10-2018
 * 
 * 
 */


#ifndef arm_motion_processing_node_H
#define arm_motion_processing_node_H

#include "constants.h"

//Wrist bounds
#define WRIST_HOME 96
#define WRIST_MIN 20
#define WRIST_MAX 160

//Elbow bounds
#define ELBOW_HOME 154
#define ELBOW_MIN 20
#define ELBOW_MAX 160

//Claw bounds
#define CLAW_HOME 158
#define CLAW_MIN 20
#define CLAW_MAX 160

//Gimbal X bounds
#define GIMBAL_X_HOME 87
#define GIMBAL_X_MIN 35
#define GIMBAL_X_MAX 160

//Gimbal Y bounds
#define GIMBAL_Y_HOME 91
#define GIMBAL_Y_MIN 45
#define GIMBAL_Y_MAX 170

#define GIMBAL_STEP_SIZE 1
#define MANIPULATOR_STEP_SIZE 3

#define LEVELER_SPEED 127

int pos_wrist = WRIST_HOME;
int pos_elbow = ELBOW_HOME;
int pos_claw = CLAW_HOME;

int pos_gimbal_x = GIMBAL_X_HOME;
int pos_gimbal_y = GIMBAL_Y_HOME;


//messages for manipulator joints
std_msgs::UInt8 wrist_value;
std_msgs::UInt8 claw_value;
std_msgs::UInt8 elbow_value;

//messages for gimbal joints
std_msgs::Int16 gimbal_x_value;
std_msgs::Int16 gimbal_y_value;

//message for leveler controls
std_msgs::UInt8 leveler_value;

std_msgs::Bool pid_state;

//Gimbal control
void gimbal_home_cb(const std_msgs::Bool &msg);
void gimbal_x_cb(const std_msgs::Int16 &msg);
void gimbal_y_cb(const std_msgs::Int16 &msg);

//Manipulator control
void manipulator_home_cb(const std_msgs::Bool &msg);

//Elbow control
void sw1_callback(const std_msgs::Bool &msg);//increment
void sw2_callback(const std_msgs::Bool &msg);//decrement

//Wrist control
void sw3_callback(const std_msgs::Bool &msg);//increment
void sw4_callback(const std_msgs::Bool &msg);//decrement

//Claw control
void sw5_callback(const std_msgs::Bool &msg);//increment
void sw6_callback(const std_msgs::Bool &msg);//decrement

void tgl1_callback(const std_msgs::Int16 &msg);//used for controlling the leveler
void tgl2_callback(const std_msgs::Int16 &msg);//used for controlling the opening/closing the claw
void m1_callback(const std:msgs::Bool &msg);//simply passses the value of M1 to the slower-running node

#endif
