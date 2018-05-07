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
#define WRIST_HOME 90
#define WRIST_MIN 0
#define WRIST_MAX 180

//Elbow bounds
#define ELBOW_HOME 90
#define ELBOW_MIN 0
#define ELBOW_MAX 180

//Claw bounds
#define CLAW_HOME 90
#define CLAW_MIN 0
#define CLAW_MAX 180

//Gimbal X bounds
#define GIMBAL_X_HOME 87
#define GIMBAL_X_MIN 35
#define GIMBAL_X_MAX 160

//Gimbal Y bounds
#define GIMBAL_Y_HOME 91
#define GIMBAL_Y_MIN 45
#define GIMBAL_Y_MAX 170

#define GIMBAL_MOVEMENT_SPEED 1
#define movement_speed 1

//messages for manipulator joints
std_msgs::UInt8 wrist_value;
std_msgs::UInt8 claw_value;
std_msgs::UInt8 elbow_value;

//messages for gimbal joints
std_msgs::Int16 gimbal_x_value;
std_msgs::Int16 gimbal_y_value;

//Gimbal control
void gimbal_home_cb(const std_msgs::Bool &msg);
void gimbal_x_cb(const std_msgs::Int16 &msg);
void gimbal_y_cb(const std_msgs::Int16 &msg);

//Elbow control
void sw1_callback(const std_msgs::Bool &msg);//increment
void sw2_callback(const std_msgs::Bool &msg);//decrement

//Wrist control
void sw3_callback(const std_msgs::Bool &msg);//increment
void sw4_callback(const std_msgs::Bool &msg);//decrement

//Claw control
void sw5_callback(const std_msgs::Bool &msg);//increment
void sw6_callback(const std_msgs::Bool &msg);//decrement

#endif
