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

#define XBOX_FORCE_X_MODIFIER 1 /*To Be Determined*/
#define XBOX_FORCE_Y_MODIFIER 1 /*To Be Determined*/
#define XBOX_MOMENT_MODIFIER 1 /*To Be Determined*/
#define ARM_MOTOR_NEUTRAL 1500
#define ARM_MOTOR_RAMP 400
#define POS_WRIST_LOWER_BOUND 0
#define POS_WRIST_UPPER_BOUND 180
#define POS_ELBOW_LOWER_BOUND 0
#define POS_ELBOW_UPPER_BOUND 180
#define POS_CLAW_LOWER_BOUND 0
#define POS_CLAW_UPPER_BOUND 180

#define GIMBAL_X_HOME 92
#define GIMBAL_X_MIN 35
#define GIMBAL_X_MAX 160

#define GIMBAL_Y_HOME 88
#define GIMBAL_Y_MIN 45
#define GIMBAL_Y_MAX 170

#define GIMBAL_MOVEMENT_SPEED 1
#define movement_speed 1

//we won't have motors, we will just have servos
std_msgs::Int16 wrist_value;
std_msgs::Int16 claw_value;
std_msgs::Int16 elbow_value;

std_msgs::Int16 gimbal_x_value;
std_msgs::Int16 gimbal_y_value;

void gimbal_home_cb(const std_msgs::Bool &msg);

void gimbal_x_cb(const std_msgs::Int16 &msg);
void gimbal_y_cb(const std_msgs::Int16 &msg);

void sw1_callback(const std_msgs::Bool &msg);
void sw2_callback(const std_msgs::Bool &msg);
void sw3_callback(const std_msgs::Bool &msg);
void sw4_callback(const std_msgs::Bool &msg);
void sw5_callback(const std_msgs::Bool &msg);
void sw6_callback(const std_msgs::Bool &msg);

#endif
