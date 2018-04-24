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

float xbox_magnitude = 0.0;
float xbox_angle = 0.0;
float xbox_moment = 0.0;

//we won't have motors, we will just have servos
std_msgs::Int16 wrist_value;
std_msgs::Int16 claw_value;
std_msgs::Int16 elbow_value;

void sw1_callback(const std_msgs::Bool &msg);
void sw2_callback(const std_msgs::Bool &msg);
void sw3_callback(const std_msgs::Bool &msg);
void sw4_callback(const std_msgs::Bool &msg);
void sw5_callback(const std_msgs::Bool &msg);
void sw6_callback(const std_msgs::Bool &msg);

#endif
