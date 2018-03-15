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


std_msgs::Int16 arm_motor1_value;
std_msgs::Int16 arm_motor2_value;
std_msgs::Int16 arm_motor3_value;
std_msgs::Int16 arm_motor4_value;
std_msgs::Int16 arm_motor5_value;
std_msgs::Int16 arm_motor6_value;


/* velocity_callback handles data recieved from the joystick_x_topic subscription
 * Pre: joystick_x_topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration
 */
void xbox_velocity_callback(const std_msgs::Float32 &msg);

/* angle_callback handles data recieved from the joystick_y_topic subscription
 * Pre: joystick_y_topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration******
 */
void xbox_angle_callback(const std_msgs::Float32 &msg);

void xbox_calc_motors();

#endif
