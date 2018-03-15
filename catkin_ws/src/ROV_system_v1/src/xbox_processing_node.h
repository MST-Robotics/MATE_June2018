/*
 * Author: Tamara Spivey
 * Email: tjsxz4@mst.edu
 * Date: 03-10-2018
 * 
 * This handles all data from the xbox 360 controller
 */

#ifndef xbox_processing_node_H
#define xbox_processing_node_H

#include "constants.h"

#include "xbox_controller.h"

std_msgs::Bool button_A_state;
std_msgs::Bool button_B_state;
std_msgs::Bool button_X_state;
std_msgs::Bool button_Y_state;

std_msgs::Bool button_back_state;
std_msgs::Bool button_start_state;
std_msgs::Bool button_power_state;

std_msgs::Bool button_LB_state;
std_msgs::Bool button_RB_state;

std_msgs::Bool button_stick_left_state;
std_msgs::Bool button_stick_right_state;

std_msgs::Float32 left_xbox_angle_value;
std_msgs::Float32 right_xbox_angle_value;

std_msgs::Float32 left_xbox_magnitude_value;
std_msgs::Float32 right_xbox_magnitude_value;

std_msgs::Float32 axis_RT_value;
std_msgs::Float32 axis_LT_value;

std_msgs::Float32 axis_cross_key_x_value;
std_msgs::Float32 axis_cross_key_y_value;

void xbox_callback(const sensor_msgs::Joy &joy);

#endif
