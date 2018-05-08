/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT
 * Email: vgmcn3@mst,edu, tjsxz4@mst.edu
 * Date: 01-18-2018
 * 
 * This node handles data from the thruster
 */

#ifndef THRUSTER_PROCESSING_NODE_H
#define THRUSTER_PROCESSING_NODE_H

#include "constants.h"
#include "x56_thruster.h"


//need to add support for the leveling device

std_msgs::Bool button_sw1_state; // move elbow up
std_msgs::Bool button_sw2_state; // move elbow down

std_msgs::Bool button_sw3_state; // move wrist up
std_msgs::Bool button_sw4_state; // move wrist down

std_msgs::Bool button_sw5_state; // claw closes
std_msgs::Bool button_sw6_state; // claw opens

std_msgs::Bool button_m1_state;//used for enabling/disabling PID adjustments

std_msgs::Int16 button_tgl_value;//used for controlling the leveler

std_msgs::Float32 axis_left_thruster_value;//used a precision scaler for the motor speeds
std_msgs::Int16 setpoint_value;//used to store the setpoint

/* thruster_callback handles data recieved from the thruster topic
 * Pre: thruster_topic has to be running
 * Post: Any variables are updated to their current values for each iteration
 */
void thruster_callback(const sensor_msgs::Joy &joy);

//this funciton maps a range of floats to another range of floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

#endif
