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

std_msgs::Float32 throttle_value; 

std_msgs::Bool button_sw1_state; // move crank(assuming its the joint attached to rov for main movement because name is not self-explanatory) up
std_msgs::Bool button_sw2_state; // move crank down

std_msgs::Bool button_sw3_state; // move wrist up
std_msgs::Bool button_sw4_state; // move wrist down

std_msgs::Bool button_sw5_state; // claw closes
std_msgs::Bool button_sw6_state; // claw opens

//I don't know what extender is, assume it extends length somehow(akin to a turtle)
std_msgs::Bool button_tgl1_up_state; //extends
std_msgs::Bool button_tgl1_down_state; //goes into the shell

std_msgs::Bool axis_left_thruster_value;

/* thruster_callback handles data recieved from the thruster topic
 * Pre: thruster_topic has to be running
 * Post: Any variables are updated to their current values for each iteration
 */
void thruster_callback(const sensor_msgs::Joy &joy);

#endif
