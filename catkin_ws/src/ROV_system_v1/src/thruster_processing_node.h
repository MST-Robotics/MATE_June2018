/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT
 * Email: vgmcn3@mst,edu
 * Date: 01-18-2018
 * 
 * This node handles data from the thruster
 */

#ifndef THRUSTER_PROCESSING_NODE_H
#define THRUSTER_PROCESSING_NODE_H

#include "constants.h"
#include "x56_thruster.h"

std_msgs::Float32 throttle_value; 

/* thruster_callback handles data recieved from the thruster topic
 * Pre: thruster_topic has to be running
 * Post: Any variables are updated to their current values for each iteration
 */
void thruster_callback(const sensor_msgs::Joy &joy);

#endif
