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

#include "Xbox_controller.h"

std_msgs::Bool button_A_state;

void xbox_callback(const sensor_msgs::Joy &joy);

#endif
