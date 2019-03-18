/*
 * Author: Matthew Healy
 * Email: matthew@healy.ink
 * Date: 03-18-2018
 */


#ifndef AUTO_NODE_H
#define AUTO_NODE_H

#include "constants.h"


/* Tuned through trial-and-error */
#define MOVEMENT_LEFT
#define MOVEMENT_RIGHT
#define MOVEMENT_UP
#define MOVEMENT_DOWN

std_msgs::Float32 magnitude_value;
std_msgs::Float32 angle_value;
std_msgs::Float32 axis_stick_rotation_value;

#endif
