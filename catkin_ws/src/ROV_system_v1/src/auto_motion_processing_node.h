/*
 * Author: Matthew Healy
 * Email: matthew@healy.ink
 * Date: 03-18-2018
 */


#ifndef AUTO_NODE_H
#define AUTO_NODE_H

#include "constants.h"
#include "x56_joystick.h"
/*
enum movements {
  move_left,
  move_right,
  move_up,
  move_down
}

std_msgs::Joy movement;
*/

/* For autonomous movement */
// Tuned through trial-and-error
typedef struct{
  double x_axis;
  double y_axis;
  double z_axis;
} movement ;

movement move_up = {.x_axis = 0, .y_axis = 0, .z_axis = 0};
movement move_down = {.x_axis = 0, .y_axis = 0, .z_axis = 0};
movement move_left = {.x_axis = 0, .y_axis = 0, .z_axis = 0};
movement move_right = {.x_axis = 0, .y_axis = 0, .z_axis = 0};


void thruster_callback(const sensor_msgs::Joy &joy);

void auto_callback(const std_msgs::Int16 &msg);
#endif
