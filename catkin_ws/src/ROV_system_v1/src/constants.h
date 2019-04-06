/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT
 * Email: vgmcn3@mst,edu, tjsxz4@mst.edu
 * Date: 01-18-2018
 *
 * This file holds the constant values used for all of our nodes, publishers, and subscribers
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "ros/ros.h" //required for use of ros

/*ros message data types can be
 * found at: http://wiki.ros.org/std_msgs
 */
#include <sensor_msgs/Joy.h>//include required per different message type
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <math.h>

#define PI 3.14159

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:movement 
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 0.116667 // in Rolla, MO
//moved all of this to the .h file for auto movement a ton of errors got fixed
/* For autonomous movement */
// Tuned through trial-and-error
/*
typedef struct{
  double x_axis;
  double y_axis;
  double z_axis;
} movement ;

movement move_up = {.x_axis = 0, .y_axis = 0, .z_axis = 0};
movement move_down = {.x_axis = 0, .y_axis = 0, .z_axis = 0};
movement move_left = {.x_axis = 0, .y_axis = 0, .z_axis = 0};
movement move_right = {.x_axis = 0, .y_axis = 0, .z_axis = 0};
*/

#endif
