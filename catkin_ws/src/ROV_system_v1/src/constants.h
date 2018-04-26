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

#include <math.h>

#define PI 3.14159

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 0.116667 // in Rolla, MO

//Pixy camera variables
#define red 1
#define blue 2
#define yellow 3
#define tolerance 1 //add a tolerance for the ratio of the object to counteract camera inaccuracy
#define rectangle_ratio 6.666 //predetermined ratios of shapes on the wings
#define triangle_ratio 1.666

#endif
