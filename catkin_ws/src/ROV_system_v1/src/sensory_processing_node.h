/*
 * Author: Vinnie Marco, Megan Cash
 * Email: vgmcn3@mst,edu, mecfkc@mst.edu
 * Date: 02-22-2018
 *
 * This handles all data for the sensors
 */

#ifndef sensory_processing_node_H
#define sensory_processing_node_H

#include "constants.h"

/*
 * Stores orientation data
 * x=roll y=pitch z=heading
 */
geometry_msgs::Vector3 orientation;

/*
 * Stores pixy data
 * x=color y=width z=height
 */
std_msgs::Char plane_type;

//
void accel_cb(const geometry_msgs::Vector3 &msg);

//
void mag_cb(const geometry_msgs::Vector3 &msg);

//
void pixy_cb(const geometry_msgs::Vector3 &msg);

#endif
