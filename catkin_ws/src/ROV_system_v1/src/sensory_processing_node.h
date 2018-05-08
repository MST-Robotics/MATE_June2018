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
 * Stores temperature data
 */
std_msgs::Float32 temp_F;

//
void raw_temp_cb(const std_msgs::Float32 &msg);

#endif
