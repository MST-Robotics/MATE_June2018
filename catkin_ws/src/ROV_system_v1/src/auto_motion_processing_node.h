/*
 * Author: Matthew Healy
 * Email: matthew@healy.ink
 * Date: 03-18-2018
 */


#ifndef AUTO_NODE_H
#define AUTO_NODE_H

#include "constants.h"

enum movements {
  move_left,
  move_right,
  move_up,
  move_down
}

std_msgs::Joy movement;

#endif
