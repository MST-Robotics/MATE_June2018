#ifndef practice_node_H
#define practice_node_H

#include "ros/ros.h"

#include <std_msgs/Bool.h>


std_msgs::Bool button_state;


void test_cb(const std_msgs::Bool &value);





#endif
