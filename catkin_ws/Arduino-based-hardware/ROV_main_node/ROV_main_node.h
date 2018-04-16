/**
 *Author: 	Vinnie Marco
 *Date:	 4-8-2017
 *Email: 	vgmcn3@mst.edu
 *
 * 
 *Description: 	This file defines ROS messages, callback functions,
				subscribers, and publishers used by the ROV_main_node
 */

#ifndef ROV_main_node_H
#define ROV_main_node_H

#include <ROV_main_setup.h>
#include <ros.h>//required to use ros with arduino

/*ros message data types can be
 * found at: http://wiki.ros.org/std_msgs
*/
#include <std_msgs/Bool.h> //include required per type of message
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;//this is necessary

geometry_msgs::Vector3 accelerometer;
geometry_msgs::Vector3 magnetometer;
geometry_msgs::Vector3 pixy_data;

/*Front left motor*/
void motor1_cb(const std_msgs::Int16 &msg)
{
  pwm_primary.writeMicroseconds(motor_1, msg.data);
}

/*Middle left motor*/
void motor2_cb(const std_msgs::Int16 &msg)
{
  pwm_primary.writeMicroseconds(motor_2, msg.data);
}

/*Back left motor*/
void motor3_cb(const std_msgs::Int16 &msg)
{
  pwm_primary.writeMicroseconds(motor_3, msg.data);
}

/*Front right motor*/
void motor4_cb(const std_msgs::Int16 &msg)
{
  pwm_primary.writeMicroseconds(motor_4, msg.data);
}

/*Middle right motor*/
void motor5_cb(const std_msgs::Int16 &msg)
{
  pwm_primary.writeMicroseconds(motor_5, msg.data);
}

/*Back right motor*/
void motor6_cb(const std_msgs::Int16 &msg)
{
  pwm_primary.writeMicroseconds(motor_6, msg.data);
}

//set up subscriptions
//ros::Subscriber<std_msgs::Bool> e_button_sub("e_button_topic", button_e_cb);
ros::Subscriber<std_msgs::Int16> motor1_sub("motor1_topic", motor1_cb);
ros::Subscriber<std_msgs::Int16> motor2_sub("motor2_topic", motor2_cb);
ros::Subscriber<std_msgs::Int16> motor3_sub("motor3_topic", motor3_cb);
ros::Subscriber<std_msgs::Int16> motor4_sub("motor4_topic", motor4_cb);
ros::Subscriber<std_msgs::Int16> motor5_sub("motor5_topic", motor5_cb);
ros::Subscriber<std_msgs::Int16> motor6_sub("motor6_topic", motor6_cb);

/*
ros::Subscriber<std_msgs::Bool> a_button_sub("a_button_topic", button_a_cb);
ros::Subscriber<std_msgs::Bool> trigger_sub("trigger_topic", trigger_cb);
ros::Subscriber<std_msgs::Bool> pinky_trigger_sub("pinky_trigger_topic", pinky_trigger_cb);
ros::Subscriber<std_msgs::Float32> joystick_rotation_sub("joystick_rotation", joystick_rotation_cb);
*/

//set up publishers
ros::Publisher accel_pub("accel_topic", &accelerometer);
ros::Publisher mag_pub("mag_topic", &magnetometer);
ros::Publisher pixy_pub("pixy_data_topic", &pixy_data);//message for raw data from pixy

#endif

