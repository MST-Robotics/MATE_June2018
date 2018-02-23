/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT
 * Email: vgmcn3@mst,edu, tjsxz4@mst.edu
 * Date: 01-18-2018
 *
 * This handles all data for the motors
 */

#ifndef motion_processing_node_H
#define motion_processing_node_H

#include "constants.h"

std_msgs::Int16 motor1_value;
std_msgs::Int16 motor2_value;
std_msgs::Int16 motor3_value;
std_msgs::Int16 motor4_value;
std_msgs::Int16 motor5_value;
std_msgs::Int16 motor6_value;


/* velocity_callback handles data recieved from the joystick_x_topic subscription
 * Pre: joystick_x_topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration
 */
void velocity_callback(const std_msgs::Float32 &msg);

/* angle_callback handles data recieved from the joystick_y_topic subscription
 * Pre: joystick_y_topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration******
 */
void angle_callback(const std_msgs::Float32 &msg);

/* trigger_callback handles data recieved from the trigger_topic subscription
 * Pre: trigger_topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration******
 */
void trigger_callback(const std_msgs::Bool &msg);

/* button_pinky_trigger_callback handles data recieved from the button_pinky_trigger_topic subscription
 * Pre: button_pinky_trigger_topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration******
 */
void button_pinky_trigger_callback(const std_msgs::Bool &msg);

void orientation_callback(const geometry_msgs::Vector3 &msg);

#endif
