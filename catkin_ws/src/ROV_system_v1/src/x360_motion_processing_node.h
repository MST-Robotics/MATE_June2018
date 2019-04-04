/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT, Nathan Welch
 * Email: vgmcn3@mst,edu, tjsxz4@mst.edu, nawwx3@mst.edu
 * Date: 04-04-2019
 *
 * This handles all data for the motors
 */

#ifndef x360_motion_processing_node_H
#define x360_motion_processing_node_H

#include "constants.h"
#define MIN_PRECISION_SCALE 0.25
#define MAX_PRECISION_SCALE 0.85

float magnitude = 0.0;
float angle = 0.0;
float moment = 0.0;

bool trigger_pulled, pinky_trigger_pulled = 0; //track the status of either of the triggers
bool button_e_pulled = 0;

//these are globals to multiply the motor vaues based on throttle values
float horizontal_precision = 1.0;
float vertical_precision = 1.0;

std_msgs::Int16 motor1x_value;
std_msgs::Int16 motor2x_value;
// std_msgs::Int16 motor3_value;
// std_msgs::Int16 motor4_value;
// std_msgs::Int16 motor5_value;
// std_msgs::Int16 motor6_value;
// std_msgs::Int16 motor7_value;

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

/* twist_callback handles data from the "" subscription (the one that handles the twisting of the joystick)
 * Pre: "" has to be initialized
 * Post: Any variables are updated to their current values for each itteration
 */
void twist_callback(const std_msgs::Float32 &msg);

/* calc_motors handles data from velocity_callback, angle_callback, and twist_callback to calculate ROV motor movement
 * Pre: magnitude, angle, and moment are initialized
 * Post: Any variables are updated to their current values for each itteration
 */
void calc_motors();

//this function calcualtes the precision scale for the movement controls from the throttle
/* axis_left_thruster_callback calcualtes the precision scale for the movement controls from the throttle
 * Pre: axis_left_thruster_topic has to be initialized
 * Post: The range -1.0 to 1.0 is mapped to MIN_PRECISION_SCALE to 1
 */
void axis_left_thruster_callback(const std_msgs::Float32 &msg);
void axis_right_thruster_callback(const std_msgs::Float32 &msg);

//dont know if this is going to work
void button_e_callback(const std_msgs::Bool &msg);

//function used for mapping float values
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

#endif
