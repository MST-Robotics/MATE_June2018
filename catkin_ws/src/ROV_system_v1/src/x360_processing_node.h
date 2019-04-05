/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT, Nathan Welch
 * Email: vgmcn3@mst,edu, tjsxz4@mst.edu, nawwx3@mst.edu
 * Date: 04-04-2019
 * 
 * This handles all data from the xbox controller
 */

#ifndef x360_processing_node_H
#define x360_processing_node_H

#include "constants.h"

/*These files map button names to array indicies
 * just open to find usage
 */
#include "x360_joystick.h"

std_msgs::Float32 magnitude_value;
std_msgs::Float32 angle_value;

/* joystick_callback handles data recieved from the joystick topic
 * Pre: Joystick topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration
 */
void joystick_callback(const sensor_msgs::Joy &joy);

/*
____________TESTED_____________

std_msgs::Bool button_a_state;
std_msgs::Bool button_e_state;
std_msgs::Float32 rotary_4_value;
std_msgs::Float32 rotary_3_value;

std_msgs::Float32 axis_joystick_thumb_stick_x_value;
std_msgs::Float32 axis_joystick_thumb_stick_y_value;

std_msgs::Float32 axis_pov_x_value;
std_msgs::Float32 axis_pov_y_value;

std_msgs::Bool button_b_state;
std_msgs::Bool button_c_state;
std_msgs::Bool button_d_state;
std_msgs::Bool button_pinky_trigger_state;

std_msgs::Bool button_h1_up_state;
std_msgs::Bool button_h1_right_state;
std_msgs::Bool button_h1_down_state;
std_msgs::Bool button_h1_left_state;

std_msgs::Bool button_h2_up_state;
std_msgs::Bool button_h2_right_state;
std_msgs::Bool button_h2_down_state;
std_msgs::Bool button_h2_left_state;

std_msgs::Float32 axis_stick_rotation_value;
std_msgs::Bool button_trigger_state;
std_msgs::Int16 gimbal_x_value;
std_msgs::Int16 gimbal_y_value;

*/
#endif
