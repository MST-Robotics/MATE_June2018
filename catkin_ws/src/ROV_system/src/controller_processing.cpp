/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst,edu
 * Date: 6-19-2017
 * 
 * To start the system, run from terminal:
 * cd ~/catkin_ws
 * source devel/setup.bash
 * roslaunch learning_joy rov_system.launch
 * 
 * This will start all necessary nodes to read both controllers
 * and stream data to the arduino.
 */

//required for use of ros
#include "ros/ros.h"

/*ros message data types can be 
 * found at: http://wiki.ros.org/std_msgs
 */
#include <sensor_msgs/Joy.h>//include required per different message type
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

/*These files map button names to array indicies
 * just open to find usage
 */
#include "x56_joystick.h"
#include "x56_thruster.h"

std_msgs::Bool button_trigger_state;
std_msgs::Bool button_a_state;
std_msgs::Bool button_e_state;
std_msgs::Float32 rotary_4_value;

/*
/*
____________TESTED_____________
std_msgs::Float32 rotary_3_value;
std_msgs::Float32 axis_stick_x_value;
std_msgs::Float32 axis_stick_y_value;
std_msgs::Float32 axis_joystick_thumb_stick_x_value;
std_msgs::Float32 axis_joystick_thumb_stick_y_value;
std_msgs::Float32 axis_stick_rotation_value;
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

std_msgs::Float32 axis_left_thruster_value;
std_msgs::Float32 axis_right_thruster_value;

std_msgs::Float32 axis_rotary_f_value;
std_msgs::Float32 axis_rotary_g_value;

std_msgs::Float32 axis_thruster_thumb_stick_x_value;
std_msgs::Float32 axis_thruster_thumb_stick_y_value;

std_msgs::Bool button_rotary_f_state;
std_msgs::Bool button_rotary_g_state;

std_msgs::Bool button_i_state;
std_msgs::Bool button_h_state;

std_msgs::Bool button_sw1_state;
std_msgs::Bool button_sw2_state;
std_msgs::Bool button_sw3_state;
std_msgs::Bool button_sw4_state;
std_msgs::Bool button_sw5_state;
std_msgs::Bool button_sw6_state;

std_msgs::Bool button_tgl1_up_state;
std_msgs::Bool button_tgl1_down_state;

std_msgs::Bool button_tgl2_up_state;
std_msgs::Bool button_tgl2_down_state;

std_msgs::Bool button_tgl3_up_state;
std_msgs::Bool button_tgl3_down_state;

std_msgs::Bool button_tgl4_up_state;
std_msgs::Bool button_tgl4_down_state;

std_msgs::Bool button_h3_up_state;
std_msgs::Bool button_h3_right_state;
std_msgs::Bool button_h3_down_state;
std_msgs::Bool button_h3_left_state;

std_msgs::Bool button_h4_up_state;
std_msgs::Bool button_h4_right_state;
std_msgs::Bool button_h4_down_state;
std_msgs::Bool button_h4_left_state;

std_msgs::Bool button_ki_up_state;
std_msgs::Bool button_ki_down_state;

std_msgs::Bool button_thumb_stick_state;
std_msgs::Bool button_thumb_slider_state;

std_msgs::Bool button_scroll_forward_state;
std_msgs::Bool button_scroll_reverse_state;

std_msgs::Bool button_m1_state;
std_msgs::Bool button_m2_state;
std_msgs::Bool button_s1_state;
*/


/*
____________NEEDS TESTING_____________

*/

//process data from joystick
void joystick_cb(const sensor_msgs::Joy &joy)
{
  //buttons are stored in joy.buttons[]
  //axes are stored in joy.axis[]
  if(joy.buttons[button_trigger] == 1)
  {
    ROS_INFO("Trigger pulled");
    button_trigger_state.data = 1;
  }
  else
    button_trigger_state.data = 0;


  if(joy.buttons[button_a] == 1)
  {
    ROS_INFO("Button A pressed");
    button_a_state.data = 1;
  }
  else
    button_a_state.data = 0;

/*
	Tested

  if(joy.buttons[button_b] == 1)
  {
    ROS_INFO("Button B pressed");
    button_b_state.data = 1;
  }
  else
    button_b_state.data = 0;

  if(joy.buttons[button_c] == 1)
  {
    ROS_INFO("Button C pressed");
    button_c_state.data = 1;
  }
  else
    button_c_state.data = 0;

  if(joy.buttons[button_d] == 1)
  {
    ROS_INFO("Button D pressed");
    button_d_state.data = 1;
  }
  else
    button_d_state.data = 0;

  if(joy.buttons[button_pinky_trigger] == 1)
  {
    ROS_INFO("Button pinky trigger pressed");
    button_pinky_trigger_state.data = 1;
  }
  else
    button_pinky_trigger_state.data = 0;

  if(joy.buttons[button_h1_up] == 1)
  {
    ROS_INFO("Button h1 up pressed");
    button_h1_up_state.data = 1;
  }
  else
    button_h1_up_state.data = 0;

  if(joy.buttons[button_h1_right] == 1)
  {
    ROS_INFO("Button h1 right pressed");
    button_h1_right_state.data = 1;
  }
  else
    button_h1_right_state.data = 0;

  if(joy.buttons[button_h1_down] == 1)
  {
    ROS_INFO("Button h1 down pressed");
    button_h1_down_state.data = 1;
  }
  else
    button_h1_down_state.data = 0;

  if(joy.buttons[button_h1_left] == 1)
  {
    ROS_INFO("Button h1 left pressed");
    button_h1_left_state.data = 1;
  }
  else
    button_h1_left_state.data = 0;


  if(joy.buttons[button_h2_up] == 1)
  {
    ROS_INFO("Button h2 up pressed");
    button_h2_up_state.data = 1;
  }
  else
    button_h2_up_state.data = 0;

  if(joy.buttons[button_h2_right] == 1)
  {
    ROS_INFO("Button h2 right pressed");
    button_h2_right_state.data = 1;
  }
  else
    button_h2_right_state.data = 0;

  if(joy.buttons[button_h2_down] == 1)
  {
    ROS_INFO("Button h2 down pressed");
    button_h2_down_state.data = 1;
  }
  else
    button_h2_down_state.data = 0;

  if(joy.buttons[button_h2_left] == 1)
  {
    ROS_INFO("Button h2 left pressed");
    button_h2_left_state.data = 1;
  }
  else
    button_h2_left_state.data = 0;

  axis_stick_x_value.data = joy.axes[axis_stick_x];
  axis_stick_y_value.data = joy.axes[axis_stick_y];
  axis_joystick_thumb_stick_x_value.data = joy.axes[axis_joystick_thumb_stick_x];
  axis_joystick_thumb_stick_y_value.data = joy.axes[axis_joystick_thumb_stick_y];
  axis_stick_rotation_value.data = joy.axes[axis_stick_rotation];
  axis_pov_x_value.data = joy.axes[axis_pov_x];
  axis_pov_y_value.data = joy.axes[axis_pov_y];


*/
}



//process data from thruster
void thruster_cb(const sensor_msgs::Joy &joy)
{
  //buttons are stored in joy.buttons[]
  //axes are stored in joy.axis[]
  if(joy.buttons[button_e] == 1)
  {
    ROS_INFO("Button E pressed");
    button_e_state.data = 1;
  }
  else
    button_e_state.data = 0;

  rotary_4_value.data = joy.axes[axis_base_rotary_4];

 /* 
  Done
  rotary_3_value.data = joy.axes[axis_base_rotary_3];
  axis_left_thruster_value.data = joy.axes[axis_left_thruster];
  axis_right_thruster_value.data = joy.axes[axis_right_thruster];

  axis_rotary_f_value.data = joy.axes[axis_rotary_f];
  axis_rotary_g_value.data = joy.axes[axis_rotary_g];

  axis_thruster_thumb_stick_x_value.data = joy.axes[axis_thruster_thumb_stick_x];
  axis_thruster_thumb_stick_y_value.data = joy.axes[axis_thruster_thumb_stick_y];

  if(joy.buttons[button_rotary_f] == 1)
  {
    ROS_INFO("Button F pressed");
    button_rotary_f_state.data = 1;
  }
  else
    button_rotary_f_state.data = 0;

  if(joy.buttons[button_rotary_g] == 1)
  {
    ROS_INFO("Button G pressed");
    button_rotary_g_state.data = 1;
  }
  else
    button_rotary_g_state.data = 0;

  if(joy.buttons[button_i] == 1)
  {
    ROS_INFO("Button i pressed");
    button_i_state.data = 1;
  }
  else
    button_i_state.data = 0;

  if(joy.buttons[button_h] == 1)
  {
    ROS_INFO("Button h pressed");
    button_h_state.data = 1;
  }
  else
    button_h_state.data = 0;

  if(joy.buttons[button_sw1] == 1)
  {
    ROS_INFO("Button sw1 pressed");
    button_sw1_state.data = 1;
  }
  else
    button_sw1_state.data = 0;

  if(joy.buttons[button_sw2] == 1)
  {
    ROS_INFO("Button sw2 pressed");
    button_sw2_state.data = 1;
  }
  else
    button_sw2_state.data = 0;

  if(joy.buttons[button_sw3] == 1)
  {
    ROS_INFO("Button sw3 pressed");
    button_sw3_state.data = 1;
  }
  else
    button_sw3_state.data = 0;


  if(joy.buttons[button_sw4] == 1)
  {
    ROS_INFO("Button sw4 pressed");
    button_sw4_state.data = 1;
  }
  else
    button_sw4_state.data = 0;

  if(joy.buttons[button_sw5] == 1)
  {
    ROS_INFO("Button sw5 pressed");
    button_sw5_state.data = 1;
  }
  else
    button_sw5_state.data = 0;

  if(joy.buttons[button_sw6] == 1)
  {
    ROS_INFO("Button sw6 pressed");
    button_sw6_state.data = 1;
  }
  else
    button_sw6_state.data = 0;

 if(joy.buttons[button_tgl1_up] == 1)
  {
    ROS_INFO("Button tgl1_up pressed");
    button_tgl1_up_state.data = 1;
  }
  else
    button_tgl1_up_state.data = 0;

  if(joy.buttons[button_tgl1_down] == 1)
  {
    ROS_INFO("Button tgl1_down pressed");
    button_tgl1_down_state.data = 1;
  }
  else
    button_tgl1_down_state.data = 0;

  if(joy.buttons[button_tgl2_up] == 1)
  {
    ROS_INFO("Button tgl2_up pressed");
    button_tgl2_up_state.data = 1;
  }
  else
    button_tgl2_up_state.data = 0;

  if(joy.buttons[button_tgl2_down] == 1)
  {
    ROS_INFO("Button tgl2_down pressed");
    button_tgl2_down_state.data = 1;
  }
  else
    button_tgl2_down_state.data = 0;


  if(joy.buttons[button_tgl3_up] == 1)
  {
    ROS_INFO("Button tgl3_up pressed");
    button_tgl3_up_state.data = 1;
  }
  else
    button_tgl3_up_state.data = 0;

  if(joy.buttons[button_tgl3_down] == 1)
  {
    ROS_INFO("Button tgl3_down pressed");
    button_tgl3_down_state.data = 1;
  }
  else
    button_tgl3_down_state.data = 0;

  if(joy.buttons[button_tgl4_up] == 1)
  {
    ROS_INFO("Button tgl4_up pressed");
    button_tgl4_up_state.data = 1;
  }
  else
    button_tgl4_up_state.data = 0;

  if(joy.buttons[button_tgl4_down] == 1)
  {
    ROS_INFO("Button tgl4_down pressed");
    button_tgl4_down_state.data = 1;
  }
  else
    button_tgl4_down_state.data = 0;

if(joy.buttons[button_h3_up] == 1)
  {
    ROS_INFO("Button h3_up pressed");
    button_h3_up_state.data = 1;
  }
  else
    button_h3_up_state.data = 0;

  if(joy.buttons[button_h3_right] == 1)
  {
    ROS_INFO("Button h3_right pressed");
    button_h3_right_state.data = 1;
  }
  else
    button_h3_right_state.data = 0;

  if(joy.buttons[button_h3_down] == 1)
  {
    ROS_INFO("Button h3_down pressed");
    button_h3_down_state.data = 1;
  }
  else
    button_h3_down_state.data = 0;

  if(joy.buttons[button_h3_left] == 1)
  {
    ROS_INFO("Button h3_left pressed");
    button_h3_left_state.data = 1;
  }
  else
    button_h3_left_state.data = 0;



  if(joy.buttons[button_h4_up] == 1)
  {
    ROS_INFO("Button h4_up pressed");
    button_h4_up_state.data = 1;
  }
  else
    button_h4_up_state.data = 0;

  if(joy.buttons[button_h4_right] == 1)
  {
    ROS_INFO("Button h4_right pressed");
    button_h4_right_state.data = 1;
  }
  else
    button_h4_right_state.data = 0;

  if(joy.buttons[button_h4_down] == 1)
  {
    ROS_INFO("Button h4_down pressed");
    button_h4_down_state.data = 1;
  }
  else
    button_h4_down_state.data = 0;

  if(joy.buttons[button_h4_left] == 1)
  {
    ROS_INFO("Button h4_left pressed");
    button_h4_left_state.data = 1;
  }
  else
    button_h4_left_state.data = 0;

  if(joy.buttons[button_ki_up] == 1)
  {
    ROS_INFO("Button ki_up pressed");
    button_ki_up_state.data = 1;
  }
  else
    button_ki_up_state.data = 0;

  if(joy.buttons[button_ki_down] == 1)
  {
    ROS_INFO("Button ki_down pressed");
    button_ki_down_state.data = 1;
  }
  else
    button_ki_down_state.data = 0;

 if(joy.buttons[button_thumb_stick] == 1)
  {
    ROS_INFO("Button thumb_stick pressed");
    button_thumb_stick_state.data = 1;
  }
  else
    button_thumb_stick_state.data = 0;

  if(joy.buttons[button_thumb_slider] == 1)
  {
    ROS_INFO("Button thumb_slider pressed");
    button_thumb_slider_state.data = 1;
  }
  else
    button_thumb_slider_state.data = 0;

  if(joy.buttons[button_scroll_forward] == 1)
  {
    ROS_INFO("Button scroll_forward pressed");
    button_scroll_forward_state.data = 1;
  }
  else
    button_scroll_forward_state.data = 0;

  if(joy.buttons[button_scroll_reverse] == 1)
  {
    ROS_INFO("Button scroll_reverse pressed");
    button_scroll_reverse_state.data = 1;
  }
  else
    button_scroll_reverse_state.data = 0;

  if(joy.buttons[button_m1] == 1)
  {
    ROS_INFO("Button m1 pressed");
    button_m1_state.data = 1;
  }
  else
    button_m1_state.data = 0;

  if(joy.buttons[button_m2] == 1)
  {
    ROS_INFO("Button m2 pressed");
    button_m2_state.data = 1;
  }
  else
    button_m2_state.data = 0;

  if(joy.buttons[button_s1] == 1)
  {
    ROS_INFO("Button s1 pressed");
    button_s1_state.data = 1;
  }
  else
    button_s1_state.data = 0;
*/


/*
____________NEEDS TESTING_____________

*/

}

//main loop
int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"controller_processing");
  ros::NodeHandle n;
  
  //set up subscriptions
  ros::Subscriber joystic_topic = n.subscribe("joystick_topic", 1000, joystick_cb);
  ros::Subscriber thruster_topic = n.subscribe("thruster_topic", 1000, thruster_cb);

  //set up publishers
  ros::Publisher trigger_pub = n.advertise<std_msgs::Bool>("trigger", 1000);
  ros::Publisher button_a_pub = n.advertise<std_msgs::Bool>("button_a", 1000);
  ros::Publisher button_e_pub = n.advertise<std_msgs::Bool>("button_e", 1000);
  ros::Publisher rotary_4_pub = n.advertise<std_msgs::Float32>("rotary_4", 1000);
 // ros::Publisher rotary_3_pub = n.advertise<std_msgs::Float32>("rotary_3", 1000);

/*
Tested
  ros::Publisher joystick_x_pub = n.advertise<std_msgs::Float32>("joystick_x", 1000);
  ros::Publisher joystick_y_pub = n.advertise<std_msgs::Float32>("joystick_y", 1000);
  ros::Publisher thumb_stick_x_pub = n.advertise<std_msgs::Float32>("thumb_stick_x", 1000);
  ros::Publisher thumb_stick_y_pub = n.advertise<std_msgs::Float32>("thumb_stick_y", 1000);
  ros::Publisher joystick_rotation_pub = n.advertise<std_msgs::Float32>("joystick_rotation", 1000);
  ros::Publisher axis_pov_x_pub = n.advertise<std_msgs::Float32>("axis_pov_x", 1000);
  ros::Publisher axis_pov_y_pub = n.advertise<std_msgs::Float32>("axis_pov_y", 1000);

  ros::Publisher button_b_pub = n.advertise<std_msgs::Bool>("button_b", 1000);
  ros::Publisher button_c_pub = n.advertise<std_msgs::Bool>("button_c", 1000);
  ros::Publisher button_d_pub = n.advertise<std_msgs::Bool>("button_d", 1000);
  ros::Publisher button_pinky_trigger_pub = n.advertise<std_msgs::Bool>("button_pinky_trigger", 1000);

  ros::Publisher button_h1_up_pub = n.advertise<std_msgs::Bool>("button_h1_up", 1000);
  ros::Publisher button_h1_right_pub = n.advertise<std_msgs::Bool>("button_h1_right", 1000);
  ros::Publisher button_h1_down_pub = n.advertise<std_msgs::Bool>("button_h1_down", 1000);
  ros::Publisher button_h1_left_pub = n.advertise<std_msgs::Bool>("button_h1_left", 1000);

  ros::Publisher button_h2_up_pub = n.advertise<std_msgs::Bool>("button_h2_up", 1000);
  ros::Publisher button_h2_right_pub = n.advertise<std_msgs::Bool>("button_h2_right", 1000);
  ros::Publisher button_h2_down_pub = n.advertise<std_msgs::Bool>("button_h2_down", 1000);
  ros::Publisher button_h2_left_pub = n.advertise<std_msgs::Bool>("button_h2_left", 1000);

  ros::Publisher axis_left_thruster_pub = n.advertise<std_msgs::Float32>("axis_left_thruster", 1000);
  ros::Publisher axis_right_thruster_pub = n.advertise<std_msgs::Float32>("axis_right_thruster", 1000);

  ros::Publisher axis_rotary_f_pub = n.advertise<std_msgs::Float32>("axis_rotary_f", 1000);
  ros::Publisher axis_rotary_g_pub = n.advertise<std_msgs::Float32>("axis_rotary_g", 1000);

  ros::Publisher axis_thruster_thumb_stick_x_pub = n.advertise<std_msgs::Float32>("ax_thrust_st_x", 1000);
  ros::Publisher axis_thruster_thumb_stick_y_pub = n.advertise<std_msgs::Float32>("ax_thrust_st_y", 1000);

  ros::Publisher button_rotary_f_pub = n.advertise<std_msgs::Bool>("button_rotary_f", 1000);
  ros::Publisher button_rotary_g_pub = n.advertise<std_msgs::Bool>("button_rotary_g", 1000);

  ros::Publisher button_i_pub = n.advertise<std_msgs::Bool>("button_i", 1000);
  ros::Publisher button_h_pub = n.advertise<std_msgs::Bool>("button_h", 1000);

  ros::Publisher button_sw1_pub = n.advertise<std_msgs::Bool>("button_sw1", 1000);
  ros::Publisher button_sw2_pub = n.advertise<std_msgs::Bool>("button_sw2", 1000);
  ros::Publisher button_sw3_pub = n.advertise<std_msgs::Bool>("button_sw3", 1000);
  ros::Publisher button_sw4_pub = n.advertise<std_msgs::Bool>("button_sw4", 1000);
  ros::Publisher button_sw5_pub = n.advertise<std_msgs::Bool>("button_sw5", 1000);
  ros::Publisher button_sw6_pub = n.advertise<std_msgs::Bool>("button_sw6", 1000);

  ros::Publisher button_tgl1_up_pub = n.advertise<std_msgs::Bool>("button_tgl1_up", 1000);
  ros::Publisher button_tgl1_down_pub = n.advertise<std_msgs::Bool>("button_tgl1_down", 1000);

  ros::Publisher button_tgl2_up_pub = n.advertise<std_msgs::Bool>("button_tgl2_up", 1000);
  ros::Publisher button_tgl2_down_pub = n.advertise<std_msgs::Bool>("button_tgl2_down", 1000);

  ros::Publisher button_tgl3_up_pub = n.advertise<std_msgs::Bool>("button_tgl3_up", 1000);
  ros::Publisher button_tgl3_down_pub = n.advertise<std_msgs::Bool>("button_tgl3_down", 1000);

  ros::Publisher button_tgl4_up_pub = n.advertise<std_msgs::Bool>("button_tgl4_up", 1000);
  ros::Publisher button_tgl4_down_pub = n.advertise<std_msgs::Bool>("button_tgl4_down", 1000);

  ros::Publisher button_h3_up_pub = n.advertise<std_msgs::Bool>("button_h3_up", 1000);
  ros::Publisher button_h3_right_pub = n.advertise<std_msgs::Bool>("button_h3_right", 1000);
  ros::Publisher button_h3_down_pub = n.advertise<std_msgs::Bool>("button_h3_down", 1000);
  ros::Publisher button_h3_left_pub = n.advertise<std_msgs::Bool>("button_h3_left", 1000);
 
  ros::Publisher button_h4_up_pub = n.advertise<std_msgs::Bool>("button_h4_up", 1000);
  ros::Publisher button_h4_right_pub = n.advertise<std_msgs::Bool>("button_h4_right", 1000);
  ros::Publisher button_h4_down_pub = n.advertise<std_msgs::Bool>("button_h4_down", 1000);
  ros::Publisher button_h4_left_pub = n.advertise<std_msgs::Bool>("button_h4_left", 1000);

  ros::Publisher button_ki_up_pub = n.advertise<std_msgs::Bool>("button_ki_up", 1000);
  ros::Publisher button_ki_down_pub = n.advertise<std_msgs::Bool>("button_ki_down", 1000);

  ros::Publisher button_thumb_stick_pub = n.advertise<std_msgs::Bool>("button_thumb_stick", 1000);
  ros::Publisher button_thumb_slider_pub = n.advertise<std_msgs::Bool>("button_thumb_slider", 1000);

  ros::Publisher button_scroll_forward_pub = n.advertise<std_msgs::Bool>("button_scroll_forward", 1000);
  ros::Publisher button_scroll_reverse_pub = n.advertise<std_msgs::Bool>("button_scroll_reverse", 1000);

  ros::Publisher button_m1_pub = n.advertise<std_msgs::Bool>("button_m1", 1000);
  ros::Publisher button_m2_pub = n.advertise<std_msgs::Bool>("button_m2", 1000);
  ros::Publisher button_s1_pub = n.advertise<std_msgs::Bool>("button_s1", 1000);
*/

/*
____________NEEDS TESTING_____________


 */
 ros::Rate loop_wait(30);//this is needed
  
  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  {  
    //publish everything once per loop
    trigger_pub.publish(button_trigger_state);
    button_a_pub.publish(button_a_state);
    button_e_pub.publish(button_e_state);
    rotary_4_pub.publish(rotary_4_value); 
/* 
_____________TESTED_____________

    rotary_3_pub.publish(rotary_3_value);  
    joystick_x_pub.publish(axis_stick_x_value);   
    joystick_y_pub.publish(axis_stick_y_value);
    thumb_stick_x_pub.publish(axis_joystick_thumb_stick_x_value);
    thumb_stick_y_pub.publish(axis_joystick_thumb_stick_y_value);
    joystick_rotation_pub.publish(axis_stick_rotation_value);
    axis_pov_x_pub.publish(axis_pov_x_value);
    axis_pov_y_pub.publish(axis_pov_y_value);

    button_b_pub.publish(button_b_state);
    button_c_pub.publish(button_c_state);
    button_d_pub.publish(button_d_state);
    button_pinky_trigger_pub.publish(button_pinky_trigger_state);

    button_h1_up_pub.publish(button_h1_up_state);
    button_h1_right_pub.publish(button_h1_right_state);
    button_h1_down_pub.publish(button_h1_down_state);
    button_h1_left_pub.publish(button_h1_left_state);

    button_h2_up_pub.publish(button_h2_up_state);
    button_h2_right_pub.publish(button_h2_right_state);
    button_h2_down_pub.publish(button_h2_down_state);
    button_h2_left_pub.publish(button_h2_left_state);

    axis_left_thruster_pub.publish(axis_left_thruster_value);
    axis_right_thruster_pub.publish(axis_right_thruster_value);

    axis_rotary_f_pub.publish(axis_rotary_f_value);
    axis_rotary_g_pub.publish(axis_rotary_g_value);

    axis_thruster_thumb_stick_x_pub.publish(axis_thruster_thumb_stick_x_value);
    axis_thruster_thumb_stick_y_pub.publish(axis_thruster_thumb_stick_y_value);

    button_rotary_f_pub.publish(button_rotary_f_state);
    button_rotary_g_pub.publish(button_rotary_g_state);


    button_i_pub.publish(button_i_state);
    button_h_pub.publish(button_h_state);

    button_sw1_pub.publish(button_sw1_state);
    button_sw2_pub.publish(button_sw2_state);
    button_sw3_pub.publish(button_sw3_state);
    button_sw4_pub.publish(button_sw4_state);
    button_sw5_pub.publish(button_sw5_state);
    button_sw6_pub.publish(button_sw6_state);

    button_tgl1_up_pub.publish(button_tgl1_up_state);
    button_tgl1_down_pub.publish(button_tgl1_down_state);

    button_tgl2_up_pub.publish(button_tgl2_up_state);
    button_tgl2_down_pub.publish(button_tgl2_down_state);

    button_tgl3_up_pub.publish(button_tgl3_up_state);
    button_tgl3_down_pub.publish(button_tgl3_down_state);

    button_tgl4_up_pub.publish(button_tgl4_up_state);
    button_tgl4_down_pub.publish(button_tgl4_down_state);

    button_h3_up_pub.publish(button_h3_up_state);
    button_h3_right_pub.publish(button_h3_right_state);
    button_h3_down_pub.publish(button_h3_down_state);
    button_h3_left_pub.publish(button_h3_left_state);

    button_h4_up_pub.publish(button_h4_up_state);
    button_h4_right_pub.publish(button_h4_right_state);
    button_h4_down_pub.publish(button_h4_down_state);
    button_h4_left_pub.publish(button_h4_left_state);

    button_ki_up_pub.publish(button_ki_up_state);
    button_ki_down_pub.publish(button_ki_down_state);

    button_thumb_stick_pub.publish(button_thumb_stick_state);
    button_thumb_slider_pub.publish(button_thumb_slider_state);

    button_scroll_forward_pub.publish(button_scroll_forward_state);
    button_scroll_reverse_pub.publish(button_scroll_reverse_state);

    button_m1_pub.publish(button_m1_state);
    button_m2_pub.publish(button_m2_state);
    button_s1_pub.publish(button_s1_state);
*/

/*
____________NEEDS TESTING_____________
*/
    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}
