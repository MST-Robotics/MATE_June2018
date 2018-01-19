/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT
 * Email: vgmcn3@mst,edu
 * Date: 01-18-2018
 * 
 * This file handles data from the thruster
 */

#include "thruster_processing_node.h"

/*
 * The main function
 * Pre: None
 * Post: Necessary publishers and subscribers are created to read controller data
*/
int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"thruster_processing_node");//set up node 
  ros::NodeHandle n;//create node handle instance
  
  //set up subscriptions
  ros::Subscriber throttle_subscriber = n.subscribe("thruster_topic", 1000, thruster_callback);//subscribe to throttle values

  //set up publishers
  ros::Publisher throttle_publisher = n.advertise<std_msgs::Float32>("throttle_topic", 1000);

  ros::Rate loop_wait(30);//this is needed
  

  while(ros::ok()) //ctr-c makes ok() return false, thus ending the program
  {  
    //publish everything once per loop
  
   throttle_publisher.publish(throttle_value);

   ros::spinOnce();
   loop_wait.sleep();//wait some
  }
  return 0;
}

/*
 * This function handles the thruster data
 * Pre: A reference to the thruster value must created
 * Post: Thruster values are updated
 */
void thruster_callback(const sensor_msgs::Joy &joy)
{
  throttle_value.data = joy.axes[axis_left_thruster];
  
  /* 
  Done
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
}

