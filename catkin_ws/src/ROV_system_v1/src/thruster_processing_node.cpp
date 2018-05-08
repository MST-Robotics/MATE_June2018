/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT
 * Email: vgmcn3@mst,edu, tjsxz4@mst.edu
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
  ros::Publisher sw1_pub = n.advertise<std_msgs::Bool>("sw1_topic", 1000);
  ros::Publisher sw2_pub = n.advertise<std_msgs::Bool>("sw2_topic", 1000);
  ros::Publisher sw3_pub = n.advertise<std_msgs::Bool>("sw3_topic", 1000);
  ros::Publisher sw4_pub = n.advertise<std_msgs::Bool>("sw4_topic", 1000);
  ros::Publisher sw5_pub = n.advertise<std_msgs::Bool>("sw5_topic", 1000);
  ros::Publisher sw6_pub = n.advertise<std_msgs::Bool>("sw6_topic", 1000);
  ros::Publisher m1_pub = n.advertise<std_msgs::Bool>("m1_topic", 1000);

  ros::Publisher axis_left_thruster_pub = n.advertise<std_msgs::Float32>("axis_left_thruster_topic", 1000);
  ros::Publisher setpoint_pub = n.advertise<std_msgs::Int16>("setpoint_topic", 100);
  
  ros::Publisher tgl1_pub = n.advertise<std_msgs::Int16>("tgl1_topic", 1000);

  ros::Rate loop_wait(30);//this is needed
  
  while(ros::ok()) //ctrl-c makes ok() return false, thus ending the program
  {  
    //publish everything once per loop 
    sw1_pub.publish(button_sw1_state);
    sw2_pub.publish(button_sw2_state);
    sw3_pub.publish(button_sw3_state);
    sw4_pub.publish(button_sw4_state);
    sw5_pub.publish(button_sw5_state);
    sw6_pub.publish(button_sw6_state);
    setpoint_pub.publish(setpoint_value);
    
    m1_pub.publish(button_m1_state);

    axis_left_thruster_pub.publish(axis_left_thruster_value);
	
	tgl1_pub.publish(button_tgl1_value);
 
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
  button_sw1_state.data = joy.buttons[button_sw1];
  button_sw2_state.data = joy.buttons[button_sw2];
  button_sw3_state.data = joy.buttons[button_sw3];
  button_sw4_state.data = joy.buttons[button_sw4];
  button_sw5_state.data = joy.buttons[button_sw5];
  button_sw6_state.data = joy.buttons[button_sw6];

  axis_left_thruster_value.data = joy.axes[axis_left_thruster];
  
  button_m1_state.data = joy.buttons[button_m1];


  //the setpoint for the PID is determined here by rty4
  setpoint_value.data = (int)mapf(joy.axes[axis_base_rotary_4], -1.0, 1.0, 200.0, 160.0);

  
  if(joy.buttons[button_tgl1_up] == 1)
  {
    button_tgl1_value.data = 1;
  }

  else if(joy.buttons[button_tgl1_down] == 1)
  {
    button_tgl1_value.data = -1;
  }

  else
  {
    button_tgl1_value.data = 0;
  }
/* 
  Done
  //buttons are stored in joy.buttons[]
  //axes are stored in joy.axis[]

  button_e_state.data = joy.buttons[button_e];
  button_rotary_f_state.data = joy.buttons[button_rotary_f];
  button_rotary_g_state.data = joy.buttons[button_rotary_g];
  button_i_state.data = joy.buttons[button_i];
  button_h_state.data = joy.buttons[button_h];

  button_tgl2_up_state.data = joy.buttons[button_tgl2_up];
  button_tgl2_down_state.data = joy.buttons[button_tgl2_down];

  button_tgl3_up_state.data = joy.buttons[button_tgl3_up];
  button_tgl3_down_state.data = joy.buttons[button_tgl3_down];

  button_tgl4_up_state.data = joy.buttons[button_tgl4_up];
  button_tgl4_down_state.data = joy.buttons[button_tgl4_down];

  button_h3_up_state.data = joy.buttons[button_h3_up];
  button_h3_right_state.data = joy.buttons[button_h3_right];
  button_h3_down_state.data = joy.buttons[button_h3_down];
  button_h3_left_state.data = joy.buttons[button_h3_left];

  button_h4_up_state.data = joy.buttons[button_h4_up];
  button_h4_right_state.data = joy.buttons[button_h4_right];
  button_h4_down_state.data = joy.buttons[button_h4_down];
  button_h4_left_state.data = joy.buttons[button_h4_left];

  button_ki_up_state.data = joy.buttons[button_ki_up];
  button_ki_down_state.data = joy.buttons[button_ki_down];

  button_thumb_stick_state.data = joy.buttons[button_thumb_stick];
  button_thumb_slider_state.data = joy.buttons[button_thumb_slider];

  button_scroll_forward_state.data = joy.buttons[button_scroll_forward];
  button_scroll_reverse_state.data = joy.buttons[button_thumb_reverse];

  button_m1_state.data = joy.buttons[button_m1];
  button_m2_state.data = joy.buttons[button_m2];

  button_s1_state.data = joy.buttons[button_s1];

  rotary_4_value.data = joy.axes[axis_base_rotary_4];
  rotary_3_value.data = joy.axes[axis_base_rotary_3];

  axis_right_thruster_value.data = joy.axes[axis_right_thruster];

  axis_rotary_f_value.data = joy.axes[axis_rotary_f];
  axis_rotary_g_value.data = joy.axes[axis_rotary_g];

  axis_thruster_thumb_stick_x_value.data = joy.axes[axis_thruster_thumb_stick_x];
  axis_thruster_thumb_stick_y_value.data = joy.axes[axis_thruster_thumb_stick_y];
*/
}

//this funciton maps a range of floats to another range of floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

