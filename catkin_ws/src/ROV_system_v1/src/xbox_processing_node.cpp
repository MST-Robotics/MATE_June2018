/*
 * Author: Tamara Spivey
 * Email: tjsxz4@mst.edu
 * 
 * This file handles data from the xbox 360 controller
 */

#include "xbox_processing_node.h"



int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"xbox_processing_node");//set up node named
  ros::NodeHandle n;//create node handle instance
  
  //set up subscriptions
  ros::Subscriber xbox_topic = n.subscribe("xbox_topic", 1000, xbox_callback);//subscribe to joystick values

  //setting up publishers
  ros::Publisher button_A_pub = n.advertise<std_msgs::Bool>("button_A_topic", 1000);
  ros::Publisher button_B_pub = n.advertise<std_msgs::Bool>("button_B_topic", 1000);
  ros::Publisher button_X_pub = n.advertise<std_msgs::Bool>("button_X_topic", 1000);
  ros::Publisher button_Y_pub = n.advertise<std_msgs::Bool>("button_Y_topic", 1000);
  ros::Publisher button_LB_pub = n.advertise<std_msgs::Bool>("button_LB_topic", 1000);
  ros::Publisher button_RB_pub = n.advertise<std_msgs::Bool>("button_RB_topic", 1000);
  ros::Publisher button_back_pub = n.advertise<std_msgs::Bool>("button_back_topic", 1000);
  ros::Publisher button_start_pub = n.advertise<std_msgs::Bool>("button_start_topic", 1000);

  ros::Publisher button_power_pub = n.advertise<std_msgs::Bool>("button_power_topic", 1000);
  ros::Publisher button_stick_left_pub = n.advertise<std_msgs::Bool>("button_stick_left_topic", 1000);
  ros::Publisher button_stick_right_pub = n.advertise<std_msgs::Bool>("button_stick_right_topic", 1000);
//axis publishers
  ros::Publisher left_xbox_stick_x_pub = n.advertise<std_msgs::Float32>("left_xbox_magnitude_topic", 1000);
  ros::Publisher left_xbox_stick_y_pub = n.advertise<std_msgs::Float32>("left_xbox_angle_topic", 1000);

  ros::Publisher right_xbox_stick_x_pub = n.advertise<std_msgs::Float32>("right_xbox_magnitude_topic", 1000);
  ros::Publisher right_xbox_stick_y_pub = n.advertise<std_msgs::Float32>("right_xbox_angle_topic", 1000);

  ros::Publisher axis_RT_pub = n.advertise<std_msgs::Float32>("axis_RT_topic", 1000);
  ros::Publisher axis_LT_pub = n.advertise<std_msgs::Float32>("axis_LT_topic", 1000);

  ros::Publisher axis_cross_key_x_pub = n.advertise<std_msgs::Float32>("axis_cross_key_x_topic", 1000);
  ros::Publisher axis_cross_key_y_pub = n.advertise<std_msgs::Float32>("axis_cross_key_y_topic", 1000);
//TO DO: set up publishers for the axis controls

  ros::Rate loop_wait(30);//this is needed
 
  while(ros::ok()) //ctrl-c makes ok() return false, thus ending the program
  {  
   button_A_pub.publish(button_A_state);
   button_B_pub.publish(button_B_state);
   button_X_pub.publish(button_X_state);
   button_Y_pub.publish(button_Y_state);
   button_LB_pub.publish(button_LB_state);
   button_RB_pub.publish(button_RB_state);
   button_back_pub.publish(button_back_state);
   button_start_pub.publish(button_start_state);
   button_power_pub.publish(button_power_state);
   button_stick_left_pub.publish(button_stick_left_state);
   button_stick_right_pub.publish(button_stick_right_state);
   
   left_xbox_stick_x_pub.publish(left_xbox_magnitude_value);   
   left_xbox_stick_y_pub.publish(left_xbox_angle_value);

   right_xbox_stick_x_pub.publish(right_xbox_magnitude_value);   
   right_xbox_stick_y_pub.publish(right_xbox_angle_value);

   axis_RT_pub.publish(axis_RT_value);
   axis_LT_pub.publish(axis_LT_value);

   axis_cross_key_x_pub.publish(axis_cross_key_x_value);
   axis_cross_key_y_pub.publish(axis_cross_key_y_value);

   ros::spinOnce();
   loop_wait.sleep();//wait some
  }
  return 0;
}


/* joystick_callback handles data recieved from the joystick topic
 * Pre: Joystick topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration
 */
void xbox_callback(const sensor_msgs::Joy &joy)
{
  left_xbox_angle_value.data = atan2(joy.axes[left_axis_stick_y],  joy.axes[left_axis_stick_x]*-1);
  left_xbox_magnitude_value.data = sqrt(pow(joy.axes[left_axis_stick_y],2) + pow(joy.axes[left_axis_stick_x],2));
  //axis_stick_rotation_value.data = joy.axes[axis_stick_rotation];

  left_xbox_angle_value.data *= 180/M_PI; //built in value for Pi

  if(left_xbox_angle_value.data < 0) //conversion to positive degree value
  {
    left_xbox_angle_value.data += 360;
  }

  if(left_xbox_magnitude_value.data > 1.0) //magnitude is a scale of our speed, from [0,1]
  {
    left_xbox_magnitude_value.data = 1.0; //mapped to one to match our scale
  }
  if(left_xbox_magnitude_value.data < 0.0)
  {
    left_xbox_magnitude_value.data = 0.0; //mapped to zero to match our scale 
  }

  right_xbox_angle_value.data = atan2(joy.axes[right_axis_stick_y],  joy.axes[right_axis_stick_x]*-1);
  right_xbox_magnitude_value.data = sqrt(pow(joy.axes[right_axis_stick_y],2) + pow(joy.axes[right_axis_stick_x],2));

  right_xbox_angle_value.data *= 180/M_PI; //built in value for Pi

  if(right_xbox_angle_value.data < 0) //conversion to positive degree value
  {
    right_xbox_angle_value.data += 360;
  }

  if(right_xbox_magnitude_value.data > 1.0) //magnitude is a scale of our speed, from [0,1]
  {
    right_xbox_magnitude_value.data = 1.0; //mapped to one to match our scale
  }
  if(right_xbox_magnitude_value.data < 0.0)
  {
    right_xbox_magnitude_value.data = 0.0; //mapped to zero to match our scale 
  }


  button_A_state.data = joy.buttons[button_A];
  button_B_state.data = joy.buttons[button_B];
  button_X_state.data = joy.buttons[button_X];
  button_Y_state.data = joy.buttons[button_Y];
  button_LB_state.data = joy.buttons[button_LB];
  button_RB_state.data = joy.buttons[button_RB];
  button_back_state.data = joy.buttons[button_back];
  button_start_state.data = joy.buttons[button_start];
  button_power_state.data = joy.buttons[button_power];
  button_stick_left_state.data = joy.buttons[button_stick_left];
  button_stick_right_state.data = joy.buttons[button_stick_right];

 // left_axis_stick_x_value.data = joy.axes[left_axis_stick_x]
 // left_axis_stick_y_value.data = joy.axes[left_axis_stick_y]

 // right_axis_stick_x_value.data = joy.axes[right_axis_stick_x]
 // right_axis_stick_y_value.data = joy.axes[right_axis_stick_y]

  axis_RT_value.data = joy.axes[axis_RT];
  axis_LT_value.data = joy.axes[axis_LT];

  axis_cross_key_x_value.data = joy.axes[axis_cross_key_x];
  axis_cross_key_y_value.data = joy.axes[axis_cross_key_y];
}
