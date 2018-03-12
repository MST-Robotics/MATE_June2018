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

  ros::Rate loop_wait(30);//this is needed
 
  while(ros::ok()) //ctrl-c makes ok() return false, thus ending the program
  {  
   button_A_pub.publish(button_A_state);
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

//test for rostopic echo
  button_A_state.data = joy.buttons[button_A]; //Gets value from button
}
