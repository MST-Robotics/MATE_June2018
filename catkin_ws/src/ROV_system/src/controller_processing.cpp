/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst,edu
 * Date: 6-19-2017
 * 
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
#include "messages.h"
#include "cp_joystick_functions.cpp"
#include "cp_thruster_functions.cpp"


//main loop
int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"controller_processing");//set up node named
  ros::NodeHandle n;//create node handle instance
  
  //set up subscriptions
  ros::Subscriber joystic_topic = n.subscribe("joystick_topic", 1000, joystick_cb);//subscribe to joystick values
  ros::Subscriber thruster_topic = n.subscribe("thruster_topic", 1000, thruster_cb);//subscribe to thruster values

  //set up publishers
  ros::Publisher joystick_x_pub = n.advertise<std_msgs::Float32>("joystick_x", 1000);
  ros::Publisher joystick_y_pub = n.advertise<std_msgs::Float32>("joystick_y", 1000);
  ros::Publisher joystick_rotation_pub = n.advertise<std_msgs::Float32>("joystick_rotation", 1000);
  ros::Publisher trigger_pub = n.advertise<std_msgs::Bool>("trigger", 1000);
  ros::Publisher button_pinky_trigger_pub = n.advertise<std_msgs::Bool>("button_pinky_trigger", 1000);

 ros::Rate loop_wait(30);//this is needed
  
  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  {  
    //publish everything once per loop

   joystick_x_pub.publish(axis_stick_x_value);   
   joystick_y_pub.publish(axis_stick_y_value);
   joystick_rotation_pub.publish(axis_stick_rotation_value);
 
   trigger_pub.publish(button_trigger_state);
   button_pinky_trigger_pub.publish(button_pinky_trigger_state);

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}


