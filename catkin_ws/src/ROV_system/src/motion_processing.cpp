/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst,edu
 * Date: 10-12-2017
 * 
 * This node is for handling motion processing
 *	This node will publish values used for controlling the motors on the main board
 *
 */

//required for use of ros
#include "ros/ros.h"

/*ros message data types can be 
 * found at: http://wiki.ros.org/std_msgs
 */
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>

void current0_cb(const std_msgs::Int16 &msg)
{
 
}

void current1_cb(const std_msgs::Int16 &msg)
{
  
}

void current2_cb(const std_msgs::Int16 &msg)
{
  
}

void current3_cb(const std_msgs::Int16 &msg)
{
 
}

void current4_cb(const std_msgs::Int16 &msg)
{

}

void current5_cb(const std_msgs::Int16 &msg)
{

}

void orientation_cb(const geometry_msgs::Vector3 &msg)
{

}

//main loop
int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"motion_processing");
  ros::NodeHandle n;

  //subscribe to current sensor data
  ros::Subscriber current0_topic = n.subscribe("current0_topic", 1000, current0_cb);
  ros::Subscriber current1_topic = n.subscribe("current1_topic", 1000, current1_cb);
  ros::Subscriber current2_topic = n.subscribe("current2_topic", 1000, current2_cb);
  ros::Subscriber current3_topic = n.subscribe("current3_topic", 1000, current3_cb);
  ros::Subscriber current4_topic = n.subscribe("current4_topic", 1000, current4_cb);
  ros::Subscriber current5_topic = n.subscribe("current5_topic", 1000, current5_cb);
  
  ros::Subscriber orientation_topic = n.subscribe("orientation_topic", 1000, orientation_cb);
 
  ros::Rate loop_wait(30);//this is needed
  
  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  {  
    

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}
