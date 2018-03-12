/*
 * Author: Tamara Spivey
 * Email: tjsxz4@mst.edu
 * Date: 01-10-2018
 * 
 * 
 */

#include "arm_motion_processing_node.h"
int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"arm_motion_processing");
  ros::NodeHandle n;
 
  ros::Rate loop_wait(30);//this is needed
  
  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  { 

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}
