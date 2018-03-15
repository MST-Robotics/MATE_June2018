/*
 * Author: Tamara Spivey
 * Email: tjsxz4@mst.edu
 * Date: 01-10-2018
 * 
 * 
 */
//MOVE THESE DEFINES TO THE HEADER FILE
#define XBOX_FORCE_X_MODIFIER 1 /*To Be Determined*/
#define XBOX_FORCE_Y_MODIFIER 1 /*To Be Determined*/
#define XBOX_MOMENT_MODIFIER 1 /*To Be Determined*/
#define ARM_MOTOR_NEUTRAL 1500
#define ARM_MOTOR_RAMP 400
#include "arm_motion_processing_node.h"
int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"arm_motion_processing");
  ros::NodeHandle n;
 
  //ros::Subscriber button_A_topic = n.subscribe("button_A_topic", 1000, button_A_callback);



  ros::Publisher arm_motor1_pub = n.advertise<std_msgs::Int16>("arm_motor1_topic", 1000);
  ros::Publisher arm_motor2_pub = n.advertise<std_msgs::Int16>("arm_motor2_topic", 1000);
  ros::Publisher arm_motor3_pub = n.advertise<std_msgs::Int16>("arm_motor3_topic", 1000);
  ros::Publisher arm_motor4_pub = n.advertise<std_msgs::Int16>("arm_motor4_topic", 1000);
  ros::Publisher arm_motor5_pub = n.advertise<std_msgs::Int16>("arm_motor5_topic", 1000);
  ros::Publisher arm_motor6_pub = n.advertise<std_msgs::Int16>("arm_motor6_topic", 1000);


  ros::Rate loop_wait(30);//this is needed
  
  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  { 

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}
