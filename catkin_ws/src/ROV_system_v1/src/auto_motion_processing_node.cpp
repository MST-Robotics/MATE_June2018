/*
 * Author: Matthew Healy
 * Email: matthew@healy.ink
 * Date: 03-18-2018
 *
 *
 */
#include "auto_motion_processing_node.h"

bool movement_needed = false;

int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"auto_motion_processing");
  ros::NodeHandle n;

  ros::Subscriber sw_topic = n.subscribe("swx_topic", 1000, sw_callback);
  ros::Subscriber auto_topic = n.subscribe("auto_topic", 1000, auto_callback);

  ros::Publisher joystick_x_pub = n.advertise<std_msgs::Float32>("magnitude_topic", 1000);
  ros::Publisher joystick_y_pub = n.advertise<std_msgs::Float32>("angle_topic", 1000);
  ros::Publisher joystick_rotation_pub = n.advertise<std_msgs::Float32>("joystick_rotation_topic", 1000);


  ros::Rate loop_wait(20);//this is needed

  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  {
    if (autonomous_mode && movement_needed)
    {
      movement_needed = false;
      controls.publish(movement);
      ros::spinOnce();
    }
    loop_wait.sleep();//wait some
  }
  return 0;
}

void auto_callback(const std_msgs::Int16 &msg)
{
  switch (msg.data)
  {
    case UP:
      break;
    case DOWN:
      break;
    case LEFT:
      break;
    case RIGHT:
      break;
    default:
      //error condition
      break;
  }
  movement_needed = true;
}

void sw_callback(const std_msgs::Bool &msg)
{
  // increment wrist
  if(msg.data == 1)
  {
    autonomous_mode = true;
  }
  else
  {
    autonomous_mode = false;
  }
}
