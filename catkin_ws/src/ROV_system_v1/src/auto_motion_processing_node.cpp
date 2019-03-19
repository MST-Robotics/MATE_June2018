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

  ros::Subscriber auto_topic = n.subscribe("auto_topic", 1000, auto_callback);
  ros::Subscriber throttle_subscriber = n.subscribe("thruster_topic", 1000, thruster_callback);//subscribe to throttle values
  ros::Publisher joystick_pub = n.advertise<std_msgs::Joy>("joystick_topic", 1000);

  ros::Rate loop_wait(20);//this is needed

  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  {
    if (autonomous_mode && movement_needed)
    {
      movement_needed = false;
      joystick_pub.publish(movement);
      ros::spinOnce();
    }
    loop_wait.sleep();
  }

  return 0;
}

void auto_callback(const std_msgs::Int16 &msg)
{
  switch (msg.data)
  {
    case move_up:
      movement.data = MOVEMENT_UP;
      break;
    case move_down:
      movement.data = MOVEMENT_DOWN;
      break;
    case move_left:
      movement.data = MOVEMENT_LEFT;
      break;
    case move_right:
      movement.data = MOVEMENT_RIGHT;
      break;
    default:
      //error condition
      break;
  }
  movement_needed = true;
}

void thruster_callback(const sensor_msgs::Joy &joy)
{
  if(joy.buttons[button_auto] == 1)
  {
    autonomous_mode = true;
  }
  else
  {
    autonomous_mode = false;
  }
}
