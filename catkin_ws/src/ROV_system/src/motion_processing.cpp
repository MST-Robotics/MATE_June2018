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
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>

float magnitude = 0.0;
std_msgs::Int16 motor1_value;
std_msgs::Int16 motor2_value;
std_msgs::Int16 motor3_value;
std_msgs::Int16 motor4_value;
std_msgs::Int16 motor5_value;
std_msgs::Int16 motor6_value;

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


void velocity_cb(const std_msgs::Float32 &msg)
{ 
  magnitude = msg.data;
}

void trigger_cb(const std_msgs::Bool &msg)//going up
{
  int trigger = msg.data;
  if(trigger == 1)
  {
    motor2_value.data = 2000;
    motor5_value.data = 2000;
  }
  else
  {
    motor2_value.data = 1500;
    motor5_value.data = 1500;
  }
}
void button_pinky_trigger_cb(const std_msgs::Bool &msg)//going down
{
  int pinky = msg.data;
  if(pinky == 1)
  {
    motor2_value.data = 2000;
    motor5_value.data = 2000;
  }
  else
  {
    motor2_value.data = 1500;
    motor5_value.data = 1500;
  }
}
void angle_cb(const std_msgs::Float32 &msg)
{ 
//BASIC DIRECTIONS

  if(msg.data > 85 && msg.data < 95)//forwards
  {
     motor1_value.data = 1500+(magnitude*400);
     motor4_value.data = 1500+(magnitude*400);
  }
  else if(msg.data > 265 && msg.data < 275)//backwards
  {
      motor1_value.data = 1500+(magnitude*-400);
      motor4_value.data = 1500+(magnitude*-400);
  }

  else if(msg.data > 175 && msg.data < 185)//left
  {
     motor4_value.data = 1500+(magnitude*400);
     motor6_value.data = 1500+(magnitude*400); 
  }

  else if(msg.data >= 0 && msg.data <= 5 || msg.data >= 355 && msg.data <= 359)//right
  {
      motor1_value.data = 1500+(magnitude*400);
      motor3_value.data = 1500+(magnitude*400);
  }

//*************************************************************************
/*
  if(msg.data > 95 && msg.data < 175)//forward left
  {
     motor6_value.data = 1500+(magnitude*-400); 
     motor3_value.data = 1500+(magnitude*400); 
     motor4_value.data = 1500+(magnitude*400); 
  }
/*
  if(msg.data > 5 && msg.data < 85)//forward right
  {
     motor1_value.data = 1500+(magnitude*-400);
     motor3_value.data = 1500+(magnitude*400); 
     motor4_value.data = 1500+(magnitude*400);
  }
  if(msg.data > 185 && msg.data < 265)//backward right
  {
     motor1_value.data = 1500+(magnitude*-400); 
     motor2_value.data = 1500+(magnitude*400);
     motor3_value.data = 1500+(magnitude*400);
  }
  if(msg.data > 275 && msg.data < 355)//backward right
  {
     motor1_value.data = 1500+(magnitude*-400); 
     motor2_value.data = 1500+(magnitude*400);
     motor4_value.data = 1500+(magnitude*400);
  }
*/
  else
  {
      motor1_value.data = 1500;
      motor3_value.data = 1500;
      motor4_value.data = 1500;
      motor6_value.data = 1500;
  }
}


//main loop
int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"motion_processing");
  ros::NodeHandle n;

  //subscribe to current sensor data
  ros::Subscriber joystick_x_topic = n.subscribe("joystick_x", 1000, velocity_cb);
  ros::Subscriber joystick_y_topic = n.subscribe("joystick_y", 1000, angle_cb);



  ros::Subscriber trigger_topic = n.subscribe("trigger", 1000, trigger_cb);
  ros::Subscriber button_pinky_trigger_topic = n.subscribe("button_pinky_trigger", 1000, button_pinky_trigger_cb);

  ros::Publisher motor1_pub = n.advertise<std_msgs::Int16>("motor1_topic", 1000);
  ros::Publisher motor2_pub = n.advertise<std_msgs::Int16>("motor2_topic", 1000);
  ros::Publisher motor3_pub = n.advertise<std_msgs::Int16>("motor3_topic", 1000);
  ros::Publisher motor4_pub = n.advertise<std_msgs::Int16>("motor4_topic", 1000);
  ros::Publisher motor5_pub = n.advertise<std_msgs::Int16>("motor5_topic", 1000);
  ros::Publisher motor6_pub = n.advertise<std_msgs::Int16>("motor6_topic", 1000);

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
    motor1_pub.publish(motor1_value);
    motor2_pub.publish(motor2_value);
    motor3_pub.publish(motor3_value);
    motor4_pub.publish(motor4_value);
    motor5_pub.publish(motor5_value);
    motor6_pub.publish(motor6_value);

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}
