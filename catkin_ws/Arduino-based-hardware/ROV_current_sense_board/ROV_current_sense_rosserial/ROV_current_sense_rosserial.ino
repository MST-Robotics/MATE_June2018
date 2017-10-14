/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst,edu
 * Date: 10-12-2017
 * 
 * Code for basic implementation of rosserial
 * 
 * This will start all necessary nodes to read both controllers
 * and stream data to the arduino
 */

//required to use ros with arduino
#include <ros.h>

/*ros message data types can be 
 * found at: http://wiki.ros.org/std_msgs
*/
#include <std_msgs/Int16.h> //include required per type of message


//these values used for calcualting current
#define voltage (5000/1024)
#define center_point 2500
#define mv_a 66

//analog pins for the current sensing channels
#define channel0 A0
#define channel1 A1
#define channel2 A2
#define channel3 A3
#define channel4 A4
#define channel5 A5

//create message types for the current values
std_msgs::Int16 channel0_val;
std_msgs::Int16 channel1_val;
std_msgs::Int16 channel2_val;
std_msgs::Int16 channel3_val;
std_msgs::Int16 channel4_val;
std_msgs::Int16 channel5_val;


ros::NodeHandle  nh;//this is necessary too

//set up the publishers for the current values
ros::Publisher channel0_pub("channel0_topic", &channel0_val);
ros::Publisher channel1_pub("channel1_topic", &channel1_val);
ros::Publisher channel2_pub("channel2_topic", &channel2_val);
ros::Publisher channel3_pub("channel3_topic", &channel3_val);
ros::Publisher channel4_pub("channel4_topic", &channel4_val);
ros::Publisher channel5_pub("channel5_topic", &channel5_val);

unsigned long previous_millis = 0;

//Arduino setup
void setup()
{
  //this is needed
  nh.initNode();

  //advertise the publishers
  nh.advertise(channel0_pub);
  nh.advertise(channel1_pub);
  nh.advertise(channel2_pub);
  nh.advertise(channel3_pub);
  nh.advertise(channel4_pub);
  nh.advertise(channel5_pub);
}


void loop()
{ 
  //read the sensors and publish the values
  update_current();
  
  
  nh.spinOnce();
}

void update_current(void)
{
  //update the current values
  channel0_val.data = analogRead(channel0);
  channel1_val.data = analogRead(channel1);
  channel2_val.data = analogRead(channel2);
  channel3_val.data = analogRead(channel3);
  channel4_val.data = analogRead(channel4);
  channel5_val.data = analogRead(channel5);

  //publish the data
  channel0_pub.publish(&channel0_val);
  channel1_pub.publish(&channel1_val);
  channel2_pub.publish(&channel2_val);
  channel3_pub.publish(&channel3_val);
  channel4_pub.publish(&channel4_val);
  channel5_pub.publish(&channel5_val);

  
  return;
}

