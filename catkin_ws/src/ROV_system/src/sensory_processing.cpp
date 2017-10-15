/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst,edu
 * Date: 10-12-2017
 * 
 * This node is for handling sensory processing
 *
 */


//required for use of ros
#include "ros/ros.h"

/*ros message data types can be 
 * found at: http://wiki.ros.org/std_msgs
 */
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

//these values used for calcualting current
#define voltage (5000/1024)
#define center_point 2500
#define mv_a 66

#define PI 3.14159

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

std_msgs::Int16 current0;
std_msgs::Int16 current1;
std_msgs::Int16 current2;
std_msgs::Int16 current3;
std_msgs::Int16 current4;
std_msgs::Int16 current5;


//value to store orientation
geometry_msgs::Vector3 orientation;


uint16_t get_current(uint16_t sensor_value);

void channel0_cb(const std_msgs::Int16 &msg)
{
   current0.data = get_current(msg.data);
}

void channel1_cb(const std_msgs::Int16 &msg)
{
  current1.data = get_current(msg.data);
}

void channel2_cb(const std_msgs::Int16 &msg)
{
  current2.data = get_current(msg.data);
}

void channel3_cb(const std_msgs::Int16 &msg)
{
  current3.data = get_current(msg.data);
}

void channel4_cb(const std_msgs::Int16 &msg)
{
  current4.data = get_current(msg.data);
}

void channel5_cb(const std_msgs::Int16 &msg)
{
  current5.data = get_current(msg.data);
}

void accel_cb(const geometry_msgs::Vector3 &msg)
{
  //calculate roll
  orientation.x = (atan2(msg.y, msg.z));
  orientation.x *= 180.0/PI;//convert to degrees
  
  //calculate pitch
  orientation.y = atan2(-1*msg.x, sqrt(msg.y * msg.y + msg.z * msg.z));
  orientation.y *= 180.0/PI;//convert to degrees
}

void mag_cb(const geometry_msgs::Vector3 &msg)
{
  //calculate heading math came from sparkfun library
  if(msg.y == 0)
    orientation.z = (msg.x < 0) ? PI : 0;
  else
    orientation.z = atan2(msg.x, msg.y);
   
  orientation.z -= DECLINATION * PI/180;

  if (orientation.z > PI) 
    orientation.z -= (2 * PI);
  else if (orientation.z < -PI) 
    orientation.z += (2 * PI);
  else if (orientation.z < 0) 
    orientation.z += 2 * PI;

  orientation.z *= 180.0/PI;//convert to degrees
}

//main loop
int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"sensory_processing");
  ros::NodeHandle n;
  
  //subscribers for raw adc values from current sensor board
  ros::Subscriber channel0_topic = n.subscribe("channel0_topic", 1000, channel0_cb);
  ros::Subscriber channel1_topic = n.subscribe("channel1_topic", 1000, channel1_cb);
  ros::Subscriber channel2_topic = n.subscribe("channel2_topic", 1000, channel2_cb);
  ros::Subscriber channel3_topic = n.subscribe("channel3_topic", 1000, channel3_cb);
  ros::Subscriber channel4_topic = n.subscribe("channel4_topic", 1000, channel4_cb);
  ros::Subscriber channel5_topic = n.subscribe("channel5_topic", 1000, channel5_cb);

  //publishers for calcualted currents from current sensor board
  ros::Publisher current0_pub = n.advertise<std_msgs::Int16>("current0_topic", 1000);
  ros::Publisher current1_pub = n.advertise<std_msgs::Int16>("current1_topic", 1000);
  ros::Publisher current2_pub = n.advertise<std_msgs::Int16>("current2_topic", 1000);
  ros::Publisher current3_pub = n.advertise<std_msgs::Int16>("current3_topic", 1000);
  ros::Publisher current4_pub = n.advertise<std_msgs::Int16>("current4_topic", 1000);
  ros::Publisher current5_pub = n.advertise<std_msgs::Int16>("current5_topic", 1000); 


  //Subscribers for raw accel and mag data
  ros::Subscriber accel_topic = n.subscribe("accel_topic", 1000, accel_cb);
  ros::Subscriber mag_topic = n.subscribe("mag_topic", 1000, mag_cb);

  //Publisher for orientation in x=roll, y=pitch, z=heading
  ros::Publisher orientation_pub = n.advertise<geometry_msgs::Vector3>("orientation_topic", 1000);

  ros::Rate loop_wait(30);//this is needed
  
  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  {  
    current0_pub.publish(current0);
    current1_pub.publish(current1);
    current2_pub.publish(current2);
    current3_pub.publish(current3);
    current4_pub.publish(current4);
    current5_pub.publish(current5);

    orientation_pub.publish(orientation);

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}

//function for calculating current
uint16_t get_current(uint16_t sensor_value)
{   
  return (((long)sensor_value * voltage) - center_point) * 1000 / mv_a;//calculate the current in (mA)
} 
