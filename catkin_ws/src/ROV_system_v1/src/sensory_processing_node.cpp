/*
 * Author: Vinnie Marco, Megan Cash
 * Email: vgmcn3@mst,edu, mecfkc@mst.edu
 * Date: 02-22-2018
 *
 * This handles all data for the sensors
 */

#include "sensory_processing_node.h"



int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv, "sensory_processing_node");
  ros::NodeHandle n;

  //Subscribers for raw accel and mag data
  ros::Subscriber accel_topic = n.subscribe("accel_topic", 1000, accel_cb);
  ros::Subscriber mag_topic = n.subscribe("mag_topic", 1000, mag_cb);
  ros::Subscriber pixy_topic = n.subscribe("pixy_topic", 1000, pixy_cb);

  //Publisher for orientation in x=roll, y=pitch, z=heading
  ros::Publisher orientation_pub = n.advertise<geometry_msgs::Vector3>("orientation_topic", 1000);
  ros::Publisher pixy_pub = n.advertise<std_msgs::Char>("pixy_topic", 1000);

  ros::Rate loop_wait(30);//this is needed

  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  {
    orientation_pub.publish(orientation);
    pixy_pub.publish(plane_type);

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
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

void pixy_cb(const geometry_msgs::Vector3 &msg)
{
  float temp;
  float ratio;
  color = msg.x;
  width = msg.y;
  height = msg.z;

  if (width > height)
  {
    temp = height;
    height = width;
    width = temp;
  }

  //ratio is height divided by width
  ratio = height/width;

  if (color == red)
  {
    if (ratio <= triangle_ratio+tolerance && ratio >= triangle_ratio-tolerance)
    {
      plane_type.data = 'A';
    }

    else
    {
      plane_type.data = 'D';
    }
  }
  else if (color == blue)
  {
    if (ratio <= triangle_ratio+tolerance && ratio >= triangle_ratio-tolerance)
    {
      plane_type.data = 'C';
    }

    else
    {
      plane_type.data = 'F';
    }
  }
  else if (color == yellow)
  {
    if (ratio <= triangle_ratio+tolerance && ratio >= triangle_ratio-tolerance)
    {
      plane_type.data = 'B';
    }

    else
    {
      plane_type.data = 'E';
    }
  }
}
