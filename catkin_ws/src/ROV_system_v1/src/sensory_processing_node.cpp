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

  //Subscribers
  ros::Subscriber pixy_topic = n.subscribe("pixy_data_topic", 1000, pixy_cb);
  ros::Subscriber raw_temp_topic = n.subscribe("raw_temp_topic", 1000, raw_temp_cb);

  //Publishers
  ros::Publisher pixy_pub = n.advertise<std_msgs::String>("pixy_result_topic", 1000);
  ros::Publisher temp_result_pub = n.advertise<std_msgs::Float32>("temp_result_topic", 1000);

  ros::Rate loop_wait(30);//this is needed

  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  {
    pixy_pub.publish(plane_type);
    temp_result_pub.publish(temp_F);

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}

void pixy_cb(const geometry_msgs::Vector3 &msg)
{
  float temp;
  float ratio;
  int color = msg.x;
  float width = msg.y;
  float height = msg.z;

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
      plane_type.data = "Plane A: red triangle";
    }

    else
    {
      plane_type.data = "Plane D: red rectangle";
    }
  }
  else if (color == blue)
  {
    if (ratio <= triangle_ratio+tolerance && ratio >= triangle_ratio-tolerance)
    {
      plane_type.data = "Plane C: blue triangle";
    }

    else
    {
      plane_type.data = "Plane F: blue rectangle";
    }
  }
  else if (color == yellow)
  {
    if (ratio <= triangle_ratio+tolerance && ratio >= triangle_ratio-tolerance)
    {
      plane_type.data = "Plane B: yellow triangle";
    }

    else
    {
      plane_type.data = "Plane E: yellow rectangle";
    }
  }
}

void raw_temp_cb(const std_msgs::Float32 &msg)
{
  float voltage = 0.0;
  float temp_C = 0.0; //temperature in Celsius

  voltage = msg.data * (5.0/1023) - 0.5; //convert adc value to voltage 0 degrees being 0.5V
  temp_C = voltage/.01; //0.01V per degree C
  temp_F.data = temp_C * (9.0/5.0) + 32;
}
