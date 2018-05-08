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
  ros::Subscriber raw_temp_topic = n.subscribe("raw_temp_topic", 1000, raw_temp_cb);

  //Publishers
  ros::Publisher temp_result_pub = n.advertise<std_msgs::Float32>("temp_result_topic", 1000);

  ros::Rate loop_wait(30);//this is needed

  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  {
    temp_result_pub.publish(temp_F);

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}

void raw_temp_cb(const std_msgs::Float32 &msg)
{
  float voltage = 0.0;
  float temp_C = 0.0; //temperature in Celsius

  voltage = msg.data * (5.0/1023) - 0.5; //convert adc value to voltage 0 degrees being 0.5V
  temp_C = voltage/.01; //0.01V per degree C
  temp_F.data = temp_C * (9.0/5.0) + 32;
}
