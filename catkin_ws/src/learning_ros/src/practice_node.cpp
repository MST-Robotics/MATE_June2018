#include "practice_node.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "practice_node");//set up the node with a name
  ros::NodeHandle n;

  ros::Subscriber test_topic = n.subscribe("test_topic", 1000, test_cb);

  ros::Publisher test_pub = n.advertise<std_msgs::Bool>("test_publish_topic", 1000);


  ros::Rate loop_wait(30);


  while(ros::ok()) 
  {  
    //publish everything once per loop
    test_pub.publish(button_state); 


    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;

}


void test_cb(const std_msgs::Bool &value)
{
  button_state.data = value.data;  

}
