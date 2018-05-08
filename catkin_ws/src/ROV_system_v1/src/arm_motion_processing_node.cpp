/*
 * Author: Tamara Spivey
 * Email: tjsxz4@mst.edu
 * Date: 01-10-2018
 * 
 * 
 */
#include "arm_motion_processing_node.h"

int pos_wrist = WRIST_HOME;
int pos_elbow = ELBOW_HOME;
int pos_claw = CLAW_HOME;

int pos_gimbal_x = GIMBAL_X_HOME;
int pos_gimbal_y = GIMBAL_Y_HOME;


int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"arm_motion_processing");
  ros::NodeHandle n;

  ros::Subscriber sw1_topic = n.subscribe("sw1_topic", 1000, sw1_callback);
  ros::Subscriber sw2_topic = n.subscribe("sw2_topic", 1000, sw2_callback);
  ros::Subscriber sw3_topic = n.subscribe("sw3_topic", 1000, sw3_callback);
  ros::Subscriber sw4_topic = n.subscribe("sw4_topic", 1000, sw4_callback);
  ros::Subscriber sw5_topic = n.subscribe("sw5_topic", 1000, sw5_callback);
  ros::Subscriber sw6_topic = n.subscribe("sw6_topic", 1000, sw6_callback);

  ros::Subscriber gimbal_home_topic = n.subscribe("button_a_topic", 1000, gimbal_home_cb);
  ros::Subscriber gimbal_xpos_topic = n.subscribe("gimbal_xpos_topic", 1000, gimbal_x_cb);
  ros::Subscriber gimbal_ypos_topic = n.subscribe("gimbal_ypos_topic", 1000, gimbal_y_cb);
  
  ros::Subscriber tgl1_topic = n.subscribe("tgl1_topic", 1000, tg1l_callback);

  ros::Publisher wrist_pub = n.advertise<std_msgs::UInt8>("wrist_topic", 1000);
  ros::Publisher claw_pub = n.advertise<std_msgs::UInt8>("claw_topic", 1000);
  ros::Publisher elbow_pub = n.advertise<std_msgs::UInt8>("elbow_topic", 1000);

  ros::Publisher gimbal_x_pub = n.advertise<std_msgs::Int16>("gimbal_x_topic", 1000);
  ros::Publisher gimbal_y_pub = n.advertise<std_msgs::Int16>("gimbal_y_topic", 1000);
  ros::Publisher leveler_pub = n.advertise<std_msgs::Int16>("levler_topic", 1000);
 

  ros::Rate loop_wait(20);//this is needed
  
  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  { 
    wrist_pub.publish(wrist_value);
    claw_pub.publish(claw_value);
    elbow_pub.publish(elbow_value);

    gimbal_x_pub.publish(gimbal_x_value);
    gimbal_y_pub.publish(gimbal_y_value);
	leveler_pub.publish(leveler_value);
	

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}

void tgl1_callback(const std_msgs::Int16 &msg)
{
  // 128 = off
  // 0 = max fwd
  // 255 = max rev
   
  //this calcualtes the value for the leveler to the above control values   
  leveler_value.data = 128 + (msg.data * 127);
}


void gimbal_home_cb(const std_msgs::Bool &msg)
{
  if(msg.data == 1)
  {
    pos_gimbal_x = GIMBAL_X_HOME;
    pos_gimbal_y = GIMBAL_Y_HOME;
  }

}

//please optimize these functions to include movement speeds >1 step
void gimbal_x_cb(const std_msgs::Int16 &msg)
{
  if(msg.data == 1)
  {
    if(pos_gimbal_x < GIMBAL_X_MAX)
      pos_gimbal_x += msg.data*GIMBAL_MOVEMENT_SPEED;
  }

  if(msg.data == -1)
  {
    if(pos_gimbal_x > GIMBAL_X_MIN)
      pos_gimbal_x += msg.data*GIMBAL_MOVEMENT_SPEED;
  }
 
  gimbal_x_value.data = pos_gimbal_x;
}

void gimbal_y_cb(const std_msgs::Int16 &msg)
{
  if(msg.data == 1)
  {
    if(pos_gimbal_y < GIMBAL_Y_MAX)
      pos_gimbal_y += msg.data*GIMBAL_MOVEMENT_SPEED;
  }

  if(msg.data == -1)
  {
    if(pos_gimbal_y > GIMBAL_Y_MIN)
      pos_gimbal_y += msg.data*GIMBAL_MOVEMENT_SPEED;
  }
 
  gimbal_y_value.data = pos_gimbal_y;
}

void sw1_callback(const std_msgs::Bool &msg)
{
  // increment wrist
  if(msg.data == 1)
  {
	if(pos_elbow < ELBOW_MAX) {
    	pos_elbow += movement_speed;
	}
	elbow_value.data = pos_elbow;
  }
}
void sw2_callback(const std_msgs::Bool &msg)
{
  // decrement wrist
  if(msg.data == 1)
  {
	if(pos_elbow > ELBOW_MIN) {
    	pos_elbow -= movement_speed;
	}
	elbow_value.data = pos_elbow;
  }
}
void sw3_callback(const std_msgs::Bool &msg)
{
  // increment elbow
  if(msg.data == 1)
  {
	if(pos_wrist < WRIST_MAX) {
    	pos_wrist += movement_speed;
	}
	wrist_value.data = pos_wrist;
  }
}
void sw4_callback(const std_msgs::Bool &msg)
{
  // decrement elbow
  if(msg.data == 1)
  {
	if(pos_wrist > WRIST_MIN) {
    	pos_wrist -= movement_speed;
	}
	wrist_value.data = pos_wrist;
  }
}
void sw5_callback(const std_msgs::Bool &msg)
{
  // increment claw
  if(msg.data == 1)
  {
	if(pos_claw < CLAW_MAX) {
	  pos_claw += movement_speed;
	}
	claw_value.data = pos_claw;
  }
}
void sw6_callback(const std_msgs::Bool &msg)
{
  //decrement claw
  if(msg.data == 1)
  {
	if(pos_claw > CLAW_MIN) {
	  pos_claw -= movement_speed;
	}
	claw_value.data = pos_claw;
  }
}
