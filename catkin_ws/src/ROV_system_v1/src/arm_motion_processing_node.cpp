/*
 * Author: Tamara Spivey
 * Email: tjsxz4@mst.edu
 * Date: 01-10-2018
 * 
 * 
 */
#include "arm_motion_processing_node.h"

int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"arm_motion_processing");
  ros::NodeHandle n;
  
  //manipulator controls are on throttle switches 1-6, button E for returning to home position
  ros::Subscriber manipulator_home_topic = n.subscribe("button_e_topic", 1000, manipulator_home_cb);
  ros::Subscriber sw1_topic = n.subscribe("sw1_topic", 1000, sw1_callback);
  ros::Subscriber sw2_topic = n.subscribe("sw2_topic", 1000, sw2_callback);
  ros::Subscriber sw3_topic = n.subscribe("sw3_topic", 1000, sw3_callback);
  ros::Subscriber sw4_topic = n.subscribe("sw4_topic", 1000, sw4_callback);
  ros::Subscriber sw5_topic = n.subscribe("sw5_topic", 1000, sw5_callback);
  ros::Subscriber sw6_topic = n.subscribe("sw6_topic", 1000, sw6_callback);

  //gimbal controls are on joystick h2, button a for returning to home position
  ros::Subscriber gimbal_home_topic = n.subscribe("button_a_topic", 1000, gimbal_home_cb);
  ros::Subscriber gimbal_xpos_topic = n.subscribe("gimbal_xpos_topic", 1000, gimbal_x_cb);
  ros::Subscriber gimbal_ypos_topic = n.subscribe("gimbal_ypos_topic", 1000, gimbal_y_cb);
  
  ros::Subscriber tgl1_topic = n.subscribe("tgl1_topic", 1000, tgl1_callback);
  ros::Subscriber tgl2_topic = n.subscribe("tgl2_topic", 1000, tgl2_callback);
  ros::Subscriber m1_topic = n.subscribe("m1_topic", 1000, m1_callback);

  ros::Publisher wrist_pub = n.advertise<std_msgs::UInt8>("wrist_topic", 100);
  ros::Publisher claw_pub = n.advertise<std_msgs::UInt8>("claw_topic", 100);
  ros::Publisher elbow_pub = n.advertise<std_msgs::UInt8>("elbow_topic", 100);

  ros::Publisher gimbal_x_pub = n.advertise<std_msgs::Int16>("gimbal_x_topic", 100);
  ros::Publisher gimbal_y_pub = n.advertise<std_msgs::Int16>("gimbal_y_topic", 100);
  ros::Publisher leveler_pub = n.advertise<std_msgs::UInt8>("leveler_topic", 100);
  ros::Publisher pid_pub = n.advertise<std_msgs::Bool>("pid_state_topic", 100);
 

  ros::Rate loop_wait(15);//this is needed
  
  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  { 
    wrist_pub.publish(wrist_value);
    claw_pub.publish(claw_value);
    elbow_pub.publish(elbow_value);

    gimbal_x_pub.publish(gimbal_x_value);
    gimbal_y_pub.publish(gimbal_y_value);
    leveler_pub.publish(leveler_value);
    pid_pub.publish(pid_state);
	

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}

//function that simply passes M1 state through to the main board, but 
//in this slower-running node to keep buffers from filling
void m1_callback(const std_msgs::Bool &msg)
{
  pid_state.data = msg.data;
}

void tgl1_callback(const std_msgs::Int16 &msg)
{
  // 128 = off
  // 0 = max fwd
  // 255 = max rev
   
  //this calcualtes the value for the leveler to the above control values   
  leveler_value.data = 128 + (msg.data * LEVELER_SPEED);
}

void tgl2_callback(const std_msgs::Int16 &msg)
{
  if(msg.data == 1)	
  {
    claw_value.data = pos_claw = CLAW_MAX;
  }
  else if(msg.data == -1)
  {
    claw_value.data = pos_claw = CLAW_MIN;
  }	
}

void manipulator_home_cb(const std_msgs::Bool &msg)
{
  if(msg.data == 1)
  {
    wrist_value.data = pos_wrist = WRIST_HOME;
    claw_value.data = pos_claw = CLAW_HOME;
    elbow_value.data = pos_elbow = ELBOW_HOME;
  }
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
      pos_gimbal_x += msg.data*GIMBAL_STEP_SIZE;
  }

  if(msg.data == -1)
  {
    if(pos_gimbal_x > GIMBAL_X_MIN)
      pos_gimbal_x += msg.data*GIMBAL_STEP_SIZE;
  }
 
  gimbal_x_value.data = pos_gimbal_x;
}

void gimbal_y_cb(const std_msgs::Int16 &msg)
{
  if(msg.data == 1)
  {
    if(pos_gimbal_y < GIMBAL_Y_MAX)
      pos_gimbal_y += msg.data*GIMBAL_STEP_SIZE;
  }

  if(msg.data == -1)
  {
    if(pos_gimbal_y > GIMBAL_Y_MIN)
      pos_gimbal_y += msg.data*GIMBAL_STEP_SIZE;
  }
 
  gimbal_y_value.data = pos_gimbal_y;
}

void sw1_callback(const std_msgs::Bool &msg)
{
  // increment wrist
  if(msg.data == 1)
  {
	if(pos_elbow + MANIPULATOR_STEP_SIZE <= ELBOW_MAX) {
    	pos_elbow += MANIPULATOR_STEP_SIZE;
	}
	elbow_value.data = pos_elbow;
  }
}
void sw2_callback(const std_msgs::Bool &msg)
{
  // decrement wrist
  if(msg.data == 1)
  {
	if(pos_elbow - MANIPULATOR_STEP_SIZE >= ELBOW_MIN) {
    	pos_elbow -= MANIPULATOR_STEP_SIZE;
	}
	elbow_value.data = pos_elbow;
  }
}
void sw3_callback(const std_msgs::Bool &msg)
{
  // increment elbow
  if(msg.data == 1)
  {
	if(pos_wrist + MANIPULATOR_STEP_SIZE <= WRIST_MAX) {
    	pos_wrist += MANIPULATOR_STEP_SIZE;
	}
	wrist_value.data = pos_wrist;
  }
}
void sw4_callback(const std_msgs::Bool &msg)
{
  // decrement elbow
  if(msg.data == 1)
  {
	if(pos_wrist - MANIPULATOR_STEP_SIZE >= WRIST_MIN) {
    	pos_wrist -= MANIPULATOR_STEP_SIZE;
	}
	wrist_value.data = pos_wrist;
  }
}
void sw5_callback(const std_msgs::Bool &msg)
{
  // increment claw
  if(msg.data == 1)
  {
	if(pos_claw + MANIPULATOR_STEP_SIZE <= CLAW_MAX) {
	  pos_claw += MANIPULATOR_STEP_SIZE;
	}
	claw_value.data = pos_claw;
  }
}
void sw6_callback(const std_msgs::Bool &msg)
{
  //decrement claw
  if(msg.data == 1)
  {
	if(pos_claw - MANIPULATOR_STEP_SIZE >= CLAW_MIN) {
	  pos_claw -= MANIPULATOR_STEP_SIZE;
	}
	claw_value.data = pos_claw;
  }
}
