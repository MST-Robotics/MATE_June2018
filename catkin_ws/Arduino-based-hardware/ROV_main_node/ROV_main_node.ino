/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst,edu
 * Date: 6-19-2017
 *
 * Code for basic implementation of rosserial
 *
 * To start the system, run from terminal:
 * Terminal 1:
 *    roscore
 *
 * Terminal 2:
 *    cd ~/MATE_June2018/catkin_ws
 *    source devel/setup.bash
 *    roslaunch ROV_system_v1 rov_system.launch
 *
 * This will start all necessary nodes to read both controllers
 * and stream data to the arduino
 */

#include "ROV_main_node.h"

//Arduino setup
void setup()
{
  main_setup();//contains the declarations and hardware setup
  motor_setup();
  roll_setpoint = 180;//when leveled, the value for roll is 180 degrees
  roll_PID.SetOutputLimits(-200, 200);//this range is an offset for motor7's speed
  roll_PID.SetMode(AUTOMATIC);
  
  nh.initNode();//initialize the node

  //set up topic subscriptions
  nh.subscribe(motor1_sub);
  nh.subscribe(motor2_sub);
  nh.subscribe(motor3_sub);
  nh.subscribe(motor4_sub);
  nh.subscribe(motor5_sub);
  nh.subscribe(motor6_sub);
  nh.subscribe(motor7_sub);

  nh.subscribe(wrist_sub);
  nh.subscribe(claw_sub);
  nh.subscribe(elbow_sub);
  nh.subscribe(gimbal_x_sub);
  nh.subscribe(gimbal_y_sub);

  //set up topic publishers
  nh.advertise(pixy_pub);
  nh.advertise(raw_temp_pub);
}

void loop()
{
  nh.spinOnce();//run ros once
  process_temperature();//update the temperature data
  process_imu();//update imu data
  //wing_detection_data();//update image recognition data
}
