/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst,edu
 * Date: 4-14-2017
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
 */

#ifndef ROV_main_node_H
#define ROV_main_node_H

#include <ROV_main_setup.h>
#include <ros.h>
#include <Servo.h>

#include "pins.h"//this file should be in the same directory as .ino
#include "pwm_channels.h"

/*ros message data types can be
 * found at: http://wiki.ros.org/std_msgs
*/
#include <std_msgs/Bool.h> //include required per type of message
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h> 
#include <geometry_msgs/Vector3.h>


//use this version of to increase the buffer size 
//12 subscribers, 5 publishers 1024 bytes per buffer
ros::NodeHandle_<ArduinoHardware, 12, 5, 1024, 1024> nh;

/*
 * Stores accelerometer data
 * x=x-axis y=y-axis z=z-axis
 */
geometry_msgs::Vector3 accelerometer;

/*
 * Stores magnetometer data
 * x=x-axis y=y-axis z=z-axis
 */
geometry_msgs::Vector3 magnetometer;

/*
 * Stores pixy data
 * x=signature y=width z=height
 */
geometry_msgs::Vector3 pixy_data;

//Stores raw adc value for temp sensor
std_msgs::Float32 raw_temp;

Servo front_right;
Servo front_left;
Servo back_right;
Servo back_left;
Servo middle_left;
Servo middle_right;


//These are the callback functions that control the motors
void motor1_cb(const std_msgs::Int16 &msg)
{
  front_left.writeMicroseconds(msg.data);
}
void motor2_cb(const std_msgs::Int16 &msg)
{
  middle_left.writeMicroseconds(msg.data);
  middle_right.writeMicroseconds(msg.data);
}
void motor3_cb(const std_msgs::Int16 &msg)
{
  back_left.writeMicroseconds(msg.data);
}
void motor4_cb(const std_msgs::Int16 &msg)
{
  front_right.writeMicroseconds(msg.data);
}
void motor5_cb(const std_msgs::Int16 &msg)
{
  //middle_right.writeMicroseconds(msg.data);
}
void motor6_cb(const std_msgs::Int16 &msg)
{
  back_right.writeMicroseconds(msg.data);
}

//Callback functions for the arm servos
void wrist_cb(const std_msgs::Int16 &msg)
{
  pwm_secondary.writeMicroseconds(elbow_servo, msg.data);
}
void claw_cb(const std_msgs::Int16 &msg)
{
  pwm_secondary.writeMicroseconds(claw_servo, msg.data);
}
void elbow_cb(const std_msgs::Int16 &msg)
{
  pwm_secondary.writeMicroseconds(elbow_servo, msg.data);
}

void gimbal_x_cb(const std_msgs::Int16 &msg)
{
  int microseconds = map(msg.data, 0, 180, 2000, 1000);
  pwm_primary.writeMicroseconds(gimbal_x, microseconds);
}

void gimbal_y_cb(const std_msgs::Int16 &msg)
{
  int microseconds = map(msg.data, 0, 180, 2000, 1000);
  pwm_primary.writeMicroseconds(gimbal_y, microseconds);
}

//set up subscriptions
//ros::Subscriber<std_msgs::Bool> e_button_sub("e_button_topic", button_e_cb);
ros::Subscriber<std_msgs::Int16> motor1_sub("motor1_topic", motor1_cb);
ros::Subscriber<std_msgs::Int16> motor2_sub("motor2_topic", motor2_cb);
ros::Subscriber<std_msgs::Int16> motor3_sub("motor3_topic", motor3_cb);
ros::Subscriber<std_msgs::Int16> motor4_sub("motor4_topic", motor4_cb);
ros::Subscriber<std_msgs::Int16> motor5_sub("motor5_topic", motor5_cb);
ros::Subscriber<std_msgs::Int16> motor6_sub("motor6_topic", motor6_cb);
ros::Subscriber<std_msgs::Int16> wrist_sub("arm_motor1_topic", wrist_cb);
ros::Subscriber<std_msgs::Int16> claw_sub("arm_motor2_topic", claw_cb);
ros::Subscriber<std_msgs::Int16> elbow_sub("arm_motor3_topic", elbow_cb);

ros::Subscriber<std_msgs::Int16> gimbal_x_sub("gimbal_x_topic", gimbal_x_cb);
ros::Subscriber<std_msgs::Int16> gimbal_y_sub("gimbal_y_topic", gimbal_y_cb);

//set up publishers
ros::Publisher accel_pub("accel_topic", &accelerometer);
ros::Publisher mag_pub("mag_topic", &magnetometer);
ros::Publisher pixy_pub("pixy_data_topic", &pixy_data);//message for raw data from pixy
ros::Publisher raw_temp_pub("raw_temp_topic", &raw_temp);

//function for setting up the brushless motors
void motor_setup(void)
{
  front_right.attach(frt_pin);
  front_left.attach(flt_pin);
  back_right.attach(brt_pin);
  back_left.attach(blt_pin);
  middle_right.attach(middle_right_pin);
  middle_left.attach(middle_left_pin);

  //start with the motors off
  front_right.writeMicroseconds(1500);
  front_left.writeMicroseconds(1500);
  middle_right.writeMicroseconds(1500);
  middle_left.writeMicroseconds(1500);
  back_right.writeMicroseconds(1500);
  back_left.writeMicroseconds(1500);   

  return;
}

//function for reading and calculating the temperature
void process_temperature(void)
{
  raw_temp.data = analogRead(temp_pin);//read the temperature sensor
 
  raw_temp_pub.publish(&raw_temp);//publish the temperature data
  return;
}

//function for reading and packaging and publishing the IMU data
void process_imu(void)
{
  //update the data if it has changed
  if(imu.gyroAvailable())
    imu.readGyro();
  if(imu.accelAvailable())
    imu.readAccel();
  if(imu.magAvailable())
    imu.readMag();

  //calcualte the accelerometer data
  accelerometer.x = imu.calcAccel(imu.ax);
  accelerometer.y = imu.calcAccel(imu.ay);
  accelerometer.z = imu.calcAccel(imu.az);

  //calcualte the magnetometer data
  magnetometer.x = imu.calcMag(imu.mx);
  magnetometer.y = imu.calcMag(imu.my);
  magnetometer.z = imu.calcMag(imu.mz);
  
  //publish the data
  accel_pub.publish(&accelerometer);
  mag_pub.publish(&magnetometer);

  return;
}

//function for reading the pixy camera and publishing its data 
void wing_detection_data()
{
  static int frame = 0;
  int j;
  uint16_t blocks;
  char buf[32];

  // grab blocks
  blocks = pixy.getBlocks();

  if (blocks)
  {
    frame++;

    // run every 50 frames
    if (frame%50==0)
    {
      for (j=0; j<blocks; j++)
      {
        pixy_data.x = pixy.blocks[j].signature;
        pixy_data.y = pixy.blocks[j].width;
        pixy_data.z = pixy.blocks[j].height;
        pixy_pub.publish(&pixy_data);//publish each frame
      }
    }
  }

  return;
}

#endif
