#include <PixyUART.h>
#include <TPixy.h>
#include <Pixy.h>
#include <PixySPI_SS.h>
#include <PixyI2C.h>

#include <ROV_main_setup.h>

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

#include <ROV_main_setup.h>

//required to use ros with arduino
#include <ros.h>

/*ros message data types can be
 * found at: http://wiki.ros.org/std_msgs
*/
#include <std_msgs/Bool.h> //include required per type of message
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include "pins.h"//this file should be in the same directory as .ino
#include <Servo.h>

ros::NodeHandle  nh;//this is necessary too

geometry_msgs::Vector3 accelerometer;
geometry_msgs::Vector3 magnetometer;

geometry_msgs::Vector3 pixy_data;


/*************************/
Servo front_right;
Servo front_left;
Servo back_right;
Servo back_left;
Servo middle_left;
Servo middle_right;

//Servo camera_one;
//Servo camera_two;

void motor1_cb(const std_msgs::Int16 &msg)
{
 front_left.writeMicroseconds(msg.data);
}
void motor2_cb(const std_msgs::Int16 &msg)
{
  middle_left.writeMicroseconds(msg.data);
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
  middle_right.writeMicroseconds(msg.data);
}
void motor6_cb(const std_msgs::Int16 &msg)
{
  back_right.writeMicroseconds(msg.data);
}
/*
void camera_move1_cb(const std_msgs::Int16 &msg)
{
  camera_one.writeMicroseconds(msg.data);
}

void camera_move2_cb(const std_msgs::Int16 &msg)
{
  camera_two.writeMicroseconds(msg.data);
}


void pinky_trigger_cb(const std_msgs::Bool &msg)
{
  middle_left.writeMicroseconds(msg.data);
}
void trigger_cb(const std_msgs::Bool &msg)
{
  middle_right.writeMicroseconds(msg.data);
}

*/
//set up subscriptions
//ros::Subscriber<std_msgs::Bool> e_button_sub("e_button_topic", button_e_cb);
ros::Subscriber<std_msgs::Int16> motor1_sub("motor1_topic", motor1_cb);
ros::Subscriber<std_msgs::Int16> motor2_sub("motor2_topic", motor2_cb);
ros::Subscriber<std_msgs::Int16> motor3_sub("motor3_topic", motor3_cb);
ros::Subscriber<std_msgs::Int16> motor4_sub("motor4_topic", motor4_cb);
ros::Subscriber<std_msgs::Int16> motor5_sub("motor5_topic", motor5_cb);
ros::Subscriber<std_msgs::Int16> motor6_sub("motor6_topic", motor6_cb);

//ros::Subscriber<std_msgs::Int16> camera_move1_sub("button_h2_up_topic", camera_move1_cb);
//ros::Subscriber<std_msgs::Int16> camera_move2_sub("motor6_topic", camera_move2_cb);

//ros::Subscriber<std_msgs::Bool> trigger_sub("trigger_topic", trigger_cb);
//ros::Subscriber<std_msgs::Bool> pinky_trigger_sub("pinky_trigger_topic", pinky_trigger_cb);
/*
ros::Subscriber<std_msgs::Bool> a_button_sub("a_button_topic", button_a_cb);
ros::Subscriber<std_msgs::Bool> trigger_sub("trigger_topic", trigger_cb);
ros::Subscriber<std_msgs::Bool> pinky_trigger_sub("pinky_trigger_topic", pinky_trigger_cb);
ros::Subscriber<std_msgs::Float32> joystick_rotation_sub("joystick_rotation", joystick_rotation_cb);
*/

//set up publishers
ros::Publisher accel_pub("accel_topic", &accelerometer);
ros::Publisher mag_pub("mag_topic", &magnetometer);
ros::Publisher pixy_pub("pixy_data_topic", &pixy_data);//message for raw data from pixy

//Arduino setup
void setup()
{
  main_setup();//contains the declarations and hardware setup

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

  //camera_one.attach(junkservo);
  //camera_two.attach(other_junkservo);
  //this is needed
  nh.initNode();

  //put all of the subscriptions here
  //nh.subscribe(trigger_sub);
  //nh.subscribe(button_a_sub);
  //nh.subscribe(button_e_sub);
  //nh.subscribe(rotary_4_sub);
  
//  nh.subscribe(trigger_sub);
  //nh.subscribe(pinky_trigger_sub);
  
  nh.subscribe(motor1_sub);
  nh.subscribe(motor2_sub);
  nh.subscribe(motor3_sub);
  nh.subscribe(motor4_sub);
  nh.subscribe(motor5_sub);
  nh.subscribe(motor6_sub);

//  nh.subscribe(camera_move1_sub);
  //nh.subscribe(camera_move2_sub);
  //nh.subscribe(joystick_rotation_sub);

  nh.advertise(accel_pub);
  nh.advertise(mag_pub);
  nh.advertise(pixy_pub);

}


void loop()
{
  nh.spinOnce();
  //process_imu();
  //wing_detection_data();
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void process_imu(void)
{
  if(imu.gyroAvailable())
    imu.readGyro();
  if(imu.accelAvailable())
    imu.readAccel();
  if(imu.magAvailable())
    imu.readMag();

  accelerometer.x = imu.calcAccel(imu.ax);
  accelerometer.y = imu.calcAccel(imu.ay);
  accelerometer.z = imu.calcAccel(imu.az);

  magnetometer.x = imu.calcMag(imu.mx);
  magnetometer.y = imu.calcMag(imu.my);
  magnetometer.z = imu.calcMag(imu.mz);

  accel_pub.publish(&accelerometer);
  mag_pub.publish(&magnetometer);
}
/*
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
        pixy_pub.publish(&pixy_data);
      }
    }
  }
}*/
