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
 *    cd ~/catkin_ws
 *    source devel/setup.bash
 *    roslaunch ROV_system rov_system.launch
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
#include <geometry_msgs/Vector3.h>
#include "pins.h"//this file should be in the same directory as .ino
#include <Servo.h>

ros::NodeHandle  nh;//this is necessary too

geometry_msgs::Vector3 accelerometer;
geometry_msgs::Vector3 magnetometer;

Servo servo;//declare a servo object

Servo servo_rty3;
Servo servo_JS_x;
//Used to test thumb_stick_x, joystick_rotation, axis_pov_x
Servo servo_JS_y;
//used to test thumb_stick_y, axis_pov_y

//React to trigger data
void trigger_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led1, msg.data);    
}

//React to button_a data
void button_a_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

//React to button_e data
void button_e_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);     
}

void rotary_4_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  servo.write(servo_val);
}

void rotary_3_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  servo_rty3.write(servo_val);
}


//set up subscriptions
ros::Subscriber<std_msgs::Bool> sub_trigger("trigger", trigger_cb);
ros::Subscriber<std_msgs::Bool> sub_button_a("button_a", button_a_cb);
ros::Subscriber<std_msgs::Bool> sub_button_e("button_e", button_e_cb);
ros::Subscriber<std_msgs::Float32> sub_rotary_4("rotary_4", rotary_4_cb);

//set up publishers
ros::Publisher accel_pub("accel_topic", &accelerometer);
ros::Publisher mag_pub("mag_topic", &magnetometer);

//Arduino setup
void setup()
{
  main_setup();//contains the declarations and hardware setups
  
  //set up pins and other things
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  servo.attach(servo1_pin);
  servo_rty3.attach(servo2_pin);
  servo_JS_x.attach(servo3_pin);
  servo_JS_y.attach(servo4_pin);
  
  //this is needed
  nh.initNode();

  //put all of the subscriptions here
  nh.subscribe(sub_trigger);
  nh.subscribe(sub_button_a);
  nh.subscribe(sub_button_e);
  nh.subscribe(sub_rotary_4);

  nh.advertise(accel_pub);
  nh.advertise(mag_pub);
  
}


void loop()
{ 
  nh.spinOnce();
  process_imu();
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

