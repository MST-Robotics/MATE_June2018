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
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include "pins.h"//this file should be in the same directory as .ino
#include <Servo.h>

ros::NodeHandle  nh;//this is necessary too

geometry_msgs::Vector3 accelerometer;
geometry_msgs::Vector3 magnetometer;

Servo servo;//declare a servo object

Servo servo_rty3;
Servo servo_JS_x;
Servo servo_JS_y;

/*************************/
Servo front_right;
Servo front_left;
Servo back_right;
Servo back_left;
Servo middle_left;
Servo middle_right;

float desired_speed = 0;

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
//  digitalWrite(led3, msg.data);     
}

//void rotary_4_cb(const std_msgs::Float32 &msg)
//{
//  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
//  servo.write(servo_val);
//}

//void rotary_3_cb(const std_msgs::Float32 &msg)
//{
//  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
//  servo_rty3.write(servo_val);
//}

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
void button_pinky_trigger_cb(const std_msgs::Bool &msg)
{
  middle_left.writeMicroseconds(msg.data);      
}
void button_trigger_cb(const std_msgs::Bool &msg)
{
  middle_right.writeMicroseconds(msg.data);    
}
*/


//left and right
void joystick_x_cb(const std_msgs::Float32 &msg)
{
 //uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
 //servo_JS_x.write(servo_val);
}
//foward and backward
void joystick_y_cb(const std_msgs::Float32 &msg)
{
}

//void joystick_rotation_cb(const std_msgs::Float32 &msg)
//{
//  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
//  servo_JS_x.write(servo_val);
//} 


//set up subscriptions


ros::Subscriber<std_msgs::Bool> sub_trigger("trigger", trigger_cb);
ros::Subscriber<std_msgs::Bool> sub_button_a("button_a", button_a_cb);
ros::Subscriber<std_msgs::Bool> sub_button_e("button_e", button_e_cb);

ros::Subscriber<std_msgs::Int16> sub_motor1("motor1_topic", motor1_cb);
ros::Subscriber<std_msgs::Int16> sub_motor2("motor2_topic", motor2_cb);
ros::Subscriber<std_msgs::Int16> sub_motor3("motor3_topic", motor3_cb);
ros::Subscriber<std_msgs::Int16> sub_motor4("motor4_topic", motor4_cb);
ros::Subscriber<std_msgs::Int16> sub_motor5("motor5_topic", motor5_cb);
ros::Subscriber<std_msgs::Int16> sub_motor6("motor6_topic", motor6_cb);

/*
ros::Subscriber<std_msgs::Bool> sub_button_trigger("motor2_topic", button_trigger_cb);
ros::Subscriber<std_msgs::Bool> sub_button_pinky_trigger("motor5S_topic", button_pinky_trigger_cb);
*/
//ros::Subscriber<std_msgs::Float32> sub_rotary_4("rotary_4", rotary_4_cb);

ros::Subscriber<std_msgs::Float32> sub_joystick_x("joystick_x", joystick_x_cb);
ros::Subscriber<std_msgs::Float32> sub_joystick_y("joystick_y", joystick_y_cb);
//ros::Subscriber<std_msgs::Float32> sub_joystick_rotation("joystick_rotation", joystick_rotation_cb);

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
 
//  servo.attach(servo1_pin);
//  servo_rty3.attach(servo2_pin);
 // servo_JS_x.attach(servo3_pin);
 // servo_JS_y.attach(servo4_pin);

  front_right.attach(frt_pin);
  front_left.attach(flt_pin);
  back_right.attach(brt_pin);
  back_left.attach(blt_pin);
  
  middle_right.attach(middle_right_pin);
  middle_left.attach(middle_left_pin);
 
  //this is needed
  nh.initNode();

  //put all of the subscriptions here

  
  nh.subscribe(sub_trigger);
  nh.subscribe(sub_button_a);
  nh.subscribe(sub_button_e);
//  nh.subscribe(sub_rotary_4);

  nh.subscribe(sub_joystick_x);
  
  nh.subscribe(sub_motor1);
  nh.subscribe(sub_motor2);
  nh.subscribe(sub_motor3);
  nh.subscribe(sub_motor4);
  nh.subscribe(sub_motor5);
  nh.subscribe(sub_motor6);

  nh.subscribe(sub_joystick_y);
  //nh.subscribe(sub_joystick_rotation);
  
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
