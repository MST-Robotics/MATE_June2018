
#ifndef ROV_main_node_H
#define ROV_main_node_H

#include <ROV_main_setup.h>
#include <PID_v1.h>
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
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3.h>

/*
 * These constants are the thresholds for switching tuning values for the PID
 * If the robot is above this angle from setpoint, use more aggressive tuning values
 */
#define PITCH_THRESHOLD 1
#define ROLL_THRESHOLD 1

/*
 * These constants are the minimum correction value that will be added to the motors
 * This prevents the motors from twitching or barely moving when very small offsets are generated
 */
#define PITCH_OFFSET_THRESHOLD 1
#define ROLL_OFFSET_THRESHOLD 1

//use this version of to increase the buffer size 
//12 subscribers, 5 publishers 1024 bytes per buffer
ros::NodeHandle_<ArduinoHardware, 17, 3, 1024, 1024> nh;

/*
 * Stores orientation data
 * x=pitch y=roll z=pitch_offset
 */
//geometry_msgs::Vector3 orientation;

//Stores raw adc value for temp sensor
std_msgs::Float32 raw_temp;
 
Servo front_right;
Servo front_left;
Servo back_right;
Servo back_left;
Servo middle_left;
Servo middle_right;
Servo back_middle;

bool pid_enable = 1;

//variables for holding the pitch and roll for the PID
double roll, roll_setpoint, roll_offset;
double pitch, pitch_setpoint, pitch_offset;

//tuning variables for the PID control
double consKp=1, consKi=0.1, consKd=.25;
double aggKp=4, aggKi=0.2, aggKd=1;

//this array stores the manipulator motor values
//elbow, wirst, claw
uint8_t manipulator_data[4] = {0, 0, 0, 128};


//initialize this value to off
uint8_t leveler_data = 127;

//create PID object
PID roll_PID(&roll, &roll_offset, &roll_setpoint, consKp, consKi, consKd, DIRECT);
PID pitch_PID(&pitch, &pitch_offset, &pitch_setpoint, consKp, consKi, consKd, DIRECT);

//These are the callback functions that control the motors
void motor1_cb(const std_msgs::Int16 &msg)
{
  front_left.writeMicroseconds(msg.data);
}
void motor3_cb(const std_msgs::Int16 &msg)
{
  back_left.writeMicroseconds(msg.data);
}
void motor4_cb(const std_msgs::Int16 &msg)
{
  front_right.writeMicroseconds(msg.data);
}
void motor6_cb(const std_msgs::Int16 &msg)
{
  back_right.writeMicroseconds(msg.data);
}
/*
 * This motor is the left middle motor for controlling roll
 * It's PID is paired with the middle right motor.
 * The middle right motor adjusts in the opposite direction of this motor
 */
void motor2_cb(const std_msgs::Int16 &msg)
{
  //update the motor's speed from contpitcher input, and also correction offset from IMU
  int motor_speed = msg.data - (int)roll_offset; 
  if(motor_speed > 1900)
    motor_speed = 1900;//motors are at max, cannot correct
  else if(motor_speed < 1100)
    motor_speed = 1100;//motors are at min, cannot correct
  middle_left.writeMicroseconds(motor_speed);
}

/*
 * This motor is the middle right motor for controlling roll
 * It's PID is paired with the middle left motor.
 * The middle right left adjusts in the opposite direction of this motor
 */
void motor5_cb(const std_msgs::Int16 &msg)
{
  //update the motor's speed from contpitcher input, and also correction offset from IMU
  int motor_speed = msg.data + (int)roll_offset; 
  if(motor_speed > 1900)
    motor_speed = 1900;//motors are at max, cannot correct
  else if(motor_speed < 1100)
    motor_speed = 1100;//motors are at min, cannot correct

  middle_right.writeMicroseconds(motor_speed);
}

/*
 * This motor is the middle back motor for controlling pitch
 * It's PID runs independly of the other middle two motors 
 */
void motor7_cb(const std_msgs::Int16 &msg)
{
  //update the motor's speed from contpitcher input, and also correction offset from IMU
  int motor_speed = msg.data - (int)pitch_offset; 
  if(motor_speed > 1900)
    motor_speed = 1900;//motors are at max, cannot correct
  else if(motor_speed < 1100)
    motor_speed = 1100;//motors are at min, cannot correct
  
  back_middle.writeMicroseconds(motor_speed);
}

//Callback function for the manipulator joints
//These functions poopulate an array of data that is sent via Serial2 to another arduino
void elbow_cb(const std_msgs::UInt8 &msg)
{
  manipulator_data[1] = msg.data;//second index gets elbow value
}
void wrist_cb(const std_msgs::UInt8 &msg)
{
  manipulator_data[2] = msg.data;//thrid index gets wirst value
}
void claw_cb(const std_msgs::UInt8 &msg)
{
  manipulator_data[0] = msg.data;//fourth index gets claw value
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

//This function uses the state of M1 on the throttle to enable/disable the PID adjustments
void pid_enable_cb(const std_msgs::Bool &msg)
{
  pid_enable = msg.data;
}

//This function uses the state of M1 on the throttle to enable/disable the PID adjustments
void setpoint_cb(const std_msgs::Int16 &msg)
{
  pitch_setpoint = (double)msg.data;//when leveled, the value for roll is 180 degrees
}

void leveler_cb(const std_msgs::UInt8 &msg)
{
  manipulator_data[3] = msg.data;
}

//set up subscriptions
//ros::Subscriber<std_msgs::Bool> e_button_sub("e_button_topic", button_e_cb);
ros::Subscriber<std_msgs::Int16> motor1_sub("motor1_topic", motor1_cb);
ros::Subscriber<std_msgs::Int16> motor2_sub("motor2_topic", motor2_cb);
ros::Subscriber<std_msgs::Int16> motor3_sub("motor3_topic", motor3_cb);
ros::Subscriber<std_msgs::Int16> motor4_sub("motor4_topic", motor4_cb);
ros::Subscriber<std_msgs::Int16> motor5_sub("motor5_topic", motor5_cb);
ros::Subscriber<std_msgs::Int16> motor6_sub("motor6_topic", motor6_cb);
ros::Subscriber<std_msgs::Int16> motor7_sub("motor7_topic", motor7_cb);

ros::Subscriber<std_msgs::UInt8> wrist_sub("wrist_topic", wrist_cb);
ros::Subscriber<std_msgs::UInt8> claw_sub("claw_topic", claw_cb);
ros::Subscriber<std_msgs::UInt8> elbow_sub("elbow_topic", elbow_cb);

ros::Subscriber<std_msgs::Int16> gimbal_x_sub("gimbal_x_topic", gimbal_x_cb);
ros::Subscriber<std_msgs::Int16> gimbal_y_sub("gimbal_y_topic", gimbal_y_cb);

ros::Subscriber<std_msgs::Bool> pid_enable_sub("pid_state_topic", pid_enable_cb);
ros::Subscriber<std_msgs::Int16> setpoint_sub("setpoint_topic", setpoint_cb);
ros::Subscriber<std_msgs::UInt8> leveler_sub("leveler_topic", leveler_cb);

//set up publishers
ros::Publisher raw_temp_pub("raw_temp_topic", &raw_temp);
//ros::Publisher orientation_pub("orientation_topic", &orientation);

//function for setting up the brushless motors
void motor_setup(void)
{
  front_right.attach(frt_pin);
  front_left.attach(flt_pin);
  back_right.attach(brt_pin);
  back_left.attach(blt_pin);
  middle_right.attach(middle_right_pin);
  middle_left.attach(middle_left_pin);
  back_middle.attach(back_pin);

  //start with the motors off
  front_right.writeMicroseconds(1500);
  front_left.writeMicroseconds(1500);
  middle_right.writeMicroseconds(1500);
  middle_left.writeMicroseconds(1500);
  back_right.writeMicroseconds(1500);
  back_left.writeMicroseconds(1500);   
  back_middle.writeMicroseconds(1500);

  return;
}

/*
 * This function sends the packaged manipulator data ocne per loop
 * In this order: 'L' Data
 */
void send_leveler_data(void)
{
  Serial2.print('L'+String(leveler_data)+'\n');
 return;  
}

/*
 * This function sends the packaged manipulator data ocne per loop
 * In this order: 'M' elbow_value wrist_value claw_value '\n'
 */
void send_manipulator_data(void)
{
  Serial2.print('M');
  for(int i = 0; i < 4; i++)
  {
    Serial2.print(manipulator_data[i]);
    Serial2.print(',');
  }
  Serial2.println();   
  
  return;  
}

//function for reading and packaging and publishing the IMU data
void process_imu(void)
{
  imu.readAccel();

  //calcualte the accelerometer data
  float accelerometer_x = imu.calcAccel(imu.ax);
  float accelerometer_y = imu.calcAccel(imu.ay);
  float accelerometer_z = imu.calcAccel(imu.az);

  //calculate pitch
  pitch = (atan2(accelerometer_y, accelerometer_z));
  pitch *= 180.0/PI;//convert to degrees
  
  //convert to positive angles only 0-360. Pitch is ROV lengthwise rotation
  if(pitch < 0)
    pitch+=360;

  //calculate roll
  roll = atan2(-1*accelerometer_x, sqrt(accelerometer_y * accelerometer_y + accelerometer_z * accelerometer_z));
  roll *= 180.0/PI;//convert to degrees


  //orientation.x = pitch;  
  //orientation.y = roll;  

  if(pid_enable)
  {
    //Calculate PID for pitch      
    pitch_PID.Compute();//calcualte the pitch_output
   
    //check for minimum amount of motor adjustment
    if(abs(pitch_offset) < PITCH_OFFSET_THRESHOLD)//this number may be adjusted as well
      pitch_offset = 0;//set the speed offset to zero, meaning no correction will be added to current speed

      
    roll_PID.Compute();//calcualte the roll_output
    
    //check for minimum amount of motor adjustment
    if(abs(roll_offset) < ROLL_OFFSET_THRESHOLD)//this number may be adjusted as well
      roll_offset = 0;//set the speed offset to zero, meaning no correction will be added to current speed
    
   // orientation.z = pitch_offset;//have this so it can be viewed with the orientation
  }

  else
  {
    roll_offset = 0;
    pitch_offset = 0;
  }

  //orientation_pub.publish(&orientation);
  return;
}

//function for reading and calculating the temperature
void process_temperature(void)
{
  raw_temp.data = analogRead(temp_pin);//read the temperature sensor
 
  raw_temp_pub.publish(&raw_temp);//publish the temperature data
  return;
}



#endif
