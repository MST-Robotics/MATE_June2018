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
 *    roslaunch start_rov.launch
 * 
 * This will start all necessary nodes to read both controllers
 * and stream data to the arduino
 */


//required to use ros with arduino
#include <ros.h>

/*ros message data types can be 
 * found at: http://wiki.ros.org/std_msgs
*/
#include <std_msgs/Bool.h> //include required per type of message
#include <std_msgs/Float32.h>
#include "pins.h"//this file should be in the same directory as .ino
#include <Servo.h>

ros::NodeHandle  nh;//this is necessary too

Servo servo;//declare a servo object

Servo servo_rty3;

Servo servo_JS_x;
//Used to test thumb_stick_x, joystick_rotation, axis_pov_x
Servo servo_JS_y;
//used to test thumb_stick_y, axis_pov_y

//React to trigger data
void trigger_cb(const std_msgs::Bool &msg)
{
  /*
  if(msg.data == 1)
    digitalWrite(led1, 1);
  else
    digitalWrite(led1, 0);
  */
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
/*
 * Before you uncomment and test, make sure that the servo 
 * they use is not being used(same with leds)
 * NEED TO ADD MORE PIN OPTIONS/HAVE MORE SERVOS AND LEDS
 */
/*
void joystick_x_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  servo_JS_x.write(servo_val);
}

void joystick_y_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
 // servo_JS_y.write(servo_val);
}

void thumb_stick_x_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  //servo_JS_x.write(servo_val);
}

void thumb_stick_y_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
 // servo_JS_y.write(servo_val);
}

void joystick_rotation_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  //servo_JS_x.write(servo_val);
}

void axis_pov_x_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  //servo_JS_x.write(servo_val);
}

void axis_pov_y_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  //servo_JS_y.write(servo_val);
}

void button_b_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_c_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led2, msg.data);       
}

void button_d_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led2, msg.data);       
}

void button_pinky_trigger_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led2, msg.data);       
}

void button_h1_up_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led2, msg.data);       
}

void button_h1_right_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led2, msg.data);       
}

void button_h1_down_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led2, msg.data);       
}

void button_h1_left_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led2, msg.data);       
}


void button_h2_up_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led3, msg.data);       
}

void button_h2_right_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led3, msg.data);       
}

void button_h2_down_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led3, msg.data);       
}

void button_h2_left_cb(const std_msgs::Bool &msg)
{
  //digitalWrite(led3, msg.data);       
}

void axis_left_thruster_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  servo_JS_x.write(servo_val);
}
void axis_right_thruster_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  servo_JS_y.write(servo_val);
}

void axis_rotary_f_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  servo_JS_y.write(servo_val);
}
void axis_rotary_g_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  servo_JS_x.write(servo_val);
}

void axis_thruster_thumb_stick_x_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  servo_JS_y.write(servo_val);
}
void axis_thruster_thumb_stick_y_cb(const std_msgs::Float32 &msg)
{
  uint8_t servo_val = mapf(msg.data, -1.0, 1.0, 0.0, 180.0);
  servo_JS_x.write(servo_val);
}

void button_rotary_f_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_rotary_g_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_i_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_h_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_sw1_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_sw2_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_sw3_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_sw4_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_sw5_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_sw6_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_tgl1_up_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_tgl1_down_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_tgl2_up_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_tgl2_down_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_tgl3_up_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_tgl3_down_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_tgl4_up_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_tgl4_down_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_h3_up_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}
void button_h3_right_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}
void button_h3_down_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}
void button_h3_left_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_h4_up_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}
void button_h4_right_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}
void button_h4_down_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}
void button_h4_left_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_ki_up_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}
void button_ki_down_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_thumb_stick_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}

void button_thumb_slider_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_scroll_forward_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}
void button_scroll_reverse_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_m1_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}
void button_m2_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led3, msg.data);       
}

void button_s1_cb(const std_msgs::Bool &msg)
{
  digitalWrite(led2, msg.data);       
}
*/


/* 
 * Rest of the buttons
 * Need to test
*/


//set up subscriptions
ros::Subscriber<std_msgs::Bool> sub_trigger("trigger", trigger_cb);
ros::Subscriber<std_msgs::Bool> sub_button_a("button_a", button_a_cb);
ros::Subscriber<std_msgs::Bool> sub_button_e("button_e", button_e_cb);
ros::Subscriber<std_msgs::Float32> sub_rotary_4("rotary_4", rotary_4_cb);
/*
 * Tested
ros::Subscriber<std_msgs::Float32> sub_rotary_3("rotary_3", rotary_3_cb);
ros::Subscriber<std_msgs::Float32> sub_joystick_x("joystick_x", joystick_x_cb);
ros::Subscriber<std_msgs::Float32> sub_joystick_y("joystick_y", joystick_y_cb);
ros::Subscriber<std_msgs::Float32> sub_thumb_stick_x("thumb_stick_x", thumb_stick_x_cb);
ros::Subscriber<std_msgs::Float32> sub_thumb_stick_y("thumb_stick_y", thumb_stick_y_cb);
ros::Subscriber<std_msgs::Float32> sub_joystick_rotation("joystick_rotation", joystick_rotation_cb);
ros::Subscriber<std_msgs::Float32> sub_axis_pov_x("axis_pov_x", axis_pov_x_cb);
ros::Subscriber<std_msgs::Float32> sub_axis_pov_y("axis_pov_y", axis_pov_y_cb);

ros::Subscriber<std_msgs::Bool> sub_button_b("button_b", button_b_cb);
ros::Subscriber<std_msgs::Bool> sub_button_c("button_c", button_c_cb);
ros::Subscriber<std_msgs::Bool> sub_button_d("button_d", button_d_cb);
ros::Subscriber<std_msgs::Bool> sub_button_pinky_trigger("button_pinky_trigger", button_pinky_trigger_cb);

ros::Subscriber<std_msgs::Bool> sub_button_h1_up("button_h1_up", button_h1_up_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h1_right("button_h1_right", button_h1_right_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h1_down("button_h1_down", button_h1_down_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h1_left("button_h1_left", button_h1_left_cb);

ros::Subscriber<std_msgs::Bool> sub_button_h2_up("button_h2_up", button_h2_up_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h2_right("button_h2_right", button_h2_right_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h2_down("button_h2_down", button_h2_down_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h2_left("button_h2_left", button_h2_left_cb);

ros::Subscriber<std_msgs::Float32> sub_axis_left_thruster("axis_left_thruster", axis_left_thruster_cb);
ros::Subscriber<std_msgs::Float32> sub_axis_right_thruster("axis_right_thruster", axis_right_thruster_cb);

ros::Subscriber<std_msgs::Float32> sub_axis_rotary_f("axis_rotary_f", axis_rotary_f_cb);
ros::Subscriber<std_msgs::Float32> sub_axis_rotary_g("axis_rotary_g", axis_rotary_g_cb);

ros::Subscriber<std_msgs::Float32> sub_axis_thruster_thumb_stick_x("ax_thrust_st_x",  axis_thruster_thumb_stick_x_cb);
ros::Subscriber<std_msgs::Float32> sub_axis_thruster_thumb_stick_y("ax_thrust_st_y",  axis_thruster_thumb_stick_y_cb);

ros::Subscriber<std_msgs::Bool> sub_button_rotary_f("button_rotary_f", button_rotary_f_cb);
ros::Subscriber<std_msgs::Bool> sub_button_rotary_g("button_rotary_g", button_rotary_g_cb);

ros::Subscriber<std_msgs::Bool> sub_button_i("button_i", button_i_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h("button_h", button_h_cb);

ros::Subscriber<std_msgs::Bool> sub_button_sw1("button_sw1", button_sw1_cb);
ros::Subscriber<std_msgs::Bool> sub_button_sw2("button_sw2", button_sw2_cb);
ros::Subscriber<std_msgs::Bool> sub_button_sw3("button_sw3", button_sw3_cb);
ros::Subscriber<std_msgs::Bool> sub_button_sw4("button_sw4", button_sw4_cb);
ros::Subscriber<std_msgs::Bool> sub_button_sw5("button_sw5", button_sw5_cb);
ros::Subscriber<std_msgs::Bool> sub_button_sw6("button_sw6", button_sw6_cb);

ros::Subscriber<std_msgs::Bool> sub_button_tgl1_up("button_tgl1_up", button_tgl1_up_cb);
ros::Subscriber<std_msgs::Bool> sub_button_tgl1_down("button_tgl1_down", button_tgl1_down_cb);

ros::Subscriber<std_msgs::Bool> sub_button_tgl2_up("button_tgl2_up", button_tgl2_up_cb);
ros::Subscriber<std_msgs::Bool> sub_button_tgl2_down("button_tgl2_down", button_tgl2_down_cb);

ros::Subscriber<std_msgs::Bool> sub_button_tgl3_up("button_tgl3_up", button_tgl3_up_cb);
ros::Subscriber<std_msgs::Bool> sub_button_tgl3_down("button_tgl3_down", button_tgl3_down_cb);

ros::Subscriber<std_msgs::Bool> sub_button_tgl4_up("button_tgl4_up", button_tgl4_up_cb);
ros::Subscriber<std_msgs::Bool> sub_button_tgl4_down("button_tgl4_down", button_tgl4_down_cb);

ros::Subscriber<std_msgs::Bool> sub_button_h3_up("button_h3_up", button_h3_up_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h3_right("button_h3_right", button_h3_right_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h3_down("button_h3_down", button_h3_down_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h3_left("button_h3_left", button_h3_left_cb);

ros::Subscriber<std_msgs::Bool> sub_button_h4_up("button_h4_up", button_h4_up_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h4_right("button_h4_right", button_h4_right_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h4_down("button_h4_down", button_h4_down_cb);
ros::Subscriber<std_msgs::Bool> sub_button_h4_left("button_h4_left", button_h4_left_cb);

ros::Subscriber<std_msgs::Bool> sub_button_ki_up("button_ki_up", button_ki_up_cb);
ros::Subscriber<std_msgs::Bool> sub_button_ki_down("button_ki_down", button_ki_down_cb);

ros::Subscriber<std_msgs::Bool> sub_button_thumb_stick("button_thumb_stick", button_thumb_stick_cb);
ros::Subscriber<std_msgs::Bool> sub_button_thumb_slider("button_thumb_slider", button_thumb_slider_cb);

ros::Subscriber<std_msgs::Bool> sub_button_scroll_forward("button_scroll_forward", button_scroll_forward_cb);
ros::Subscriber<std_msgs::Bool> sub_button_scroll_reverse("button_scroll_reverse", button_scroll_reverse_cb);

ros::Subscriber<std_msgs::Bool> sub_button_m1("button_m1", button_m1_cb);
ros::Subscriber<std_msgs::Bool> sub_button_m2("button_m2", button_m2_cb);
ros::Subscriber<std_msgs::Bool> sub_button_s1("button_s1", button_s1_cb);
*/

/*
 * Need to test
 */
//Arduino setup
void setup()
{
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
  /*
  nh.subscribe(sub_rotary_3);
  nh.subscribe(sub_joystick_x);
  nh.subscribe(sub_joystick_y);
  nh.subscribe(sub_thumb_stick_x);
  nh.subscribe(sub_thumb_stick_y);
  nh.subscribe(sub_joystick_rotation);
  nh.subscribe(sub_axis_pov_x);
  nh.subscribe(sub_axis_pov_y);

  nh.subscribe(sub_button_b);
  nh.subscribe(sub_button_c);
  nh.subscribe(sub_button_d);
  nh.subscribe(sub_button_pinky_trigger);

  nh.subscribe(sub_button_h1_up);
  nh.subscribe(sub_button_h1_right);
  nh.subscribe(sub_button_h1_down);
  nh.subscribe(sub_button_h1_left);

  nh.subscribe(sub_button_h2_up);
  nh.subscribe(sub_button_h2_right);
  nh.subscribe(sub_button_h2_down);
  nh.subscribe(sub_button_h2_left);

  nh.subscribe(sub_axis_left_thruster);
  nh.subscribe(sub_axis_right_thruster);
 
  nh.subscribe(sub_axis_rotary_f);
  nh.subscribe(sub_axis_rotary_g);

  nh.subscribe(sub_axis_thruster_thumb_stick_x);
  nh.subscribe(sub_axis_thruster_thumb_stick_y);

  nh.subscribe(sub_button_rotary_f);
  nh.subscribe(sub_button_rotary_g);

  nh.subscribe(sub_button_i);
  nh.subscribe(sub_button_h);

  nh.subscribe(sub_button_sw1);
  nh.subscribe(sub_button_sw2);
  nh.subscribe(sub_button_sw3);
  nh.subscribe(sub_button_sw4);
  nh.subscribe(sub_button_sw5);
  nh.subscribe(sub_button_sw6);

  nh.subscribe(sub_button_tgl1_up);
  nh.subscribe(sub_button_tgl1_down);

  nh.subscribe(sub_button_tgl2_up);
  nh.subscribe(sub_button_tgl2_down);
    
  nh.subscribe(sub_button_tgl3_up);
  nh.subscribe(sub_button_tgl3_down);

  nh.subscribe(sub_button_tgl4_up);
  nh.subscribe(sub_button_tgl4_down);

  nh.subscribe(sub_button_h3_up);
  nh.subscribe(sub_button_h3_right);
  nh.subscribe(sub_button_h3_down);
  nh.subscribe(sub_button_h3_left);

  nh.subscribe(sub_button_h4_up);
  nh.subscribe(sub_button_h4_right);
  nh.subscribe(sub_button_h4_down);
  nh.subscribe(sub_button_h4_left);

  nh.subscribe(sub_button_ki_up);
  nh.subscribe(sub_button_ki_down);

  nh.subscribe(sub_button_thumb_stick);
  nh.subscribe(sub_button_thumb_slider);

  nh.subscribe(sub_button_scroll_forward);
  nh.subscribe(sub_button_scroll_reverse);

  nh.subscribe(sub_button_m1);
  nh.subscribe(sub_button_m2);
  nh.subscribe(sub_button_s1);
*/
/*
 *Need to test:
  */
}

//everything will happen in our call back functions, no need for anything here
//could potentially add automatic leveling code here
void loop()
{ 
  nh.spinOnce();
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
