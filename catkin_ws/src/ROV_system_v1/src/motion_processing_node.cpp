/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT
 * Email: vgmcn3@mst,edu, tjsxz4@mst.edu
 * Date: 01-18-2018
 * 
 * 
 */

#include "motion_processing_node.h"

//MOVE THESE DEFINES TO THE HEADER FILE
#define FORCE_X_MODIFIER 1 /*To Be Determined*/
#define FORCE_Y_MODIFIER 1 /*To Be Determined*/
#define MOMENT_MODIFIER 1 /*To Be Determined*/
#define MOTOR_NEUTRAL 1500
#define MOTOR_RAMP -400 /*Make sure this is negative*/

/*This is our main function
 *Pre: None
 *Post: Necessary publishers and subscribers are created to read controller data and publish motor data
 */
int main(int argc, char **argv)
{
  //necessary ros things
  ros::init(argc, argv,"motion_processing");
  ros::NodeHandle n;

  //subscribe to current sensor data
  ros::Subscriber joystick_x_topic = n.subscribe("magnitude_topic", 1000, velocity_callback);
  ros::Subscriber joystick_y_topic = n.subscribe("angle_topic", 1000, angle_callback);
  ros::Subscriber joystick_rotation_topic = n.subscribe("joystick_rotation_topic", 1000, twist_callback);
  ros::Subscriber trigger_topic = n.subscribe("trigger_topic", 1000, trigger_callback);
  ros::Subscriber button_pinky_trigger_topic = n.subscribe("button_pinky_trigger_topic", 1000, button_pinky_trigger_callback);

//  ros::Subscriber button_h2_up_topic = n.subscribe("button_h2_up_topic", 1000, button_h2_up_callback);
 // ros::Subscriber button_h2_down_topic = n.subscribe("button_h2_down_topic", 1000, button_h2_down_callback);

  ros::Publisher motor1_pub = n.advertise<std_msgs::Int16>("motor1_topic", 1000);
  ros::Publisher motor2_pub = n.advertise<std_msgs::Int16>("motor2_topic", 1000);
  ros::Publisher motor3_pub = n.advertise<std_msgs::Int16>("motor3_topic", 1000);
  ros::Publisher motor4_pub = n.advertise<std_msgs::Int16>("motor4_topic", 1000);
  ros::Publisher motor5_pub = n.advertise<std_msgs::Int16>("motor5_topic", 1000);
  ros::Publisher motor6_pub = n.advertise<std_msgs::Int16>("motor6_topic", 1000);

 // ros::Publisher camera_one_pub = n.advertise<std_msgs::Int16>("camera_one_topic", 1000);
 // ros::Publisher camera_two_pub = n.advertise<std_msgs::Int16>("camera_two_topic", 1000);

 // ros::Subscriber orientation_topic = n.subscribe("orientation_topic", 1000, orientation_callback);
 
  ros::Rate loop_wait(30);//this is needed
  
  //ctr-c makes ok() return false, thus ending the program
  while(ros::ok())
  { 
    calc_motors();
    motor1_pub.publish(motor1_value);
    motor2_pub.publish(motor2_value);
    motor3_pub.publish(motor3_value);
    motor4_pub.publish(motor4_value);
    motor5_pub.publish(motor5_value);
    motor6_pub.publish(motor6_value);

   // camera_one_pub.publish(camera_one_value);
   // camera_two_pub.publish(camera_two_value);

    ros::spinOnce();
    loop_wait.sleep();//wait some
  }
  return 0;
}

/* velocity_callback handles data recieved from the jo ystick_x_topic subscription
 * Pre: joystick_x_topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration
 */
void velocity_callback(const std_msgs::Float32 &msg)
{ 
  magnitude = msg.data;
}

/* angle_callback handles data recieved from the joystick_y_topic subscription
 * Pre: joystick_y_topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration
 */
void angle_callback(const std_msgs::Float32 &msg)
{ 
  angle = msg.data;
}

/* twist_callback handles data from the "" subscription (the one that handles the twisting of the joystick)
 * Pre: "" has to be initialized
 * Post: Any variables are updated to their current values for each itteration
 */
void twist_callback(const std_msgs::Float32 &msg)
{
  moment = MOMENT_MODIFIER * msg.data; //Neutral: moment = 0 = msg.data;
}

//This exists.
float normalize_400(float x)
{
  return (x>400 ? 400 : (x < -400 ? -400 : x ) );
}

/* calc_motors handles data from velocity_callback, angle_callback, and twist_callback to calculate ROV motor movement
 * Pre: magnitude, angle, and moment are initialized
 * Post: Any variables are updated to their current values for each itteration
 */
void calc_motors()
{
  float force_x = FORCE_X_MODIFIER * magnitude * cos(angle * M_PI / 180);
  float force_y = FORCE_Y_MODIFIER * magnitude * sin(angle * M_PI / 180);
  
  motor4_value.data = MOTOR_NEUTRAL + MOTOR_RAMP * normalize_400(-force_y + force_x - moment);
  motor1_value.data = MOTOR_NEUTRAL + MOTOR_RAMP * normalize_400( -force_y - force_x + moment);
  motor3_value.data = MOTOR_NEUTRAL + MOTOR_RAMP * normalize_400( force_y - force_x - moment);
  motor6_value.data = MOTOR_NEUTRAL + MOTOR_RAMP * normalize_400( force_y + force_x + moment);
}

/* trigger_callback handles data recieved from the trigger_topic subscription
 * Pre: trigger_topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration.
 *       Determines upward movements of ROV.
 */
void trigger_callback(const std_msgs::Bool &msg)
{
  if(msg.data == 1)
  {
    motor2_value.data = 2000;
    motor5_value.data = 2000;
  }
  else
  {
    motor2_value.data = 1500;
    motor5_value.data = 1500;
  }
}

/* button_pinky_trigger_callback handles data recieved from the button_pinky_trigger_topic subscription
 * Pre: button_pinky_trigger_topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration.
 *       Determines downnward movements of ROV.
 */
/*
 *
 * IT DOESN'T GO DOWN. WE'LL SEE
 *
 */
void button_pinky_trigger_callback(const std_msgs::Bool &msg)
{
  if(msg.data == 1)
  {
    motor2_value.data = 1100;
    motor5_value.data = 1100;
  }
  else
  {
    motor2_value.data = 1500;
    motor5_value.data = 1500;
  }
}
/*
default code
void button_h2_up_callback(const std_msgs::Bool &msg)
{
  if(msg.data == 1)
  {
    camera_one_value.data = 2000;
  }
  else
  {
    camera_one_value.data = 1500;
  }
}

void button_h2_down_callback(const std_msgs::Bool &msg)
{
  if(msg.data == 1)
  {
    camera_one_value.data = -2000;
  }
  else
  {
    camera_one_value.data = 1500;
  }
}
*/
