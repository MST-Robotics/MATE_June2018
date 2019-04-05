/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT, Nathan Welch
 * Email: vgmcn3@mst,edu, tjsxz4@mst.edu, nawwx3@mst.edu
 * Date: 04-04-2019
 * 
 * 
 */

#include "x360_motion_processing_node.h"

//MOVE THESE DEFINES TO THE HEADER FILE
#define FORCE_X_MODIFIER 1 /*To Be Determined*/
#define FORCE_Y_MODIFIER 1 /*To Be Determined*/
#define MOTOR_NEUTRAL 1500
#define VERTICAL_SCALE 400
#define MOTOR_RAMP -400 /*Make sure this is negative*/

/*This is our main function
 *Pre: None
 *Post: Necessary publishers and subscribers are created to read controller data and publish motor data
 */
int main(int argc, char **argv)
{
    //necessary ros things
    ros::init(argc, argv, "motion_processing");
    ros::NodeHandle n;

    //subscribe to current sensor data
    ros::Subscriber joystick_thumb_x_topic = n.subscribe("joystick_x_topic", 1000, velocity_callback);
    ros::Subscriber joystick_thumb_y_topic = n.subscribe("joystick_y_topic", 1000, angle_callback);

    ros::Publisher motor1x_pub = n.advertise<std_msgs::Int16>("motor1x_topic", 100);
    ros::Publisher motor2x_pub = n.advertise<std_msgs::Int16>("motor2x_topic", 100);

    ros::Rate loop_wait(15); //this is needed

    //ctr-c makes ok() return false, thus ending the program
    while (ros::ok())
    {
        calc_motors();
        motor1_pub.publish(motor1_value);
        motor2_pub.publish(motor2_value);

        ros::spinOnce();
        loop_wait.sleep(); //wait some
    }
    return 0;
}

void angle_callback(const std_msgs::Float32 &msg)
{
    angle = msg.data;
}

void velocity_callback(const std_msgs::Float32 &msg)
{
    magnitude = msg.data;
}

//This exists.
float normalize_400(float x)
{
    return (x > 400 ? 400 : (x < -400 ? -400 : x));
}

/* calc_motors handles data from velocity_callback, angle_callback, and twist_callback to calculate ROV motor movement
 * Pre: magnitude, angle, and moment are initialized
 * Post: Any variables are updated to their current values for each itteration
 */
void calc_motors()
{
    float force_x = FORCE_X_MODIFIER * magnitude * cos(angle * M_PI / 180);
    float force_y = FORCE_Y_MODIFIER * magnitude * sin(angle * M_PI / 180);

    motor1_value.data = MOTOR_NEUTRAL + MOTOR_RAMP * horizontal_precision * normalize_400(-force_y - force_x + moment);
    motor2_value.data = MOTOR_NEUTRAL + MOTOR_RAMP * horizontal_precision * normalize_400(-force_y - force_x + moment);
}
