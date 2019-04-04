/*
 * Author: Vinnie Marco, Tamara Spivey, and the Most Honorable Christian Upschulte: President of S&T URT, Nathan Welch
 * Email: vgmcn3@mst,edu, tjsxz4@mst.edu, nawwx3@mst.edu
 * 
 * This file handles data from the xbox controller
 */

#include "x360_processing_node.h"

/*This is our main function
 *Pre: None
 *Post: Necessary publishers and subscribers are created to read controller data
 */
int main(int argc, char **argv)
{
    //necessary ros things
    ros::init(argc, argv, "x360_processing_node"); //set up node named
    ros::NodeHandle n;                             //create node handle instance

    //set up subscriptions  
    ros::Subscriber x360_topic = n.subscribe("x360_topic", 1000, joystick_callback); //subscribe to joystick values

    //set up publishers
    ros::Publisher joystick_thumb_x_pub = n.advertise<std_msgs::Float32>("joystick_x_topic", 1000);
    ros::Publisher joystick_thumb_y_pub = n.advertise<std_msgs::Float32>("joystick_y_topic", 1000);

    ros::Rate loop_wait(30); //this is needed

    while (ros::ok()) //ctrl-c makes ok() return false, thus ending the program
    {
        //publish everything once per loop
        joystick_thumb_x_pub.publish(axis_joystick_thumb_stick_x_value);
        joystick_thumb_y_pub.publish(axis_joystick_thumb_stick_y_value);

        ros::spinOnce();
        loop_wait.sleep(); //wait some
    }
    return 0;
}

/* joystick_callback handles data recieved from the joystick topic
 * Pre: Joystick topic has to be initalized
 * Post: Any variables are updated to their current values for each itteration
 */
void joystick_callback(const sensor_msgs::Joy &joy)
{
    angle_value.data = atan2(joy.axes[axis_stick_right_y], joy.axes[axis_stick_right_x] * -1);
    magnitude_value.data = sqrt(pow(joy.axes[axis_stick_right_y], 2) + pow(joy.axes[axis_stick_right_x], 2));
    // axis_stick_rotation_value.data = joy.axes[axis_stick_rotation];

    angle_value.data *= 180 / M_PI; //built in value for Pi

    if (angle_value.data < 0) //conversion to positive degree value
    {
        angle_value.data += 360;
    }

    if (magnitude_value.data > 1.0) //magnitude is a scale of our speed, from [0,1]
    {
        magnitude_value.data = 1.0; //mapped to one to match our scale
    }
    if (magnitude_value.data < 0.0)
    {
        magnitude_value.data = 0.0; //mapped to zero to match our scale
    }

    /*
  button_b_state.data = joy.buttons[button_b];
  button_c_state.data = joy.buttons[button_c];
  button_d_state.data = joy.buttons[button_d];

  button_h1_up_state.data = joy.buttons[button_h1_up];
  button_h1_right_state.data = joy.buttons[button_h1_right];
  button_h1_down_state.data = joy.buttons[button_h1_down];
  button_h1_left_state.data = joy.buttons[button_h1_left];

  button_h2_up_state.data = joy.buttons[button_h2_up];
  button_h2_right_state.data = joy.buttons[button_h2_right];
  button_h2_down_state.data = joy.buttons[button_h2_down];
  button_h2_left_state.data = joy.buttons[button_h2_left];

  axis_pov_x_value.data = joy.axes[axis_pov_x];
  axis_pov_y_value.data = joy.axes[axis_pov_y];
*/
}