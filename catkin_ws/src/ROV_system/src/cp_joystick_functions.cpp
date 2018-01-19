#ifndef CP_JOYSTICK_FUNCTIONS
#define CP_JOYSTICK_FUNCTIONS

//process data from joystick
void joystick_cb(const sensor_msgs::Joy &joy)
{
  float angle = atan2(joy.axes[axis_stick_y],  joy.axes[axis_stick_x]*-1);
  float magnitude = sqrt(pow(joy.axes[axis_stick_y],2) + pow(joy.axes[axis_stick_x],2));


  angle *= 180/M_PI;

  if(angle < 0)
  {
    angle += 360;
  }

  if(magnitude > 1.0)
  {
    magnitude = 1.0;
  }
  if(magnitude < 0.0)
  {
    magnitude = 0.0;
  }

  axis_stick_x_value.data = magnitude;
  axis_stick_y_value.data = angle;
  
  axis_stick_rotation_value.data = joy.axes[axis_stick_rotation];



  if(joy.buttons[button_trigger] == 1)
  {
    ROS_INFO("Trigger pulled");
    button_trigger_state.data = 1;
  }
  else
    button_trigger_state.data = 0;


  if(joy.buttons[button_pinky_trigger] == 1)
  {
    ROS_INFO("Button pinky trigger pressed");
    button_pinky_trigger_state.data = 1;
  }
  else
    button_pinky_trigger_state.data = 0;






/*
 //buttons are stored in joy.buttons[]
  //axes are stored in joy.axis[]


  if(joy.buttons[button_a] == 1)
  {
    ROS_INFO("Button A pressed");
    button_a_state.data = 1;
  }
  else
    button_a_state.data = 0;


  if(joy.buttons[button_b] == 1)
  {
    ROS_INFO("Button B pressed");
    button_b_state.data = 1;
  }
  else
    button_b_state.data = 0;

  if(joy.buttons[button_c] == 1)
  {
    ROS_INFO("Button C pressed");
    button_c_state.data = 1;
  }
  else
    button_c_state.data = 0;

  if(joy.buttons[button_d] == 1)
  {
    ROS_INFO("Button D pressed");
    button_d_state.data = 1;
  }
  else
    button_d_state.data = 0;


  if(joy.buttons[button_h1_up] == 1)
  {
    ROS_INFO("Button h1 up pressed");
    button_h1_up_state.data = 1;
  }
  else
    button_h1_up_state.data = 0;

  if(joy.buttons[button_h1_right] == 1)
  {
    ROS_INFO("Button h1 right pressed");
    button_h1_right_state.data = 1;
  }
  else
    button_h1_right_state.data = 0;

  if(joy.buttons[button_h1_down] == 1)
  {
    ROS_INFO("Button h1 down pressed");
    button_h1_down_state.data = 1;
  }
  else
    button_h1_down_state.data = 0;

  if(joy.buttons[button_h1_left] == 1)
  {
    ROS_INFO("Button h1 left pressed");
    button_h1_left_state.data = 1;
  }
  else
    button_h1_left_state.data = 0;


  if(joy.buttons[button_h2_up] == 1)
  {
    ROS_INFO("Button h2 up pressed");
    button_h2_up_state.data = 1;
  }
  else
    button_h2_up_state.data = 0;

  if(joy.buttons[button_h2_right] == 1)
  {
    ROS_INFO("Button h2 right pressed");
    button_h2_right_state.data = 1;
  }
  else
    button_h2_right_state.data = 0;

  if(joy.buttons[button_h2_down] == 1)
  {
    ROS_INFO("Button h2 down pressed");
    button_h2_down_state.data = 1;
  }
  else
    button_h2_down_state.data = 0;

  if(joy.buttons[button_h2_left] == 1)
  {
    ROS_INFO("Button h2 left pressed");
    button_h2_left_state.data = 1;
  }
  else
    button_h2_left_state.data = 0;

  axis_pov_x_value.data = joy.axes[axis_pov_x];
  axis_pov_y_value.data = joy.axes[axis_pov_y];
*/
}
#endif
