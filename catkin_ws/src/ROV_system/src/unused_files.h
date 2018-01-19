//unused

/*
  ros::Publisher button_a_pub = n.advertise<std_msgs::Bool>("button_a", 1000);
  ros::Publisher button_e_pub = n.advertise<std_msgs::Bool>("button_e", 1000);
  ros::Publisher rotary_4_pub = n.advertise<std_msgs::Float32>("rotary_4", 1000);
  ros::Publisher rotary_3_pub = n.advertise<std_msgs::Float32>("rotary_3", 1000);

  ros::Publisher thumb_stick_x_pub = n.advertise<std_msgs::Float32>("thumb_stick_x", 1000);
  ros::Publisher thumb_stick_y_pub = n.advertise<std_msgs::Float32>("thumb_stick_y", 1000);
  ros::Publisher axis_pov_x_pub = n.advertise<std_msgs::Float32>("axis_pov_x", 1000);
  ros::Publisher axis_pov_y_pub = n.advertise<std_msgs::Float32>("axis_pov_y", 1000);

  ros::Publisher button_b_pub = n.advertise<std_msgs::Bool>("button_b", 1000);
  ros::Publisher button_c_pub = n.advertise<std_msgs::Bool>("button_c", 1000);
  ros::Publisher button_d_pub = n.advertise<std_msgs::Bool>("button_d", 1000);

  ros::Publisher button_h1_up_pub = n.advertise<std_msgs::Bool>("button_h1_up", 1000);
  ros::Publisher button_h1_right_pub = n.advertise<std_msgs::Bool>("button_h1_right", 1000);
  ros::Publisher button_h1_down_pub = n.advertise<std_msgs::Bool>("button_h1_down", 1000);
  ros::Publisher button_h1_left_pub = n.advertise<std_msgs::Bool>("button_h1_left", 1000);

  ros::Publisher button_h2_up_pub = n.advertise<std_msgs::Bool>("button_h2_up", 1000);
  ros::Publisher button_h2_right_pub = n.advertise<std_msgs::Bool>("button_h2_right", 1000);
  ros::Publisher button_h2_down_pub = n.advertise<std_msgs::Bool>("button_h2_down", 1000);
  ros::Publisher button_h2_left_pub = n.advertise<std_msgs::Bool>("button_h2_left", 1000);

  ros::Publisher axis_left_thruster_pub = n.advertise<std_msgs::Float32>("axis_left_thruster", 1000);
  ros::Publisher axis_right_thruster_pub = n.advertise<std_msgs::Float32>("axis_right_thruster", 1000);

  ros::Publisher axis_rotary_f_pub = n.advertise<std_msgs::Float32>("axis_rotary_f", 1000);
  ros::Publisher axis_rotary_g_pub = n.advertise<std_msgs::Float32>("axis_rotary_g", 1000);

  ros::Publisher axis_thruster_thumb_stick_x_pub = n.advertise<std_msgs::Float32>("ax_thrust_st_x", 1000);
  ros::Publisher axis_thruster_thumb_stick_y_pub = n.advertise<std_msgs::Float32>("ax_thrust_st_y", 1000);

  ros::Publisher button_rotary_f_pub = n.advertise<std_msgs::Bool>("button_rotary_f", 1000);
  ros::Publisher button_rotary_g_pub = n.advertise<std_msgs::Bool>("button_rotary_g", 1000);

  ros::Publisher button_i_pub = n.advertise<std_msgs::Bool>("button_i", 1000);
  ros::Publisher button_h_pub = n.advertise<std_msgs::Bool>("button_h", 1000);

  ros::Publisher button_sw1_pub = n.advertise<std_msgs::Bool>("button_sw1", 1000);
  ros::Publisher button_sw2_pub = n.advertise<std_msgs::Bool>("button_sw2", 1000);
  ros::Publisher button_sw3_pub = n.advertise<std_msgs::Bool>("button_sw3", 1000);
  ros::Publisher button_sw4_pub = n.advertise<std_msgs::Bool>("button_sw4", 1000);
  ros::Publisher button_sw5_pub = n.advertise<std_msgs::Bool>("button_sw5", 1000);
  ros::Publisher button_sw6_pub = n.advertise<std_msgs::Bool>("button_sw6", 1000);

  ros::Publisher button_tgl1_up_pub = n.advertise<std_msgs::Bool>("button_tgl1_up", 1000);
  ros::Publisher button_tgl1_down_pub = n.advertise<std_msgs::Bool>("button_tgl1_down", 1000);

  ros::Publisher button_tgl2_up_pub = n.advertise<std_msgs::Bool>("button_tgl2_up", 1000);
  ros::Publisher button_tgl2_down_pub = n.advertise<std_msgs::Bool>("button_tgl2_down", 1000);

  ros::Publisher button_tgl3_up_pub = n.advertise<std_msgs::Bool>("button_tgl3_up", 1000);
  ros::Publisher button_tgl3_down_pub = n.advertise<std_msgs::Bool>("button_tgl3_down", 1000);

  ros::Publisher button_tgl4_up_pub = n.advertise<std_msgs::Bool>("button_tgl4_up", 1000);
  ros::Publisher button_tgl4_down_pub = n.advertise<std_msgs::Bool>("button_tgl4_down", 1000);

  ros::Publisher button_h3_up_pub = n.advertise<std_msgs::Bool>("button_h3_up", 1000);
  ros::Publisher button_h3_right_pub = n.advertise<std_msgs::Bool>("button_h3_right", 1000);
  ros::Publisher button_h3_down_pub = n.advertise<std_msgs::Bool>("button_h3_down", 1000);
  ros::Publisher button_h3_left_pub = n.advertise<std_msgs::Bool>("button_h3_left", 1000);
 
  ros::Publisher button_h4_up_pub = n.advertise<std_msgs::Bool>("button_h4_up", 1000);
  ros::Publisher button_h4_right_pub = n.advertise<std_msgs::Bool>("button_h4_right", 1000);
  ros::Publisher button_h4_down_pub = n.advertise<std_msgs::Bool>("button_h4_down", 1000);
  ros::Publisher button_h4_left_pub = n.advertise<std_msgs::Bool>("button_h4_left", 1000);

  ros::Publisher button_ki_up_pub = n.advertise<std_msgs::Bool>("button_ki_up", 1000);
  ros::Publisher button_ki_down_pub = n.advertise<std_msgs::Bool>("button_ki_down", 1000);

  ros::Publisher button_thumb_stick_pub = n.advertise<std_msgs::Bool>("button_thumb_stick", 1000);
  ros::Publisher button_thumb_slider_pub = n.advertise<std_msgs::Bool>("button_thumb_slider", 1000);

  ros::Publisher button_scroll_forward_pub = n.advertise<std_msgs::Bool>("button_scroll_forward", 1000);
  ros::Publisher button_scroll_reverse_pub = n.advertise<std_msgs::Bool>("button_scroll_reverse", 1000);

  ros::Publisher button_m1_pub = n.advertise<std_msgs::Bool>("button_m1", 1000);
  ros::Publisher button_m2_pub = n.advertise<std_msgs::Bool>("button_m2", 1000);
  ros::Publisher button_s1_pub = n.advertise<std_msgs::Bool>("button_s1", 1000);
*/


/* 
    button_a_pub.publish(button_a_state);
    button_e_pub.publish(button_e_state);
    rotary_4_pub.publish(rotary_4_value);
    rotary_3_pub.publish(rotary_3_value);
  
    thumb_stick_x_pub.publish(axis_joystick_thumb_stick_x_value);
    thumb_stick_y_pub.publish(axis_joystick_thumb_stick_y_value);
    axis_pov_x_pub.publish(axis_pov_x_value);
    axis_pov_y_pub.publish(axis_pov_y_value);

    button_b_pub.publish(button_b_state);
    button_c_pub.publish(button_c_state);
    button_d_pub.publish(button_d_state);

    button_h1_up_pub.publish(button_h1_up_state);
    button_h1_right_pub.publish(button_h1_right_state);
    button_h1_down_pub.publish(button_h1_down_state);
    button_h1_left_pub.publish(button_h1_left_state);

    button_h2_up_pub.publish(button_h2_up_state);
    button_h2_right_pub.publish(button_h2_right_state);
    button_h2_down_pub.publish(button_h2_down_state);
    button_h2_left_pub.publish(button_h2_left_state);

    axis_left_thruster_pub.publish(axis_left_thruster_value);
    axis_right_thruster_pub.publish(axis_right_thruster_value);

    axis_rotary_f_pub.publish(axis_rotary_f_value);
    axis_rotary_g_pub.publish(axis_rotary_g_value);

    axis_thruster_thumb_stick_x_pub.publish(axis_thruster_thumb_stick_x_value);
    axis_thruster_thumb_stick_y_pub.publish(axis_thruster_thumb_stick_y_value);

    button_rotary_f_pub.publish(button_rotary_f_state);
    button_rotary_g_pub.publish(button_rotary_g_state);


    button_i_pub.publish(button_i_state);
    button_h_pub.publish(button_h_state);

    button_sw1_pub.publish(button_sw1_state);
    button_sw2_pub.publish(button_sw2_state);
    button_sw3_pub.publish(button_sw3_state);
    button_sw4_pub.publish(button_sw4_state);
    button_sw5_pub.publish(button_sw5_state);
    button_sw6_pub.publish(button_sw6_state);

    button_tgl1_up_pub.publish(button_tgl1_up_state);
    button_tgl1_down_pub.publish(button_tgl1_down_state);

    button_tgl2_up_pub.publish(button_tgl2_up_state);
    button_tgl2_down_pub.publish(button_tgl2_down_state);

    button_tgl3_up_pub.publish(button_tgl3_up_state);
    button_tgl3_down_pub.publish(button_tgl3_down_state);

    button_tgl4_up_pub.publish(button_tgl4_up_state);
    button_tgl4_down_pub.publish(button_tgl4_down_state);

    button_h3_up_pub.publish(button_h3_up_state);
    button_h3_right_pub.publish(button_h3_right_state);
    button_h3_down_pub.publish(button_h3_down_state);
    button_h3_left_pub.publish(button_h3_left_state);

    button_h4_up_pub.publish(button_h4_up_state);
    button_h4_right_pub.publish(button_h4_right_state);
    button_h4_down_pub.publish(button_h4_down_state);
    button_h4_left_pub.publish(button_h4_left_state);

    button_ki_up_pub.publish(button_ki_up_state);
    button_ki_down_pub.publish(button_ki_down_state);

    button_thumb_stick_pub.publish(button_thumb_stick_state);
    button_thumb_slider_pub.publish(button_thumb_slider_state);

    button_scroll_forward_pub.publish(button_scroll_forward_state);
    button_scroll_reverse_pub.publish(button_scroll_reverse_state);

    button_m1_pub.publish(button_m1_state);
    button_m2_pub.publish(button_m2_state);
    button_s1_pub.publish(button_s1_state);
*/
