#ifndef PWM_CHANNELS_H
#define PWM_CHANNELS_H

/*
 *	Author: Vinnie Marco
 *	Email: vgmcn3@mst.edu
 *	Date: 3-21-2018
 
 *	Description: This file contains information about pwm controller
 *  	channel functions defined by ROV_motor_driver_board_R01
 */
 
 
/*
 * PWM Controller 1 (Core functions)
 */
//Thrusters 1-6 
#define motor_6 0
#define motor_4 1
#define motor_2 2
#define motor_1 3
#define motor_3 4
#define motor_5 5

//Manipulator joints
#define claw_a 6
#define claw_b 7

#define wrist_a 8
#define wrist_b 9

#define crank_a 10
#define crank_b 11

#define extender_a 12
#define extender_b 13

//Gimbal Servo channels
#define gimbal_x 14
#define gimbal_y 15


/*
 * PWM Controller 2 (Spare funtions)
 */ 
//Thrusters 7-8
#define motor_7 0
#define motor_8 1

//Space DC motor channels
#define spare_1a 2
#define spare 1b 3

#define spare_2a 4 
#define spare_3b 5

#define spare_3a 6
#define spare_3b 7

//Spare servo motor channels
#define spare_servo_1 8
#define spare_servo_2 9
#define spare_servo_3 10
#define spare_servo_4 11

//PWM12-15 not implemented



#endif