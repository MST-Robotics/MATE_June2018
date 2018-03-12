/**
 *Author: 	Tamara Spivey
 *Date:		3-10-2018
 *Email: 	tjsxz4@mst.edu
 *
 * 
 *Description: 	This file names buttons and axes for 
 *		cleaner reference with joy node with an xbox 360 wireless controller
 */

#ifndef xbox_controller_H
#define xbox_controller_H


///////////////////////////////
////////////Buttons////////////
///////////////////////////////


/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Button A
 *
 *USAGE: 	
 */
#define button_A 0

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Button B
 *
 *USAGE: 	
 */
#define button_B 1

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Button X
 *
 *USAGE: 	
 */
#define button_X 2

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Button Y
 *
 *USAGE: 	
 */
#define button_Y 3

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Left bumper
 *
 *USAGE: 	
 */
#define button_LB 4

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Right bumper
 *
 *USAGE: 	
 */
#define button_RB 5

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Back button
 *
 *USAGE: 	
 *Mapping: done	
 */
#define button_back 6

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Start button
 *
 *USAGE: 	
 */
#define button_start 7

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Power button
 *
 *USAGE: 	
 */
#define button_power 8

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    left analog stick
 *
 *USAGE: 	
 */
#define button_stick_left 9

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Right analog stick
 *
 *USAGE: 	
 */
#define button_stick_right 10

///////////////////////////////
/////////Analog Axes///////////
///////////////////////////////

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    Left joystick left and right
 *		  X-axis value: left = positive dir, right = negative dir
 *USAGE: 	
 */	
#define left_axis_stick_x 0

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    Left joystick up and down
 *		  y-axis value: up = positive dir, down = negative dir
 *USAGE: 	
 */
#define left_axis_stick_y 1

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    Right joystick left and right
 *		  X-axis value: left = positive dir, right = negative dir
 *USAGE: 		
 */
#define right_axis_stick_x 2

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    Right joystick up and down
 *		  X-axis value: up = positive dir, down = negative dir
 *USAGE: 	
 */
#define right_axis_stick_y 3

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0(default)
 *Nomial Value:	 0.0
 *
 *Description:    Right Trigger
 *		  RT values: default = 1, Fully pressed = -1
 *USAGE: 	
 */
#define axis_RT 4

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0(default)
 *Nomial Value:	 0.0
 *
 *Description:    Left Trigger
 *		  LT values: default = 1, Fully pressed = -1
 *USAGE: 	
 */
#define axis_LT 5

/**
 *Type:		3-state - float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    Directional pad left and right
 *		  X-axis value left = +1, right = -1
 *USAGE: 		
 */
#define axis_cross_key_x 6

/**
 *Type:		3-state - float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    Directional pad up and down
 *		  X-axis value up = +1, down = -1
 *USAGE:
 */
#define axis_cross_key_y 7



#endif
