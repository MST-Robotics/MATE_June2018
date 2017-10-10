/**
 *Author: 	Vinnie Marco
 *Date:		6-15-2017
 *Email: 	vgmcn3@mst.edu
 *
 * 
 *Description: 	This file names buttons and axes for 
 *		cleaner reference with joy node with x56-Rhino flight stick
 */


#ifndef x56_joystick_H
#define x56_joystick_H

///////////////////////////////
/////////Analog Axes///////////
///////////////////////////////

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    X-axis value: left = positive dir, right = negative dir
 *
 *USAGE: 	
 *Mapping: done	
 */
#define axis_stick_x 0

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    Y-axis value: up = positive dir, down = negative dir
 *
 *USAGE: 	
 *Mapping: done	
 */
#define axis_stick_y 1

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    X-axis value left = positive dir, right = negative dir
 *
 *USAGE: 	
 *Mapping: done	
 */
#define axis_joystick_thumb_stick_x 2

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    X-axis value up = positive dir, down = negative dir
 *
 *USAGE: 
 *Mapping: done		
 */
#define axis_joystick_thumb_stick_y 3

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    Stick rotation, left = positive dir, right = negative dir
 *
 *USAGE:
 *Mapping: done 		
 */
#define axis_stick_rotation 4

/**
 *Type:		3-state - float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    X-axis value left = +1, right = -1
 *
 *USAGE:
 *Mapping: done 		
 */
#define axis_pov_x 5

/**
 *Type:		3-state - float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:    X-axis value up = +1, down = -1
 *
 *USAGE:
 *Mapping: done 		
 */
#define axis_pov_y 6



///////////////////////////////
////////////Buttons////////////
///////////////////////////////

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Trigger on joystick
 *
 *USAGE: 	
 *Mapping: done	
 */
#define button_trigger 0

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Blue button accessible by thumb
 *
 *USAGE:
 *Mapping: done 		
 */
#define button_a 1

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Blue button accessible by index finger
 *
 *USAGE: 	
 *Mapping: done	
 */
#define button_b 2

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Thumbstick button
 *
 *USAGE:
 *Mapping: done 		
 */
#define button_c 3

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Blue button accessible by pinky
 *
 *USAGE: 	
 *Mapping: done
 */
#define button_d 4

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Trigger accessible by pinky
 *
 *USAGE: 
 *Mapping: done		
 */
#define button_pinky_trigger 5

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    conical 4-directional pad
 *
 *USAGE:
 *Mapping: done 		
 */
#define button_h1_up 6
#define button_h1_right 7
#define button_h1_down 8
#define button_h1_left 9

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    4-directional pad
 *
 *USAGE: 	
 *Mapping: done	
 */
#define button_h2_up 10
#define button_h2_right 11
#define button_h2_down 12
#define button_h2_left 13


#endif
