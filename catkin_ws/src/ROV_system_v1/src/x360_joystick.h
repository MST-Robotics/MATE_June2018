/**
 *Author: 	Vinnie Marco
 *Date:		6-15-2017
 *Email: 	vgmcn3@mst.edu
 *
 * 
 * Description: 	This file names buttons and axes for 
 *		cleaner reference with joy node with wireless Xbox 360 controller
 */

#ifndef x360_joystick_H
#define x360_joystick_H

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
// #define axis_joystick_left_x 0

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
// #define axis_stick_left_y 1

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
#define axis_stick_right_x 2

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
#define axis_stick_right_y 3

#endif
