/**
 *Author: 	Vinnie Marco
 *Date:		6-15-2017
 *Email: 	vgmcn3@mst.edu
 *
 * 
 *Description: 	This file names buttons and axes for 
 *		cleaner reference with joy node with x56-Rhino thruster
 */


#ifndef x56_thruster_H
#define x56_thruster_H

///////////////////////////////
/////////Analog Axes///////////
///////////////////////////////

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0 - holds position
 *
 *Description:   Left thruster: forward = positive dir, reverse = negative dir
 *
 *USAGE: 
 *Mapping: done		
 */
#define axis_left_thruster 0

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0 - holds position
 *
 *Description:   Right thruster: forward = positive dir, reverse = negative dir
 *
 *USAGE: 	
 *Mapping: done	
 */
#define axis_right_thruster 1

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0 - holds position
 *
 *Description:   Top Throttle rotary, cw = negative, ccw = positive
 *
 *USAGE: 		
 *Mapping: done
 */
#define axis_rotary_f 2

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:   X-axis, left = positive dir, right = negative dir
 *
 *USAGE: 		
 *Mapping: done
 */
#define axis_thruster_thumb_stick_x 3

/**
 *Type:		Analog - Float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0
 *
 *Description:   Y-axis, up = positive dir, down = negative dir
 *
 *USAGE: 		
 *Mapping: done
 */
#define axis_thruster_thumb_stick_y 4

/**
 *Type:		Analog - float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0 - holds position
 *
 *Description:    Bottom Throttle rotary 3, cw = negative, ccw = positive
 *
 *USAGE: 		
 *Mapping: done
 */
#define axis_rotary_g 5

/**
 *Type:		Analog - float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0 - holds position
 *
 *Description:   Base rotary, cw = negative, ccw = positive
 *
 *USAGE: 
 *Mapping: done		
 */
#define axis_base_rotary_4 6

/**
 *Type:		 Analog - float32
 *Min Value:	-1.0
 *Max Value:	 1.0
 *Nomial Value:	 0.0 - holds position
 *
 *Description:   Base rotary, cw = negative, ccw = positive
 *
 *USAGE: 
 *Mapping: done		
 */
#define axis_base_rotary_3 7



///////////////////////////////
////////////Buttons////////////
///////////////////////////////

/**
 *Type:		 Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Blue button accessible by thumb
 *
 *USAGE: 		
 *Mapping: done
 */
#define button_e 0

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Top throttle rotary button
 *
 *USAGE: 
 *Mapping: done		
 */
#define button_rotary_f 1

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Bottom throttle rotary button
 *
 *USAGE: 
 *Mapping: done		
 */
#define button_rotary_g 2

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Throttle button accessible by middle finger
 *
 *USAGE: 	
 *Mapping: done		
 */
#define button_i 3

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Throttle button accessible by index finger
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_h 4

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Toggle Switch on front
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_sw1 5

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Toggle Switch on front
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_sw2 6

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Toggle Switch on front
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_sw3 7

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 * 
 *Description:    Toggle Switch on front
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_sw4 8

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Toggle Switch on front
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_sw5 9

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Toggle Switch on front
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_sw6 10

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Toggle Switch 1 on right side
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_tgl1_up 11
#define button_tgl1_down 12

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Toggle Switch 2 on right side
 *
 *USAGE: 	
 *Mapping: done		
 */
#define button_tgl2_up 13
#define button_tgl2_down 14

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Toggle Switch 3 on right side
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_tgl3_up 15
#define button_tgl3_down 16

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Toggle Switch 4 on right side
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_tgl4_up 17
#define button_tgl4_down 18

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Top directional stick H3 accessible by thumb
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_h3_up 19
#define button_h3_right 20
#define button_h3_down 21
#define button_h3_left 22

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Bottom directional stick H4 accessible by thumb
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_h4_up 23
#define button_h4_right 24
#define button_h4_down 25
#define button_h4_left 26

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0
 *
 *Description:    Black pivot button Ki on left thruster
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_ki_up 27
#define button_ki_down 28

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1 - pulses to 1 per tick
 *Released State: 0
 *
 *Description:    Left thruster scroll wheel
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_scroll_forward 29
#define button_scroll_reverse 30

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
#define button_thumb_stick 31

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Released State: 0 - holds Position
 *
 *Description:    Thumb slider SLD
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_thumb_slider 32

/**
 *Type:		  Digital - int32
 *Min Value:	  0
 *Max Value:	  1
 *Selected State: 0 - holds Position
 *
 *Description:    Mode Selector, 1 on selected choice
 *
 *USAGE: 		
 *Mapping: done	
 */
#define button_m1 33
#define button_m2 34
#define button_s1 35



#endif
