/*
 *	Author: Vinnie Marco
 *	Email: vgmcn3@mst.edu
 *	Date: 10-10-2017
 
 *	Description: main setup function for onboard hardware
 */

#ifndef ROV_MAIN_SETUP_H
#define ROV_MAIN_SETUP_H

#include "ROV_main.h"
#include "Arduino.h"

#include <SPI.h>
#include <SD.h>

//cs pin for sd card
#define chip_select 22

//comment out this line if not using sd card
#define card_used

LSM9DS1 imu;// imu object

PWMcontroller external_servo = PWMcontroller(0x7f);//pwm controller object

/* sets up all hardware on the main board
 * and returns an error flag if something fails
 */
bool main_setup(void)
{	
  bool error_flag = 0;
  
  //pinMode to set up cs pin on sd card
  pinMode(chip_select, OUTPUT);
  
  
  //IMU setup
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
  
  if(!imu.begin())//returning a zero
    error_flag = 1;	  
  
  //external servo controller setup
  external_servo.begin();
  
  
  //only wory about sd card init errors if using the card
  #ifdef card_used
	if(!SD.begin(chip_select))
		error_flag = 1;
  #endif	
  
  return error_flag;
}

#endif