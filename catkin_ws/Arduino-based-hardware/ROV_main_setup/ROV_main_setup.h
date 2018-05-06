/*
 *	Author: Vinnie Marco
 *	Email: vgmcn3@mst.edu
 *	Date: 10-10-2017

 *	Description: main setup function for onboard hardware
 */

#ifndef ROV_MAIN_SETUP_H
#define ROV_MAIN_SETUP_H

#include <Arduino.h>

#include "LSM9DS1.h"
#include "PWMcontroller.h"

LSM9DS1 imu;//imu object

PWMcontroller pwm_primary = PWMcontroller(0x40);//pwm controller object
PWMcontroller pwm_secondary = PWMcontroller(0x41);


/* sets up all hardware on the main board
 * and returns an error flag if something fails
 */
bool main_setup(void)
{
  analogReference(EXTERNAL);
  bool error_flag = 0;

  Serial2.begin(9600);//set up the hardware uart2 for communication with manipulator arduino.

  //IMU setup
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if(!imu.begin())//returning a zero
    error_flag = 1;

  //external servo controller setup
  pwm_primary.begin();
  pwm_secondary.begin();

  return error_flag;
}

#endif
