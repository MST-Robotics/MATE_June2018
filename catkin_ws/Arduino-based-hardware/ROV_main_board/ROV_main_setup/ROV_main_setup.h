#ifndef ROV_MAIN_SETUP_H
#define ROV_MAIN_SETUP_H

#include "ROV_main.h"
#include <SPI.h>
#include <SD.h>

//cs pin for sd card
#define chip_select 22

LSM9DS1 imu;// imu object

PWMcontroller external_servo = PWMcontroller(0x7f);//pwm controller object

/* sets up all hardware on the main board
 * and returns an error flag if something fails
 */
bool main_setup(void)
{	
  bool error_flag = 0;
  //IMU setup
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
  error_flag |= imu.begin();  
  
  //external servo controller setup
  external_servo.begin();
  
  //only wory about sd card init errors if using the card
  #ifdef card_used
    error_flag |= SD.begin(chip_select);
  #endif	
  
  return error_flag;
}

#endif