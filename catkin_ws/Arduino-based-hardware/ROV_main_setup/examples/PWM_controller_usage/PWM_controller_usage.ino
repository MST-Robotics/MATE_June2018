/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst.edu
 * 
 * Date: 3-21-2018
 * 
 * Description: Basic usage for onboard 
 *      external servo controller
 *      
 * NOTE: Motor controller PCB channel implemetations can be
 *      found in "pwm_channels.h"
 */

/*
 * Inlucdes setup for:
 *  IMU hardware on mainboard
 *  Motor controller hardware on motor_driver_board
 */
#include <ROV_main_setup.h>
#include "pwm_channels.h"

void setup() 
{
  bool error_flag = main_setup();//initalize the imu sensor
  if(error_flag)//check for an IMU error
  {
    //Serial.println("ERROR");//prints error message if serial is started
    while(1);//halt if error
  }
}

void loop() 
{  
  pwm_primary.writeMicroseconds(gimbal_x, 1000);//max one direction
  delay(2000);
  pwm_primary.writeMicroseconds(gimbal_x, 2000);//max other direction
  delay(2000);
}
