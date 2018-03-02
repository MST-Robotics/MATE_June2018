/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst.edu
 * 
 * Date: 10-10-2017
 * 
 * Description: Basic usage for onboard 
 *      external servo controller
 */

#include <ROV_main_setup.h>

//define pin 7 as a servo pin
#define servo 7

void setup() 
{
  bool error_flag = main_setup();//check for an error
  if(error_flag)
  {
    //Serial.println("ERROR");//prints error message if serial is started
    while(1);
  }
}

void loop() 
{  
  external_servo.writeMicroseconds(servo, 1000);//max one direction
  delay(2000);
  external_servo.writeMicroseconds(servo, 2000);//max other direction
  delay(2000);
}
