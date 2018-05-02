/*
 *	Author: Vinnie Marco
 *	Email: vgmcn3@mst.edu
 *	Date: 10-10-2017
 
 *	Description: This file contains libraries used for onboard hardware
 */
#include "PWMcontroller.h"

//PWM controller functions
PWMcontroller::PWMcontroller(uint8_t addr) 
{
  _i2caddr = addr;
}

void PWMcontroller::begin(void) 
{
  WIRE.begin();
  reset();
 
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; //sleep
  
  write8(PCA9685_MODE1, newmode); //go to sleep
  write8(PCA9685_PRESCALE, 112); //set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);  //This sets the MODE1 register to turn on auto increment.
}

void PWMcontroller::reset(void) 
{
 write8(PCA9685_MODE1, 0x0);
}

/*Channel must be between 0-15, and microseconds 
must be between 1000-2000*/
void PWMcontroller::writeMicroseconds(uint8_t channel, uint16_t microseconds) 
{	
  //map 1000-2000 usecond range of servo to 150-600 counts out of 4096
  microseconds = map(microseconds, 1000, 2000, 150, 600);
  
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(LED0_ON_L+4*channel);
  WIRE.write(0);
  WIRE.write(0>>8);
  WIRE.write(microseconds);
  WIRE.write(microseconds>>8);
  WIRE.endTransmission();
}

uint8_t PWMcontroller::read8(uint8_t addr) 
{
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(addr);
  WIRE.endTransmission();

  WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return WIRE.read();
}

void PWMcontroller::write8(uint8_t addr, uint8_t d) 
{
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(addr);
  WIRE.write(d);
  WIRE.endTransmission();
}


