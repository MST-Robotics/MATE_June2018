/*
 *	Author: Vinnie Marco
 *	Email: vgmcn3@mst.edu
 *	Date: 10-10-2017
 
 *	Description: This file contains libraries used for onboard hardware
 */

#ifndef PWMcontroller_H
#define PWMcontroller_H

#include <Wire.h> // Wire library is used for I2C
#include <SPI.h>  // SPI library is used for...SPI.

#if defined(__AVR__)
 #define WIRE Wire
#endif

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

//PWM controller defines
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD


class PWMcontroller 
{
  public:
	  PWMcontroller(uint8_t addr);
	  void begin(void);
	  void reset(void);
	  void writeMicroseconds(uint8_t channel, uint16_t microseconds);

  private:
	  uint8_t _i2caddr;
	  uint8_t read8(uint8_t addr);
	  void write8(uint8_t addr, uint8_t d);
};

#endif
