/*
 *	Author: Vinnie Marco
 *	Email: vgmcn3@mst.edu
 *	Date: 10-11-2017
 *
 *	Description: This file pair is used for the current sense pcb
 *			
 */

#ifndef ROV_CS_H
#define ROV_CS_H

#include "Arduino.h"

#define channel0 A0
#define channel1 A1
#define channel2 A2
#define channel3 A3
#define channel4 A4
#define channel5 A5

class ROV_cs
{
  public:
    uint32_t get_current(uint8_t analog_pin);
};

#endif