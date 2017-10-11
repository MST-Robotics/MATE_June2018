/*
 *	Author: Vinnie Marco
 *	Email: vgmcn3@mst.edu
 *	Date: 10-11-2017
 *
 *	Description: This file pair is used for the current sense pcb
 *			
 */
#include "ROV_cs.h"
#include "Arduino.h"

uint32_t ROV_cs::get_current(uint8_t analog_pin)
{ 
  uint16_t sensorValue = analogRead(analog_pin);//read the raw analog value
  
  return (((long)sensorValue * 5000/ 1024) - 2500) * 1000 / 66;//calculate the current in (mA)
} 