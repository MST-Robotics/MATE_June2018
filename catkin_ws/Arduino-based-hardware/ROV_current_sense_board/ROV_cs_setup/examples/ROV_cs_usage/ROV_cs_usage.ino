/*
 *  Author: Vinnie Marco
 *  Email: vgmcn3@mst.edu
 *  Date: 10-11-2017
 *
 *  Description: This demonstrates the usage of the ROV_cs board
 *      
 */

#include <ROV_cs.h>

ROV_cs cs_board;

uint8_t channels[6] = {channel0, channel1, channel2,
          channel3, channel4, channel5};
          
void setup() 
{
  Serial.begin(115200);//serial is needed for this example
}

void loop() 
{
  //print each current sensor value
  for(uint8_t i = 0; i < 6; i++)
  {
    uint32_t current_val = cs_board.get_current(channels[i]);//get the channel's current

    //Do some formatting
    Serial.print(F("Channel"));
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(current_val);
    Serial.println(F(" mA"));
  }
  Serial.println();
}
