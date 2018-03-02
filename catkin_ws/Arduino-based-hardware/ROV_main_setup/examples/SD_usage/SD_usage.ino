/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst.edu
 * 
 * Date: 10-10-2017
 * 
 * Description: Basic usage for onboard 
 *      SD Card
 */

#include <ROV_main_setup.h>

void setup() 
{
  Serial.begin(115200);//need to set up serial for this example 
  
  bool error_flag = main_setup();//check for an error
  if(error_flag)
  {
    //Serial.println("ERROR");//prints error message if serial is started
    while(1);
  }
  
}

void loop() 
{
  // make a string for assembling the data to log:
  String dataString = "";

  // read three sensors and append to the string:
  for (int analogPin = 0; analogPin < 3; analogPin++) 
  {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) 
      dataString += ",";
  }

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) 
  {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else 
    Serial.println("error opening datalog.txt");
}
