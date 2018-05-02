/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst,edu
 * Date: 6-19-2017
 *
 *
 */

#include <ROV_main_setup.h>
#include <Servo.h>

//motor objects
Servo front_right;
Servo front_left;
Servo back_right;
Servo back_left;
Servo middle_left;
Servo middle_right;


void setup() 
{
  //these are the pins that the motors are connected to in order
  front_left.attach(4);//front left
  middle_left.attach(5);//middle left
  back_left.attach(6);//back left
  front_right.attach(9);//front right
  middle_right.attach(10);//middle right
  back_right.attach(12);//back right

  //start with the motors off
  front_right.writeMicroseconds(1500);
  front_left.writeMicroseconds(1500);
  middle_right.writeMicroseconds(1500);
  middle_left.writeMicroseconds(1500);
  back_right.writeMicroseconds(1500);
  back_left.writeMicroseconds(1500);   

}

void loop() 
{
  //cycle each motor at a time
  cycle_motor(front_left);
  cycle_motor(middle_left);
  cycle_motor(back_left);
  cycle_motor(front_right);
  cycle_motor(middle_right);
  cycle_motor(back_right);
}

void cycle_motor(Servo &servo)
{
  //cycle motor from stop, to max speed one direction
  for(int i = 1500; i < 1900; i+=5)
  {
    servo.writeMicroseconds(i);
    delay(10);
  }

  delay(1000);

  //cycle motor from max speed down to stop
  for(int i = 1900; i >= 1500; i-=5)
  {
    servo.writeMicroseconds(i);
    delay(10);
  }

  delay(1000);

  //cycle motor from stop, to max speed other direction
  for(int i = 1500; i >= 1100; i-=5)
  {
    servo.writeMicroseconds(i);
    delay(10);
  }

  delay(1000);

  //cycle motor from max speed down to stop
  for(int i = 1100; i < 1500; i+=5)
  {
    servo.writeMicroseconds(i);
    delay(10);
  }

  delay(1000);

  return;
}

