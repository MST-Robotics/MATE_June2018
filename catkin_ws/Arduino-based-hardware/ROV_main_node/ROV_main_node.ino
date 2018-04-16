/*
 * Author: Vinnie Marco
 * Email: vgmcn3@mst,edu
 * Date: 4-8-2017
 *
 * Code for basic implementation of rosserial
 *
 * To start the system, run from terminal:
 * Terminal 1:
 *    roscore
 *
 * Terminal 2:
 *    cd ~/MATE_June2018/catkin_ws
 *    source devel/setup.bash
 *    roslaunch ROV_system_v1 rov_system.launch
 *
 * This will start all necessary nodes to read both controllers
 * and stream data to the arduino
 */

#include "ROV_main_node.h"

//Arduino setup
void setup()
{
  main_setup();//contains system declarations and hardware setup
    
  //this is needed
  nh.initNode();

  //put all of the subscriptions here
  //nh.subscribe(trigger_sub);
  //nh.subscribe(button_a_sub);
  //nh.subscribe(button_e_sub);
  //nh.subscribe(rotary_4_sub);

  nh.subscribe(motor1_sub);
  nh.subscribe(motor2_sub);
  nh.subscribe(motor3_sub);
  nh.subscribe(motor4_sub);
  nh.subscribe(motor5_sub);
  nh.subscribe(motor6_sub);

  //nh.subscribe(joystick_rotation_sub);

  nh.advertise(accel_pub);
  nh.advertise(mag_pub);
  nh.advertise(pixy_pub);

}

void loop()
{
  nh.spinOnce();
  process_imu();
  wing_detection_data();
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void process_imu(void)
{
  if(imu.gyroAvailable())
    imu.readGyro();
  if(imu.accelAvailable())
    imu.readAccel();
  if(imu.magAvailable())
    imu.readMag();

  accelerometer.x = imu.calcAccel(imu.ax);
  accelerometer.y = imu.calcAccel(imu.ay);
  accelerometer.z = imu.calcAccel(imu.az);

  magnetometer.x = imu.calcMag(imu.mx);
  magnetometer.y = imu.calcMag(imu.my);
  magnetometer.z = imu.calcMag(imu.mz);

  accel_pub.publish(&accelerometer);
  mag_pub.publish(&magnetometer);
}

void wing_detection_data()
{
  static int frame = 0;
  int j;
  uint16_t blocks;
  char buf[32];

  // grab blocks
  blocks = pixy.getBlocks();

  if (blocks)
  {
    frame++;

    // run every 50 frames
    if (frame%50==0)
    {
      for (j=0; j<blocks; j++)
      {
        pixy_data.x = pixy.blocks[j].signature;
        pixy_data.y = pixy.blocks[j].width;
        pixy_data.z = pixy.blocks[j].height;
        pixy_pub.publish(&pixy_data);
      }
    }
  }
}
