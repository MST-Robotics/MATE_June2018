//Programmer: Reuben French

/*
Motor arrangement:

front    y
1---0    ^
|   |    |
|   |    +--> x
2---3

This function will calculate the thrust required from each of the 4 corner motors
given the desired net force in the x and y directions and the moment (CCW is positive).
Results are returned in "double thrust[4]".

The motors are assumed to be pointed such that positive thrust blows toward the ends of the robot.
*/
void getThrust(double thrust[], const double Fx, const double Fy, const double M)
{
  double tmp[4] = {(Fy+M)/2, (Fx+Fy)/2, (Fx+M)/2, 0};
  if(Fx>0)
  {
    if(Fy>0)
      thrust[3] = tmp[0];
    else
      thrust[3] = tmp[3];
  }
  else
  {
    if(Fy>0)
      thrust[3] = tmp[1];
    else
      thrust[3] = tmp[2];
  }
  
  for(int i = 0; i < 2; i++)
    thrust[i] = thrust[3] - tmp[i];
}

//Not finished!
short thrustToPulse(const double thrust)
{
  //Use data at "http://docs.bluerobotics.com/thrusters/#performance-charts"
  //Remember Deadband
  return 0;
}
