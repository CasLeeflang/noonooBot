#include "Odometry.h"

// variables are declared here
float X_pos = 0;
float Y_pos = 0;
float theta = 0;
float prevTheta = 0;
float omegaThreshold = 0.1 * M_PI;


void CommandVelocity(float v, float w, float &leftMotorSpeed, float &rightMotorSpeed) 
{
  leftMotorSpeed = (v - w * (L / 2)) / r;
  rightMotorSpeed = (v + w * (L / 2)) / r;
}

void EstimatePose(float Wl, float Wr)
{
  double w = r * ((Wr - Wl) / L);
  double X = r * ((Wr + Wl) / 2) * cos(theta) * diffT;
  double Y = r * ((Wr + Wl) / 2) * sin(theta) * diffT;

  // Speed difference thresholding
  // set omega to 0.0 when speed difference is in the threshold region 
  ((w <= omegaThreshold) && (w >= -omegaThreshold)) ? w = 0 : w = w;
  theta = prevTheta + (w * diffT);

 

  // limit outtput from 0 to 2 * pi
  if (theta > 2 * M_PI) 
  {
     theta = theta - 2 * M_PI;
   }
   else if (theta < 0.0) 
   { 
     theta = theta + 2 * M_PI;
   }

  prevTheta = theta;
  Y_pos = Y_pos + Y;
  X_pos = X_pos + X;
}