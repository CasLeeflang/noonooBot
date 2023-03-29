#include "Odometry.h"

// variables are declared here
float X_pos = 0;
float Y_pos = 0;
float theta = 0;
float prevTheta = 0;


void CommandVelocity(float w, float v, float &leftMotorSpeed, float &rightMotorSpeed) 
{
  leftMotorSpeed = (v - w * (L / 2)) / r;
  rightMotorSpeed = (v + w * (L / 2)) / r;
}

void EstimatePose(float Wl, float Wr)
{
  double w = r * ((Wr - Wl) / L);
  //Serial.println(w);
  double X = r * ((Wr + Wl) / 2) * cos(theta) * diffT;
  double Y = r * ((Wr + Wl) / 2) * sin(theta) * diffT;

  theta = prevTheta + (w * diffT);
  if (theta > 2 * M_PI) {
    theta = theta - 2 * M_PI;
  }
  else if (theta < -2 * M_PI) {
    theta = theta + 2 * M_PI;
  }
  prevTheta = theta;
  Y_pos = Y_pos + Y;
  X_pos = X_pos + X;
}