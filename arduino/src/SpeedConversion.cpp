#include "SpeedConversion.h"


void CommandVelocity(float w, float v, float &leftMotorSpeed, float &rightMotorSpeed) 
{
  leftMotorSpeed = (v - w * (L / 2)) / r;
  rightMotorSpeed = (v + w * (L / 2)) / r;
}