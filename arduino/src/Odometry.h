#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <math.h>

// constants / variables to describe dimensions of the robot and store the estimated positiion
#define L 0.285
#define r 0.045
#define diffT 0.05

// referencing of the local variables in Odometry.cpp
extern float X_pos;
extern float Y_pos;
extern float theta;


// function to translate robot orientated velocity's towards left/right motor angular velocity's (differential drive)
void CommandVelocity(float w, float v, float &leftMotorSpeed, float &rightMotorSpeed);


// Estimate pose --> function needs to be run periodically (now set to t = 0.05, so every 50 ms)
void EstimatePose(float Wl, float Wr);


#endif // ODOMETRY_H


