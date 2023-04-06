#include <Arduino.h>
#include "Encoder.h"
#include "PID.h"
#include "Motor.h"
#include "Odometry.h"
#include "SerialCommunication.h"

#define encLA_pin 2
#define encLB_pin 3
#define encRA_pin 19
#define encRB_pin 18

#define pwmM_mot 11 
#define mot_in1 6
#define mot_in2 7 

#define pwmM_mot2 9 
#define mot_in1_2 4 
#define mot_in2_2 5  

#define Kp 10
#define Ki 50 
#define Kd 0.00
#define Km 0.005

void encoderInterruptL();
void encoderInterruptR();

SerialCommunication comm;
Motor motL;
Encoder encL;
Motor motR;
Encoder encR;
PID pidL;
PID pidR;

float linearVel = 0.0;        //   [m/s]
float rotationVel = 0.0;      //   [rad/s]
float setpointLeft = 0.0;     //   [rad/s]
float setpointRight = 0.0;    //   [rad/s]

unsigned long previousTimeOdometry;
unsigned long timeLedTest;


void setup() {
  //                        [        tx          ]  [           rx         ]
  comm.SetCommunicationVars(&X_pos, &Y_pos, &theta, &linearVel, &rotationVel);
 
  encL.Init(encLA_pin, encLB_pin, 320, false); 
  encR.Init(encRA_pin, encRB_pin, 320, false); 
  attachInterrupt(digitalPinToInterrupt(encLA_pin), encoderInterruptL, RISING);
  attachInterrupt(digitalPinToInterrupt(encRA_pin), encoderInterruptR, RISING);
  
  motL.Init(pwmM_mot, mot_in1, mot_in2, false);
  motR.Init(pwmM_mot2, mot_in1_2, mot_in2_2, true);

  pidL.SetFactors(Kp, Ki, Kd, Km, 255.0);
  pidR.SetFactors(Kp, Ki, Kd, Km, 255.0);

  // initialize led of life pin  
  pinMode(LED_BUILTIN, OUTPUT);

  timeLedTest = millis();  
  previousTimeOdometry = millis();
}


void loop() {

  // get the current serial commanding velocity's and convert them to setpoints for the left/right motor
  CommandVelocity(linearVel, rotationVel, setpointLeft, setpointRight);

  // Left motor PID controller
  float errorLeft = setpointLeft - encL.GetAngularVelocity();
  pidL.SetInput(errorLeft);
  motL.SetValue(pidL.GetOutput());

  // Right motor PID controller
  float errorRight = setpointRight - encR.GetAngularVelocity();
  pidR.SetInput(errorRight);
  motR.SetValue(pidR.GetOutput());



  // 50 ms interval odometry pose update
  if((millis() - previousTimeOdometry) >= 50)
  {
    EstimatePose(encL.GetAngularVelocity(), encR.GetAngularVelocity());
    previousTimeOdometry = millis();
  } 


  // flash onboard led to show sign of life
  if((millis() - timeLedTest) >= 1000)
  {
    static bool led = false;
    led = !led;
    digitalWrite(LED_BUILTIN, led);
    timeLedTest = millis();
  }





  encL.Update();
  encR.Update();
  pidL.Update();
  pidR.Update();
  comm.Update();
}



void encoderInterruptL()
{
  encL.InterruptTrigger();
} 
void encoderInterruptR()
{
  encR.InterruptTrigger();
} 