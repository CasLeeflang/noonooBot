#include <Arduino.h>
#include "Encoder.h"
#include "PID.h"
#include "Motor.h"
#include "Odometry.h"
#include "SerialCommunication.h"

#define encA_pin 2
#define encB_pin 3
#define encA2_pin 19
#define encB2_pin 18

#define pwmM_mot 11 // 9
#define mot_in1 6 // 4
#define mot_in2 7 // 5

#define pwmM_mot2 9 // 11
#define mot_in1_2 4 //  6
#define mot_in2_2 5  // 7

#define Kp 15  // 20
#define Ki 80 // 5 //200
#define Kd 0.00
#define Km 0.005

void encoder1Interrupt();
void encoder2Interrupt();

SerialCommunication comm;
Motor motL;
Encoder encL;
Motor motR;
Encoder encR;
PID pidL;
PID pidR;


float dummyvaltx1 = 1.0;
float dummyvaltx2 = 2.0;
float dummyvaltx3 = 3.0;

float linearVel = 0.0;
float rotationVel = 0.0;
float setpointLeft = 0.0; // = 2*PI;  // [rad/s]
float setpointRight = 0.0; // = 2*PI; // [rad/s]

float actualRPM;
float filteredRPM;
float setPWM;


unsigned long previousTime;
unsigned long previousTime2;

unsigned long previousTimeOdometry;
unsigned long timeSerialTest;



// global odemetry variables
// float 

void setup() {


  previousTime = millis();
  previousTimeOdometry = millis();
  
  //Serial.begin(115200);
  comm.SetCommunicationVars(&X_pos, &Y_pos, &theta, &linearVel, &rotationVel);
  //comm.SetCommunicationVars(&dummyvaltx1, &dummyvaltx2, &dummyvaltx3, &linearVel, &rotationVel);

  encL.Init(encA_pin, encB_pin, 320, false); 
  encR.Init(encA2_pin, encB2_pin, 320, false);
  attachInterrupt(digitalPinToInterrupt(encA_pin), encoder1Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2_pin), encoder2Interrupt, RISING);
  
  pidL.SetFactors(Kp, Ki, Kd, Km, 255.0);
  pidR.SetFactors(Kp, Ki, Kd, Km, 255.0);
  motL.Init(pwmM_mot, mot_in1, mot_in2, false);
  motR.Init(pwmM_mot2, mot_in1_2, mot_in2_2, true);

  timeSerialTest = millis();  
  pinMode(LED_BUILTIN, OUTPUT);
}



void loop() {

  // get the current serial commanding velocity's and convert them to setpoints for the left/right motor
  CommandVelocity(linearVel, rotationVel, setpointLeft, setpointRight);
  //CommandVelocity(0.25, 0.0, setpointLeft, setpointRight);
  //setpointLeft = 2.0;
 // setpointRight = 2.0;


  // PID CONTROL var passing
  float errorLeft = setpointLeft - encL.GetAngularVelocity();
  //float errorLeft = 6 - encL.GetAngularVelocity();
  pidL.SetInput(errorLeft);
  motL.SetValue(pidL.GetOutput());
  //motL.SetValue(-50);

  float errorRight = setpointRight - encR.GetAngularVelocity();
  //float errorRight = 6 - encR.GetAngularVelocity();
  pidR.SetInput(errorRight);
  motR.SetValue(pidR.GetOutput());
  //motR.SetValue(-50);


  // 50 ms interval odometry pose update
  if((millis() - previousTimeOdometry) >= 50)
  {
    EstimatePose(encL.GetAngularVelocity(), encR.GetAngularVelocity());
    //EstimatePose(6.0, -6.0);
    
    /*
    Serial.print(encL.GetAngularVelocity());
    Serial.print(",");
    Serial.println(encR.GetAngularVelocity());
    */ 

    /*
    Serial.print(X_pos);
    Serial.print(",");
    Serial.print(Y_pos);
    Serial.print(",");
    Serial.println(theta); 
    previousTimeOdometry = millis(); 
    */
  } 


  if((millis() - timeSerialTest) >= 1000)
  {
    static bool led = false;
    led = !led;
    digitalWrite(LED_BUILTIN, led);
    timeSerialTest = millis();
  }



  encL.Update();
  encR.Update();
  pidL.Update();
  pidR.Update();
  comm.Update();
}



void encoder1Interrupt()
{
  encL.InterruptTrigger();
} 
void encoder2Interrupt()
{
  encR.InterruptTrigger();
} 