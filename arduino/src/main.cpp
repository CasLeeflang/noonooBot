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

#define pwmM_mot 9
#define mot_in1 4
#define mot_in2 5

#define pwmM_mot2 11
#define mot_in1_2 6
#define mot_in2_2 7

#define Kp 20  // 20
#define Ki 200 // 5
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
  // set motor to full spin
  //pinMode(pwmM_mot, OUTPUT);
  //pinMode(mot_in1, OUTPUT);
  //pinMode(mot_in2, OUTPUT);

  previousTime = millis();
  previousTimeOdometry = millis();
  // = millis();
  
  //Serial.begin(115200);
  comm.SetCommunicationVars(&X_pos, &Y_pos, &theta, &linearVel, &rotationVel);
  //comm.SetCommunicationVars(&dummyvaltx1, &dummyvaltx2, &dummyvaltx3, &linearVel, &rotationVel);

  

  //pinMode(encA_pin, INPUT_PULLUP);
 // attachInterrupt(digitalPinToInterrupt(encA_pin), encoderInterruptA, RISING);
  encL.Init(encA_pin, encB_pin, 320, false); //16
  encR.Init(encA2_pin, encB2_pin, 320, false);
  attachInterrupt(digitalPinToInterrupt(encA_pin), encoder1Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2_pin), encoder2Interrupt, RISING);
  
  pidL.SetFactors(Kp, Ki, Kd, Km, 255.0);
  pidR.SetFactors(Kp, Ki, Kd, Km, 255.0);
  motL.Init(pwmM_mot, mot_in1, mot_in2, false);
  motR.Init(pwmM_mot2, mot_in1_2, mot_in2_2, true);
  //motL.SetValue(35);
  //tone(32, 240);
  //tone(40, 320);
  // put your setup code here, to run once:
  //setMotorPWM(-32);
  //CommandVelocity(1.0, 0.0, setpointLeft, setpointRight);
  timeSerialTest = millis();
  pinMode(LED_BUILTIN, OUTPUT);
}



void loop() {

  // get the current serial commanding velocity's and convert them to setpoints for the left/right motor
  CommandVelocity(linearVel, rotationVel, setpointLeft, setpointRight);
  //CommandVelocity(4.0, 0.0, setpointLeft, setpointRight);
  //setpointLeft = 0.0;
  //setpointRight = 2.0;


  // PID CONTROL var passing
  float errorLeft = setpointLeft - encL.GetAngularVelocity();
  pidL.SetInput(errorLeft);
  motL.SetValue(pidL.GetOutput());

  float errorRight = setpointRight - encR.GetAngularVelocity();
  pidR.SetInput(errorRight);
  motR.SetValue(pidR.GetOutput());

  // 50 ms interval odometry pose update
  if((millis() - previousTimeOdometry) >= 50)
  {
    EstimatePose(encL.GetAngularVelocity(), encR.GetAngularVelocity());
    //EstimatePose(6.0, -6.0);
    
    /*
    Serial.print(encL.GetAngularVelocity());
    Serial.print(",");
    Serial.println(encR.GetAngularVelocity()); */

    /*
    Serial.print(X_pos);
    Serial.print(",");
    Serial.print(Y_pos);
    Serial.print(",");
    Serial.println(theta); 
    previousTimeOdometry = millis(); */
  }

  if((millis() - timeSerialTest) >= 1000)
  {
    static bool led = false;
    led = !led;
    digitalWrite(LED_BUILTIN, led);
    timeSerialTest = millis();
  }



  /*
  // print angular velocity every 100 ms to the serial connection
  if((millis() - previousTime) > 100)
  {
    Serial.print(encL.GetAngularVelocity());
    Serial.print(",");
    Serial.println(encR.GetAngularVelocity());
    previousTime = millis();

  } */




  /*

  if(// interval - 50ms)
  {
    float speed1 = encL.speed;
    float speed2 = encL.speed;

    // odemetry(speed1, speed2);
    

  } */

/*
  //actualRPM = filter(encL.GetRPM());

    // output rpm every 1 ms
    delsu
  if((millis() - previousTime2) >= 10)
  {
    actualRPM = encL.GetRPM(); //filter();
    //filteredRPM = filter(actualRPM);
   previousTime2 = millis(); */

/*
    float error = setpoint - actualRPM;
   // Serial.print("errror = ");
    //Serial.println(error, 6); 

    setPWM = pidL.Update(error);
    //Serial.print("pwm = ");
    //Serial.println(pwmVal, 6); 
    setMotorPWM(setPWM);
    previousTime2 = millis();
  
  } 
  */

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