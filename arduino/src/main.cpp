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

#define Kp 20
#define Ki 5.0
#define Kd 0.1
#define Km 0.005

void encoder1Interrupt();
void encoder2Interrupt();

SerialCommunication comm;
Motor mot1;
Encoder enc1;
Motor mot2;
Encoder enc2;
PID pid;
PID pid2;


float dummyvalrx1 = -10.0;
float dummyvalrx2 = 0.0;

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
  
  comm.SetCommunicationVars(&X_pos, &Y_pos, &theta, &linearVel, &rotationVel);
  //comm.SetCommunicationVars(&dummyvalrx1, &dummyvalrx1, &dummyvalrx2, &dummyvalrx1, &dummyvalrx2);

  

  //pinMode(encA_pin, INPUT_PULLUP);
 // attachInterrupt(digitalPinToInterrupt(encA_pin), encoderInterruptA, RISING);
  enc1.Init(encA_pin, encB_pin, 320, false); //16
  enc2.Init(encA2_pin, encB2_pin, 320, false);
  attachInterrupt(digitalPinToInterrupt(encA_pin), encoder1Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2_pin), encoder2Interrupt, RISING);
  
  pid.SetFactors(Kp, Ki, Kd, Km, 255.0);
  pid2.SetFactors(Kp, Ki, Kd, Km, 255.0);
  mot1.Init(pwmM_mot, mot_in1, mot_in2, false);
  mot2.Init(pwmM_mot2, mot_in1_2, mot_in2_2, true);
  //mot1.SetValue(35);
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



  // PID CONTROL var passing
  float errorLeft = setpointLeft - enc1.GetAngularVelocity();
  pid.SetInput(errorLeft);
  mot1.SetValue(pid.GetOutput());

  float errorRight = setpointRight - enc2.GetAngularVelocity();
  pid2.SetInput(errorRight);
  mot2.SetValue(pid2.GetOutput());

  // 50 ms interval odometry pose update
  if((millis() - previousTimeOdometry) >= 50)
  {
    EstimatePose(enc1.GetAngularVelocity(), enc2.GetAngularVelocity());
    previousTimeOdometry = millis();
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
    Serial.print(enc1.GetAngularVelocity());
    Serial.print(",");
    Serial.println(enc2.GetAngularVelocity());
    previousTime = millis();

  } */




  /*

  if(// interval - 50ms)
  {
    float speed1 = enc1.speed;
    float speed2 = enc1.speed;

    // odemetry(speed1, speed2);
    

  } */

/*
  //actualRPM = filter(enc1.GetRPM());

    // output rpm every 1 ms
    delsu
  if((millis() - previousTime2) >= 10)
  {
    actualRPM = enc1.GetRPM(); //filter();
    //filteredRPM = filter(actualRPM);
   previousTime2 = millis(); */

/*
    float error = setpoint - actualRPM;
   // Serial.print("errror = ");
    //Serial.println(error, 6); 

    setPWM = pid.Update(error);
    //Serial.print("pwm = ");
    //Serial.println(pwmVal, 6); 
    setMotorPWM(setPWM);
    previousTime2 = millis();
  
  } 
  */

  enc1.Update();
  enc2.Update();
  pid.Update();
  pid2.Update();
  comm.Update();
}



void encoder1Interrupt()
{
  enc1.InterruptTrigger();
} 
void encoder2Interrupt()
{
  enc2.InterruptTrigger();
} 