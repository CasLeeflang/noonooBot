#include <Arduino.h>
#include "Encoder.h"
#include "PID.h"
#include "Motor.h"

#define encA_pin 2
#define encB_pin 3
#define encA2_pin 20
#define encB2_pin 21

#define pwmM_mot 9
#define mot_in1 4
#define mot_in2 5

#define Kp 20
#define Ki 5.0
#define Kd 0.1
#define Km 0.01

void encoder1Interrupt();
void encoder2Interrupt();

Motor mot1;
Encoder enc1;
Motor mot2;
Encoder enc2;
PID pid;
float setpoint = 10.0;
float actualRPM;
float filteredRPM;
float setPWM;


unsigned long previousTime;
unsigned long previousTime2;

void setup() {
  // set motor to full spin
  //pinMode(pwmM_mot, OUTPUT);
  //pinMode(mot_in1, OUTPUT);
  //pinMode(mot_in2, OUTPUT);

  previousTime = millis();
  // = millis();
  Serial.begin(9600);

  

  //pinMode(encA_pin, INPUT_PULLUP);
 // attachInterrupt(digitalPinToInterrupt(encA_pin), encoderInterruptA, RISING);
  enc1.Init(encA_pin, encB_pin, 320); //16
  enc2.Init(encA2_pin, encB2_pin, 320);
  attachInterrupt(digitalPinToInterrupt(encA_pin), encoder1Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2_pin), encoder2Interrupt, RISING);
  
  pid.SetFactors(Kp, Ki, Kd, Km, 255.0);
  mot1.Init(pwmM_mot, mot_in1, mot_in2, false);
  //mot1.SetValue(35);
  //tone(32, 240);
  //tone(40, 320);
  // put your setup code here, to run once:
  //setMotorPWM(-32);
}



void loop() {

  float error = setpoint - enc1.GetAngularVelocity();
  pid.SetInput(error);
  mot1.SetValue(pid.GetOutput());

  // print angular velocity every 100 ms to the serial connection
  if((millis() - previousTime) > 100)
  {
    Serial.print(enc1.GetAngularVelocity());
    Serial.print(",");
    Serial.println(enc2.GetAngularVelocity());
    previousTime = millis();

  }

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
}



void encoder1Interrupt()
{
  enc1.InterruptTrigger();
} 
void encoder2Interrupt()
{
  enc2.InterruptTrigger();
} 