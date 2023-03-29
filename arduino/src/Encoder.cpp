#include "Encoder.h"

// default address value to check if object is existing 
Encoder* Encoder::sEncoder = 0;

Encoder::Encoder(void)
{
  sEncoder = this;
  /*
  m_lastMicros = 0;
  m_lastDuration = 0;
  m_lastRotationDir = 0; */
}

void Encoder::Init(int pinEncA, int pinEncB, float pulsesPerRotation, bool inverse)
{
  m_encAPin = pinEncA;
  m_encBPin = pinEncB;
  m_pulsesPerRation = pulsesPerRotation;
  m_inverse = inverse;

  // set pin direction 
  pinMode(m_encAPin, INPUT_PULLUP);
  pinMode(m_encBPin, INPUT_PULLUP);

  //attachInterrupt(digitalPinToInterrupt(m_encAPin), Encoder::UpdateEncoderISR, RISING);


  // store current micro time
  m_previousTime = micros();
}

float Encoder::GetAngularVelocity(void)
{
  return m_angularVelocity;
}

void Encoder::Update(void)
{
  unsigned long currentTime = micros();
  if((currentTime - m_previousTime) >= UPDATE_TIME)
  {
    int pulses;
    // disable interrupts and get the current amount of pulses
    uint8_t oldSREG = SREG;
    cli();
    pulses = m_pulses;
    m_pulses = 0;
    SREG = oldSREG;
//                      [                                      rad/s                                ]
//                       [                               rotations/s                      ]
// [      rad/s     ]     [                  pulses/s                ]    [pulses/rotation]                
    m_angularVelocity = ((pulses * ((float)1000000/(float)UPDATE_TIME)) / m_pulsesPerRation) * 2 * PI;

    if (m_inverse)
    {
      m_angularVelocity = -m_angularVelocity;
    }
    
    // store the old interval time
    m_previousTime = currentTime;
  }

}


float Encoder::GetRPM(void)
{
  return (m_angularVelocity / (2 * PI)) * 60;
  /*
  volatile unsigned long pulses;
  volatile unsigned long currentTime;
  volatile unsigned long duration;

  // disable interrupts and get the current amount of pulses
  uint8_t oldSREG = SREG;
  cli();
  pulses = m_pulses;
  m_pulses = 0;
  SREG = oldSREG;

  // get the current time and calculate the delta time
  currentTime = micros();
  duration = currentTime - m_previousTime; // us
  */
 


 // volatile float duration_s = (float)duration / 1000000.0;
  
  
  
  /*
  Serial.print("duration = ");
  Serial.println(duration);
  Serial.println(duration_s, 6);
  Serial.print("pulses = ");
  Serial.println(pulses); */

  //float rotationalVelocity = duration_s*pulses*(60.0/312.0);
  //float rotationalVelocity = ((float)pulses / duration_s) * m_pulsesRatio * 60.0; 
  //m_previousTime = currentTime;


/*
  Serial.print("rpm = ");
  Serial.println(rotationalVelocity);*/


  // assign direction to the velocity
 // rotationalVelocity = (m_lastRotationDir) ? (rotationalVelocity * -1.0) : (rotationalVelocity);

  
  //m_previousTime = 0;

  /*
  float duration_s = (float)m_lastDuration / 1000000;
  float rotationalVelocity = (m_pulsesRatio / duration_s) * 60.0; 
  */

  // assign direction to the velocity
  /*
  if(m_lastRotationDir == 0)
  {
    rotationalVelocity *= -1.0;
  }*/

   //return rotationalVelocity; //* m_gearRatio;

   /*

  if(isinf(rotationalVelocity))
  {
    return 0.0;
  }
  else
  {
   
  } 
  */
  
}

// corresponding interrupt trigger for the specified object
void Encoder::InterruptTrigger(void)
{
  int b = digitalRead(m_encBPin);
  if (b > 0) {
    m_pulses++;
  } else {
    m_pulses--;
  }
}


/*
// static interrupt handler for all incoming encoder object interrupts
void Encoder::UpdateEncoderISR(void)
{
  
  if(sEncoder != 0)
  {
    sEncoder->InterruptTrigger();
  }
} */