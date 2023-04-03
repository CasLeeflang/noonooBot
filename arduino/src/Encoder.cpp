#include "Encoder.h"

Encoder::Encoder(void)
{
  // clearing the buffer to 0
  for(int i = 0; i < MOVING_AVERAGE_SIZE; i++)
  {
    m_angularVelocity[i] = 0.0;
  }
  m_currentBufIndex = 0;

  m_total_pulses = 0;

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

  // calculate the multiplier for update loop to convert from pulses/x_us to pulses/s
  m_toSecMult = (1000000.0F/(float)UPDATE_TIME);

  // store current us time
  m_previousTime = micros();
}

float Encoder::GetRPM(void)
{
  float sum = 0.0;
  for(int k = 0; k < MOVING_AVERAGE_SIZE; k++)
  {
    sum += m_angularVelocity[k];
  }
  return ((sum / MOVING_AVERAGE_SIZE) / (2 * PI)) * 60;
  
}

float Encoder::GetAngularVelocity(void)
{
  float sum = 0.0;
  for(int k = 0; k < MOVING_AVERAGE_SIZE; k++)
  {
    sum += m_angularVelocity[k];
  }
  return (sum / MOVING_AVERAGE_SIZE);
}

float Encoder::GetTotalAngularDisplacement(void)
{
  return m_angularDisplacement;
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

    // angular displacement calculation
    m_total_pulses += pulses;
    m_angularDisplacement = ((float)m_total_pulses / m_pulsesPerRation) * 2.0 * PI; 



    // angular velocity calculation
//                                        [                                            rad/s                                    ]
//                                         [                                      rotations/s                      ]
// [      rad/s     ]                       [                  pulses/s                      ]    [pulses/rotation]                
    m_angularVelocity[m_currentBufIndex] = (((float)pulses * m_toSecMult) / m_pulsesPerRation) * 4.0F * PI;

    if (m_inverse)
    {
      m_angularVelocity[m_currentBufIndex] = -m_angularVelocity[m_currentBufIndex];
    }
    
    // increment the bufcounter 
    m_currentBufIndex++;
    m_currentBufIndex = (m_currentBufIndex >= (MOVING_AVERAGE_SIZE)) ? (m_currentBufIndex = 0) : (m_currentBufIndex);

    // store the old interval time
    m_previousTime = micros();
  }

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

