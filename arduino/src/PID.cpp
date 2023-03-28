#include "PID.h"

PID::PID()
{
  m_controllerData.currentError = 0;
  m_controllerData.previousError = 0;
  m_controllerData.previousIntegral = 0;
}

void PID::SetFactors(float p, float i, float d, float SamplingTime, float limit)
{
  m_gainP = p;
  m_gainI = i;
  m_gainD = d;
  m_ts = SamplingTime;
  m_interval = (unsigned long)(m_ts * 1000.0);
  m_limit = abs(limit);
}

void PID::SetInput(float error)
{
  m_controllerData.currentError = error;
}

float PID::GetOutput(void)
{
  return m_controllerData.currentOutput;
}


void PID::Update(void)
{
  unsigned long currentTime = millis();
  if((currentTime - m_previousTime) >= m_interval)
  {
    // P-factor
    m_controllerData.currentOutput = m_controllerData.previousOutput + (m_gainP + m_gainD/m_ts) * m_controllerData.currentError;
    // I-factor
    m_controllerData.currentOutput = m_controllerData.currentOutput + (-m_gainP + m_gainI*m_ts -2*m_gainD/m_ts) * m_controllerData.previousError;
    // D-factor
    m_controllerData.currentOutput = m_controllerData.currentOutput + (m_gainD/m_ts) * m_controllerData.previousError2;
  
    // previous data storing
    m_controllerData.previousError2 = m_controllerData.previousError; 
    m_controllerData.previousError = m_controllerData.currentError;
    
    m_controllerData.previousOutput = m_controllerData.currentOutput;

    m_previousTime = currentTime;

  //cv = cv1 + (kp + kd/tm)*error + (-kp + ki*tm -2*kd/tm)*error1 +(kd/tm)*error2;
  //cv = error * kp; 
  //cv1 = cv;
  //error2 = error1;
  //error1 = error;


    //m_previousTime = currentTime;
    /*
    float p = m_controllerData.currentError * m_gainP;
    
    float i = ((m_gainI * m_ts) / 2.0) * (m_controllerData.currentError + m_controllerData.previousError);
    i = i + m_controllerData.previousIntegral;
    
    float limitIntMax, limitIntMin;
    if(p > 0.0)
    {
      // positive number
      limitIntMax = ((m_limit - p) < 0.0) ? (0.0) : (m_limit - p);
      limitIntMin = -m_limit;
    }
    else
    {
      // negative number
      limitIntMax = m_limit;
      limitIntMin = ((-m_limit - p) > 0.0) ? (0.0) : (-m_limit - p);
    }
    i = (i > limitIntMax) ? (limitIntMax) : (i);
    i = (i < limitIntMin) ? (limitIntMin) : (i);


    // store the current control data in the old one
    m_controllerData.previousIntegral = i;
    m_controllerData.previousError = m_controllerData.currentError;

    // make sure output stays in defined limit
    float output = p + i;
    output = (output < -m_limit) ? (-m_limit) : (output); 
    output = (output > m_limit) ? (m_limit) : (output); 

    m_controllerData.currentOutput = output;
    */

   
  }
}