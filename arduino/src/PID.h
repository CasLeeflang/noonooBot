#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID
{
  public:
    PID(void);
    void SetFactors(float p, float i, float d, float SamplingTime, float limit);
    void SetInput(float error);
    float GetOutput(void);


    void Update(void);

  private:
    unsigned long m_previousTime;
    unsigned long m_interval;
    float m_gainP;
    float m_gainI;
    float m_gainD;
    float m_limit;
    float m_ts;
    struct s_controllerData
    {
      float currentError;
      float previousError;
      float previousError2;
      float previousIntegral;
      float currentOutput;
      float previousOutput;
    } m_controllerData;
    
};

#endif //PID_H