#ifndef ENCODER_H
#define ENCODER_H

#define UPDATE_TIME 50000 // us update time
#define MOVING_AVERAGE_SIZE 1


#include <Arduino.h>

class Encoder
{
  public:
    Encoder(void);
    void Init(int pinEncA, int pinEncB, float pulsesPerRotation, bool inverse);
    float GetRPM(void);
    float GetAngularVelocity(void);
    float GetTotalAngularDisplacement(void);

    // continously calling this function
    void Update(void);

    // interrupt function for class object
    void InterruptTrigger(void);
  private:
    int m_encAPin;
    int m_encBPin;
    bool m_inverse;
    float m_pulsesPerRation;

    float m_toSecMult; 
    float m_angularVelocity[MOVING_AVERAGE_SIZE]; // rad/s
    float m_angularDisplacement; // rad
    int m_currentBufIndex;
    long m_total_pulses;

  
    volatile int m_pulses; 
    unsigned long m_previousTime; 

};

#endif //ENCODER_H