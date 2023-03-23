#ifndef ENCODER_H
#define ENCODER_H

#define UPDATE_TIME 50000 // us update time


#include <Arduino.h>

class Encoder
{
  public:
    Encoder(void);
    void Init(int pinEncA, int pinEncB, float pulsesPerRotation);
    float GetRPM(void);
    float GetAngularVelocity(void);

    // continously calling this function
    void Update(void);

    void InterruptTrigger(void);
  private:
    // static function used to have interrupts definitions in the class
    static Encoder* sEncoder;
    static void UpdateEncoderISR(void);
    // interrupt function for class object
    

    int m_encAPin;
    int m_encBPin;
    //float m_pulsesRatio;
    //float m_gearRatio;
    //float m_velocity;
    float m_pulsesPerRation;
    float m_angularVelocity; // rad/s

    //volatile unsigned long m_lastMicros;
    //volatile unsigned long m_lastDuration;
    volatile int m_pulses; 
    unsigned long m_previousTime; 
   // volatile int m_lastRotationDir;

};

#endif //ENCODER_H