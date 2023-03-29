#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor
{
  public:
    Motor(void);
    ~Motor(void);
    void Init(int pwmPin, int in1Pin, int in2Pin, bool inverseDirection);

    // set pwm value of the output (-255 to 255)
    void SetValue(int value);

  private:
    int m_pwmPin;
    int m_in1Pin;
    int m_in2Pin;
    bool m_InverseDirection; 

};



#endif // MOTOR_H