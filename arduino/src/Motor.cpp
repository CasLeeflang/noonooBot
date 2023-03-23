#include "Motor.h"


Motor::Motor(void)
{

}

Motor::~Motor(void)
{
  
}

void Motor::Init(int pwmPin, int in1Pin, int in2Pin, bool inverseDirection)
{
  m_pwmPin = pwmPin;
  m_in1Pin = in1Pin;
  m_in2Pin = in2Pin;
  m_InverseDirection = inverseDirection;



  // initialise pins
  pinMode(m_pwmPin, OUTPUT);
  pinMode(m_in1Pin, OUTPUT);
  pinMode(m_in2Pin, OUTPUT);

  // set outputs to stop H-bridge control
  analogWrite(m_pwmPin, 0);
  digitalWrite(m_in1Pin, LOW);
  digitalWrite(m_in2Pin, LOW);
}

// set pwm value of the output (-255 to 255)
void Motor::SetValue(int value)
{
  // check if output stays between the limit
  value = (value < -255) ? -255 : value;
  value = (value > 255) ? 255 : value;

  // inverse value if needed
  if(m_InverseDirection)
  {
    value = -value;
  }

  if(value == 0)
  {
    analogWrite(m_pwmPin, 0);
    digitalWrite(m_in1Pin, LOW);
    digitalWrite(m_in2Pin, LOW);
  }
  else if (value > 0)
  {
    analogWrite(m_pwmPin, abs(value));
    digitalWrite(m_in1Pin, HIGH);
    digitalWrite(m_in2Pin, LOW);
  }
  else
  {
    analogWrite(m_pwmPin, abs(value));
    digitalWrite(m_in1Pin, LOW);
    digitalWrite(m_in2Pin, HIGH);
  }
}