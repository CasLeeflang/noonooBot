#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <Arduino.h>

#define TRANSMIT_INTERVAL 100 // tx update ms update time

class SerialCommunication
{
  public:
    SerialCommunication(void);

    void SetCommunicationVars(float *tx_posx, float *tx_posy, float *tx_rotz, float *rx_linvel, float *rx_angvel);
    void Update(void);

  private:
    typedef union
    {
      float number;
      uint8_t bytes[4];
    } floatUnion_t;

    // tx vars
    unsigned long m_previoustxTime;
    float *m_tx_posx;
    float *m_tx_posy;
    float *m_tx_rotz;

    // rx vars
    float *m_rx_linvel;
    float *m_rx_angvel;
    int m_receivedFrameLength;
    byte m_incomingDataBuff[50];

};





#endif // SERIAL_COMM_H