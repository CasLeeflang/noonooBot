#include "SerialCommunication.h"

SerialCommunication::SerialCommunication(void)
{

}

void SerialCommunication::SetCommunicationVars(float *tx_posx, float *tx_posy, float *tx_rotz, float *rx_linvel, float *rx_angvel)
{
  m_tx_posx = tx_posx;
  m_tx_posy = tx_posy;
  m_tx_rotz = tx_rotz;

  m_rx_linvel = rx_linvel;
  m_rx_angvel = rx_angvel;

  m_receivedFrameLength = 0;

  m_previoustxTime = millis();
  /*
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
        delay(2000);
        digitalWrite(LED_BUILTIN, LOW); */
}

void SerialCommunication::Update(void)
{
  // tx data of the arduino
  if((millis() - m_previoustxTime) >= TRANSMIT_INTERVAL)
  {
    floatUnion_t sendPosX;
    floatUnion_t sendPosY;
    floatUnion_t sendRotZ;

    sendPosX.number = *m_tx_posx;
    sendPosY.number = *m_tx_posy;
    sendRotZ.number = *m_tx_rotz;


    byte sendBytes[16];
    sendBytes[0] = sendPosX.bytes[0];
    sendBytes[1] = sendPosX.bytes[1];
    sendBytes[2] = sendPosX.bytes[2];
    sendBytes[3] = sendPosX.bytes[3];

    sendBytes[4] = sendPosY.bytes[0];
    sendBytes[5] = sendPosY.bytes[1];
    sendBytes[6] = sendPosY.bytes[2];
    sendBytes[7] = sendPosY.bytes[3];

    sendBytes[8] = sendRotZ.bytes[0];
    sendBytes[9] = sendRotZ.bytes[1];
    sendBytes[10] = sendRotZ.bytes[2];
    sendBytes[11] = sendRotZ.bytes[3];

    sendBytes[12] = '\r';
    sendBytes[13] = '\n';
    
    Serial.write(sendBytes, 14);

    m_previoustxTime = millis();
  }

  // rx of the data
  if (Serial.available())
  {
    m_incomingDataBuff[m_receivedFrameLength] = Serial.read(); 
    m_receivedFrameLength++;

   

    if (m_incomingDataBuff[m_receivedFrameLength-1] == '\n')
    {
      
        
      if(m_receivedFrameLength == 10)
      {
        /*
          digitalWrite(LED_BUILTIN, HIGH);
        delay(2000);
        digitalWrite(LED_BUILTIN, LOW);*/
        floatUnion_t receiveLinVel;
        floatUnion_t receiveAngVel;

        receiveLinVel.bytes[0] = m_incomingDataBuff[0];
        receiveLinVel.bytes[1] = m_incomingDataBuff[1];
        receiveLinVel.bytes[2] = m_incomingDataBuff[2];
        receiveLinVel.bytes[3] = m_incomingDataBuff[3];

        receiveAngVel.bytes[0] = m_incomingDataBuff[4];
        receiveAngVel.bytes[1] = m_incomingDataBuff[5];
        receiveAngVel.bytes[2] = m_incomingDataBuff[6];
        receiveAngVel.bytes[3] = m_incomingDataBuff[7];

        *m_rx_linvel = receiveLinVel.number;
        *m_rx_angvel = receiveAngVel.number;



        
      }
      // set buffer length to 0
      m_receivedFrameLength = 0;
    }
    
  }
  

}