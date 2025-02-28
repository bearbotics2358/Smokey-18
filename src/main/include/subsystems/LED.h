#pragma once

// LED.h - Control LED lights representing desired game piece
#ifndef LED_H 
#define LED_H

#include "Constants.h"
#include <frc/SerialPort.h>

#define BUFF_SIZE 256

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <functional>
#include <array>

class LED : public frc2::SubsystemBase {
 public:
  LED();
  ~LED() override = default;  // Use override keyword

  void Periodic() override;

 void SetLEDState(ArduinoConstants::RIO_MESSAGES ledState);

 private:
  void SendMSG(const char* message);
  void ProcessReport();  // Kept this as it was in the original

  frc::SerialPort* m_pserial;
  char m_rxBuff[32];
  int m_rxIndex = 0;

  ArduinoConstants::RIO_MESSAGES m_LEDPrevCommand = ArduinoConstants::RIO_MESSAGES::NO_COMMS;
  ArduinoConstants::RIO_MESSAGES m_LEDCurrentCommand = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;
};

#endif  // LED_H
