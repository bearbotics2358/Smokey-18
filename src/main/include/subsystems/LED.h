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

  void Periodic() override; // Declare Update

  frc2::CommandPtr SetLEDState(ArduinoConstants::RIO_MESSAGES ledState);


  void SendIdleMSG();
  void SendNoCommsMSG();
  void SendElevatorL1MSG();
  void SendAlgaeHeldMSG();
  void SendElevatorL2MSG();
  void SendElevatorL3MSG();
  void SendIDKMSG();
  void SendTestMSG();

 private:
  void ProcessReport();  // Declare ProcessReport

  
  frc::SerialPort* m_pserial;
  char m_rxBuff[32];  // Smaller buffer
  int m_rxIndex = 0;

  ArduinoConstants::RIO_MESSAGES m_LEDPrevCommand = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;
  ArduinoConstants::RIO_MESSAGES m_LEDCurrentCommand = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;


};

#endif  // LED_H
