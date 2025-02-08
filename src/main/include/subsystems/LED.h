// LED.h - Control LED lights representing desired game piece
#ifndef LED_H
#define LED_H

#include "Constants.h"
#include <frc/SerialPort.h>

#define BUFF_SIZE 256

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>

#include <functional>
#include <array>

class LED : public frc2::SubsystemBase {
 public:
  LED();
  ~LED() override = default;  // Use override keyword

  void Periodic() override;

  void SetLEDState(ArduinoConstants::RIO_MESSAGES ledState);

 private:
  frc::SerialPort* m_pserial;
  char rx_buff[32];  // Smaller buffer
  int rx_index = 0;

  ArduinoConstants::RIO_MESSAGES LED_prevCommand = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;
  ArduinoConstants::RIO_MESSAGES LED_currentCommand = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;

  void SendWhiteMSG();
  void SendIdleMSG();
  void SendNoCommsMSG();
  void SendElevatorL1MSG();
  void SendAlgaeHeldMSG();
  void SendElevatorL2MSG();
  void SendElevatorL3MSG();
  void SendIDKMSG();
  void SendTestMSG();
};

#endif  // LED_H
