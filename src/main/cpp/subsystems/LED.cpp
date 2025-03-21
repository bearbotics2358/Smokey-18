#include <stdio.h> // for using printf
#include <stdlib.h> // atoi
#include "frc/DriverStation.h"
#include "subsystems/LED.h"

LED::LED() {
  m_pserial = new frc::SerialPort(
    ArduinoConstants::BAUD_RATE_ARDUINO,
    ArduinoConstants::USB_PORT_ARDUINO,
    ArduinoConstants::DATA_BITS_ARDUINO,
    ArduinoConstants::PARITY_ARDUINO,
    ArduinoConstants::STOP_BITS_ARDUINO
  );

  try {
    m_pserial->DisableTermination();
    m_pserial->SetFlowControl(frc::SerialPort::kFlowControl_None);
    m_pserial->SetReadBufferSize(1);  // Read one byte at a time
    m_pserial->SetWriteBufferMode(frc::SerialPort::WriteBufferMode::kFlushOnAccess);
  } catch (std::exception& e) {
    std::printf("Error configuring serial port: %s\n", e.what());
  }

  memset(m_rxBuff, 0, sizeof(m_rxBuff));

  SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);
}

LED::~LED() {
    delete m_pserial;
}

void LED::Periodic() {
  try {
    while (m_pserial->GetBytesReceived() > 0) {
      char byte;
      m_pserial->Read(&byte, 1);

      if (byte == '\r' || byte == '\n') {
        if (m_rxIndex > 0) {
          m_rxBuff[m_rxIndex] = 0;  // Null-terminate
          printf("RX: %s\n", m_rxBuff);
          ProcessReport();
        }
        m_rxIndex = 0;
        memset(m_rxBuff, 0, sizeof(m_rxBuff));  // Clear buffer
      } else {
        if (m_rxIndex < static_cast<int>(sizeof(m_rxBuff)) - 1) {
          m_rxBuff[m_rxIndex++] = byte;
        } else {
          printf("LED: RX buffer overflow!\n");
          m_rxIndex = 0;
          memset(m_rxBuff, 0, sizeof(m_rxBuff));
        }
      }
    }
  } catch (std::exception& e) {
    std::printf("Error reading from serial port: %s\n", e.what());
  }

  if (m_LEDCurrentCommand != m_LEDPrevCommand) {
    m_LEDPrevCommand = m_LEDCurrentCommand;
    const char* message = "";

    switch (m_LEDCurrentCommand) {
      case ArduinoConstants::RIO_MESSAGES::MSG_IDLE:            message = "1\r\n"; break;
      case ArduinoConstants::RIO_MESSAGES::NO_COMMS:            message = "2\r\n"; break;
      case ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1:         message = "3\r\n"; break;
      case ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3_ALGAE:   message = "4\r\n"; break;
      case ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2:         message = "5\r\n"; break;
      case ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3:         message = "6\r\n"; break;
      case ArduinoConstants::RIO_MESSAGES::IDK:                 message = "7\r\n"; break;
      case ArduinoConstants::RIO_MESSAGES::TEST:                message = "8\r\n"; break;
      default:
        std::printf("Unknown LED command: %d\n", static_cast<int>(m_LEDCurrentCommand));
        break;
    }
    if (message[0] != '\0') {
      SendMSG(message);
    }
  }
  if (!frc::DriverStation::IsDSAttached()) {
    SetLEDState(ArduinoConstants::RIO_MESSAGES::NO_COMMS);
  }
  else{
    if(!wasDSAttached){
      SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);
      wasDSAttached = true;
    }
  }
}

void LED::SetLEDState(ArduinoConstants::RIO_MESSAGES ledState) {
  m_LEDCurrentCommand = ledState;
}

void LED::SendMSG(const char* message) {
   try {
    m_pserial->Write(message, strlen(message));
    printf("Wrote message: %s\n", message); // Debugging
   } catch (const std::exception& e) {
    std::printf("Error writing to serial port: %s\n", e.what());
  }
}

void LED::ProcessReport() {
  // Parse report - no action needed in this example
  // Kept as it was in the original file
}
