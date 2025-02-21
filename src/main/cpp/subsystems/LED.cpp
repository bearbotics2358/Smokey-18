#include <stdio.h> // for using printf
// TODO: can we remove this stdlib.h include statement?
#include <stdlib.h> // atoi
#include "frc/DriverStation.h"
#include <Constants.h>
#include "subsystems/LED.h"

LED::LED() {
  m_pserial = new frc::SerialPort(
    ArduinoConstants::BAUD_RATE_ARDUINO, 
    ArduinoConstants::USB_PORT_ARDUINO,
    ArduinoConstants::DATA_BITS_ARDUINO,
    ArduinoConstants::PARITY_ARDUINO,
    ArduinoConstants::STOP_BITS_ARDUINO
  );
  m_pserial->DisableTermination();
  m_pserial->SetFlowControl(frc::SerialPort::kFlowControl_None);
  m_pserial->SetReadBufferSize(1);  // Read one byte at a time
  m_pserial->SetWriteBufferMode(frc::SerialPort::WriteBufferMode::kFlushOnAccess); // Correct Usage

  memset(m_rxBuff, 0, sizeof(m_rxBuff));
}

void LED::Periodic() { 
  while (m_pserial->GetBytesReceived() > 0) {
    char byte;
    m_pserial->Read(&byte, 1);

    if (byte == '\r' || byte == '\n') {
      if (m_rxIndex > 0) {
        m_rxBuff[m_rxIndex] = 0;  // Null-terminate
        printf("RX: %s\n", m_rxBuff);
        // m_pserial->Flush(); // No need to flush reads

        ProcessReport();
      }
      m_rxIndex = 0;
      memset(m_rxBuff, 0, sizeof(m_rxBuff));  // Clear buffer
    } else {
      if (m_rxIndex < static_cast<int>(sizeof(m_rxBuff)) - 1) {
        m_rxBuff[m_rxIndex++] = byte;
      } else {
        // Buffer overflow!  Handle the error (e.g., reset the index, log a message)
        printf("LED: RX buffer overflow!\n");
        m_rxIndex = 0;
        memset(m_rxBuff, 0, sizeof(m_rxBuff));
      }
    }
  }

  if (m_LEDCurrentCommand != m_LEDPrevCommand) {
    m_LEDPrevCommand = m_LEDCurrentCommand;
    switch (m_LEDCurrentCommand) {
      case ArduinoConstants::RIO_MESSAGES::MSG_IDLE:
        SendIdleMSG();
        break;
      case ArduinoConstants::RIO_MESSAGES::NO_COMMS:
        SendNoCommsMSG();
        break;
      case ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1:
        SendElevatorL1MSG();
        break;
      case ArduinoConstants::RIO_MESSAGES::ALGAE_HELD:
        SendAlgaeHeldMSG();
        break;
      case ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2:
        SendElevatorL2MSG();
        break;
      case ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3:
        SendElevatorL3MSG();
        break;
      case ArduinoConstants::RIO_MESSAGES::IDK:
        SendIDKMSG();
        break;
      case ArduinoConstants::RIO_MESSAGES::TEST:
        SendTestMSG();
        break;
    }
  } 
}

frc2::CommandPtr LED::SetLEDState(ArduinoConstants::RIO_MESSAGES ledState) {
  return RunOnce([this, ledState] {
    m_LEDCurrentCommand = ledState;
  });
}

// TODO: Can we remove this function?
void LED::ProcessReport() {
  // Parse report - no action needed in this example
}

void LED::SendIdleMSG() {
  m_pserial->Write("1,0\r\n", 6);
  //m_pserial->Flush(); //kFlushOnAccess is set.
}

void LED::SendNoCommsMSG() {
  m_pserial->Write("2,0\r\n", 6);
  //m_pserial->Flush(); //kFlushOnAccess is set.
}

void LED::SendElevatorL1MSG() {
  m_pserial->Write("3,0\r\n", 6);
  //m_pserial->Flush(); //kFlushOnAccess is set.
}

void LED::SendAlgaeHeldMSG() {
  m_pserial->Write("4,0\r\n", 6);
  //m_pserial->Flush(); //kFlushOnAccess is set.
}

void LED::SendElevatorL2MSG() {
  m_pserial->Write("5,0\r\n", 6);
  //m_pserial->Flush(); //kFlushOnAccess is set.
}

void LED::SendElevatorL3MSG() {
  m_pserial->Write("6,0\r\n", 6);
  //m_pserial->Flush(); //kFlushOnAccess is set.
}

void LED::SendIDKMSG() {
  m_pserial->Write("7,0\r\n", 6);
  //m_pserial->Flush(); //kFlushOnAccess is set.
}

void LED::SendTestMSG() {
  m_pserial->Write("8,0\r\n", 6);
  //m_pserial->Flush(); //kFlushOnAccess is set.
}
