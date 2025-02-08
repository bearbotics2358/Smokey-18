#include <stdio.h> // printf
#include <stdlib.h> // atoi
#include <Constants.h>
#include <string.h>
#include <commands/ChangeLEDs.h>
#include "subsystems/LED.h"

LED::LED() {
  m_pserial = new frc::SerialPort(
      ArduinoConstants::BAUD_RATE_ARDUINO, ArduinoConstants::USB_PORT_ARDUINO,
      ArduinoConstants::DATA_BITS_ARDUINO, ArduinoConstants::PARITY_ARDUINO,
      ArduinoConstants::STOP_BITS_ARDUINO);
  m_pserial->DisableTermination();
  m_pserial->SetFlowControl(frc::SerialPort::kFlowControl_None);
  m_pserial->SetReadBufferSize(1);  // Read one byte at a time
  m_pserial->SetWriteBufferMode(frc::SerialPort::WriteBufferMode::kFlushOnAccess); // Correct Usage

  rx_index = 0;
  memset(rx_buff, 0, sizeof(rx_buff));
}

void LED::Periodic() { Update(); }

void LED::SetLEDState(ArduinoConstants::RIO_MESSAGES ledState) {
  LED_currentCommand = ledState;
}

void LED::Update() {
  while (m_pserial->GetBytesReceived() > 0) {
    char byte;
    m_pserial->Read(&byte, 1);

    if (byte == '\r' || byte == '\n') {
      if (rx_index > 0) {
        rx_buff[rx_index] = 0;  // Null-terminate
        printf("RX: %s\n", rx_buff);
        // m_pserial->Flush(); // No need to flush reads

        ProcessReport();
      }
      rx_index = 0;
      memset(rx_buff, 0, sizeof(rx_buff));  // Clear buffer
    } else {
      if (rx_index < static_cast<int>(sizeof(rx_buff)) - 1) {
        rx_buff[rx_index++] = byte;
      } else {
        // Buffer overflow!  Handle the error (e.g., reset the index, log a message)
        printf("LED: RX buffer overflow!\n");
        rx_index = 0;
        memset(rx_buff, 0, sizeof(rx_buff));
      }
    }
  }

  if (LED_currentCommand != LED_prevCommand) {
    LED_prevCommand = LED_currentCommand;
    switch (LED_currentCommand) {
      case ArduinoConstants::RIO_MESSAGES::WHITE:
        SendWhiteMSG();
        break;
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
      default:
        break;
    }
  }
}

void LED::ProcessReport() {
  // Parse report - no action needed in this example
}

void LED::SendWhiteMSG() {
  m_pserial->Write("0,0\r\n", 6); // Use literal, shorter length
  //m_pserial->Flush(); //kFlushOnAccess is set.
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
