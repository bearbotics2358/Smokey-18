/* 
 *LED.cpp - Control LED lights representing desired game pieces
 */

#include <stdio.h> // printf
#include <stdlib.h> // atoi
#include <Constants.h>
#include <string.h>

#include "subsystems/LED.h"


LED::LED() {
    Init();
    m_LEDArray = {
        std::bind(&LED::SetWhite, this),
        std::bind(&LED::SetMSGIdle, this),
        std::bind(&LED::SetNoComms, this),
        std::bind(&LED::SetElevatorL1, this),
        std::bind(&LED::SetAlgaeHeld, this),
        std::bind(&LED::SetElevatorL2, this),
        std::bind(&LED::SetElevatorL3, this),
        std::bind(&LED::SetIDK, this),
        std::bind(&LED::SetGoToMcDonalds, this),
    };
}

void LED::Init() {
    LED_currentCommand = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;
    LED_prevCommand = ArduinoConstants::RIO_MESSAGES::WHITE;

    m_pserial = new frc::SerialPort(
        ArduinoConstants::BAUD_RATE_ARDUINO,
        ArduinoConstants::USB_PORT_ARDUINO,
        ArduinoConstants::DATA_BITS_ARDUINO,
        ArduinoConstants::PARITY_ARDUINO,
        ArduinoConstants::STOP_BITS_ARDUINO
    );
    m_pserial->DisableTermination();
    m_pserial->frc::SerialPort::kFlushOnAccess;
    m_pserial->SetFlowControl(frc::SerialPort::kFlowControl_None);
    m_pserial->SetReadBufferSize(0);
    m_pserial->SetWriteBufferMode(frc::SerialPort::kFlushOnAccess);

    m_rxIndex = 0;
    for(int i = 0; i < BUFF_SIZE; i++) {
        m_rxBuff[i] = 0;
    }
    //SetTargetType(LED_STAGE_enum::WHITE);
}

void LED::SetLEDState(ArduinoConstants::RIO_MESSAGES ledState) {
    (m_LEDArray[static_cast<int>(ledState)])();
}

void LED::Update() {
    // Not available on the practice bot
    // call this routine periodically to check for any readings and store
    // into result registers

    // get report if there is one
    // every time called, and every time through loop, get report chars if available
    // and add to rx buffer
    // when '\r' (or '\t') found, process reading
   
    // printf("LED: in Update, attempting to receive\n");

    while (m_pserial->GetBytesReceived() > 0) {
        m_pserial->Read(&m_rxBuff[m_rxIndex], 1);
    
        if((m_rxBuff[m_rxIndex] == '\r') || (m_rxBuff[m_rxIndex] == '\n')) {

            // process report
            if(m_rxIndex == 0) {
                // no report
                continue;
            }

            // terminate the report string
            m_rxBuff[m_rxIndex] = 0;

            // print the report:
            printf("RX: %s\n", m_rxBuff);
            m_pserial->Flush();

            ProcessReport();
           
            // printf("LED report: rx_buff\n");

            // reset for next report
            m_rxIndex = 0;
        } else {
            // have not received end of report yet
            if(m_rxIndex < BUFF_SIZE - 1) {
                m_rxIndex++;
            }
        }
    }

    if(LED_currentCommand != LED_prevCommand){
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

            case ArduinoConstants::RIO_MESSAGES::GOTOMCDONALDS:
                SendGoToMcDonaldsMSG();
                break;  

            default:
                break;
        }
    }
}

// instead of atoi(), UltrasonicSerial used strtol(&readBuffer[1], (char **)NULL, 10);

void LED::ProcessReport() {
    // parse report
    // no action needed, no report expected
}


// void LED::SetTargetType(LED_STAGE_enum target_type_param)
// {
// #ifdef COMP_BOT  // Not available on the practice bot
//  char cmd[10];
//  strncpy(cmd, "1,1,1\r\n", 8);
//  target_type = target_type_param;
//  // lazy way to build a message
//  cmd[4] = target_type ? '1' : '0';
//  m_serial.Write(cmd, strlen(cmd));
//  m_serial.Flush();
// #endif
// }


// LED_STAGE_enum LED::GetTargetType()
// {
//  return target_type;
// }

void LED::SetWhite() {
    LED_currentCommand = ArduinoConstants::RIO_MESSAGES::WHITE;
}

void LED::SendWhiteMSG() {
    char cmd[10];
    strncpy(cmd, "(0,0)\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();
}

void LED::SetMSGIdle() {
    LED_currentCommand = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;
}

void LED::SendIdleMSG() {
    char cmd[10];
    strncpy(cmd, "1,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();
}

void LED::SetNoComms() {
    LED_currentCommand = ArduinoConstants::RIO_MESSAGES::NO_COMMS;
}

void LED::SendNoCommsMSG() {
    int ret = 0;
    printf("in SetNoComms\n");
    char cmd[10];
    strncpy(cmd, "2,0\r\n", 8);
    printf("about to Write: %s\n", cmd);
    ret = m_pserial->Write(cmd, strlen(cmd));
    printf("written: %d characters\n", ret);
    printf("about to Flush\n");
    m_pserial->Flush();
    printf("flushed\n\n");
}

void LED::SetElevatorL1() {
    LED_currentCommand = ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1;
}

void LED::SendElevatorL1MSG() {
    char cmd[10];
    strncpy(cmd, "3,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();
}

void LED::SetAlgaeHeld() {
    LED_currentCommand = ArduinoConstants::RIO_MESSAGES::ALGAE_HELD;
}

void LED::SendAlgaeHeldMSG() {
    char cmd[10];
    sprintf(cmd, "4,0,%1.2f\r\n");
    printf("%s\n", cmd);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();
}

void LED::SetElevatorL2() {
    LED_currentCommand = ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2;
}

void LED::SendElevatorL2MSG() {
    char cmd[10];
    strncpy(cmd, "5,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();
}

void LED::SetElevatorL3() {
    LED_currentCommand = ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3;
}

void LED::SendElevatorL3MSG() {
    char cmd[10];
    strncpy(cmd, "6,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();
}

void LED::SetIDK() {
    LED_currentCommand = ArduinoConstants::RIO_MESSAGES::IDK;
}

void LED::SendIDKMSG() {
    char cmd[10];
    strncpy(cmd, "7,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();
}

void LED::SetGoToMcDonalds() {
    LED_currentCommand = ArduinoConstants::RIO_MESSAGES::GOTOMCDONALDS;
}

void LED::SendGoToMcDonaldsMSG() {
    char cmd[10];
    strncpy(cmd, "8,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();    
}