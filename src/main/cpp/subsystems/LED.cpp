/* LED.cpp - Control LED lights represnting desired game piece


 */


#include <stdio.h> // printf
#include <stdlib.h> // atoi
#include <Prefs.h>
#include <string.h>
#include "LED.h"


LED::LED()
#ifdef COMP_BOT  // Not available on the practice bot
// :
    // m_serial(BAUD_RATE_ARDUINO, USB_PORT_ARDUINO, DATA_BITS_ARDUINO, PARITY_ARDUINO, STOP_BITS_ARDUINO)
#endif
    // comments from 2018:
    // USB1 is the onboard port closest to the center of the rio
    // I dunno which one USB2 is yet. (Rio docs aren't very helpful)


{
    Init();
}


void LED::Init()
{
    int i;
    LED_currentCommand = RIO_msgs_enum::MSG_IDLE;
    LED_prevCommand = RIO_msgs_enum::WHITE;


    /*
    sport = new SerialPort(baud);
    sport->DisableTermination();
    sport->SetWriteBufferMode(SerialPort::kFlushOnAccess);
    sport->SetFlowControl(SerialPort::kFlowControl_None);
    sport->SetReadBufferSize(0);
    */


    m_pserial = new frc::SerialPort(BAUD_RATE_ARDUINO, USB_PORT_ARDUINO, DATA_BITS_ARDUINO, PARITY_ARDUINO, STOP_BITS_ARDUINO);
    m_pserial->DisableTermination();
    m_pserial->frc::SerialPort::kFlushOnAccess;
    m_pserial->SetFlowControl(frc::SerialPort::kFlowControl_None);
    m_pserial->SetReadBufferSize(0);
    m_pserial->SetWriteBufferMode(frc::SerialPort::kFlushOnAccess);




    rx_index = 0;
    for(i = 0; i < BUFF_SIZE; i++) {
        rx_buff[i] = 0;
    }
    //SetTargetType(LED_STAGE_enum::WHITE);
}


void LED::Update()
{
#ifdef COMP_BOT  // Not available on the practice bot
    // call this routine periodically to check for any readings and store
    // into result registers


    // get report if there is one
    // every time called, and every time through loop, get repotr chars if available
    // and add to rx buffer
    // when '\r' (or '\t') found, process reading
   
    // printf("LED: in Update, attempting to receive\n");


    while (m_pserial->GetBytesReceived() > 0) {
        m_pserial->Read(&rx_buff[rx_index], 1);


        //printf("LED LED LED LED: %c\n", rx_buff[rx_index]);
 
       
        if((rx_buff[rx_index] == '\r')
             || (rx_buff[rx_index] == '\n')) {


            // process report
            if(rx_index == 0) {
                // no report
                continue;
            }


            // terminate the report string
            rx_buff[rx_index] = 0;


            // print the report:
            printf("RX: %s\n", rx_buff);
            m_pserial->Flush();


            ProcessReport();
           
            // printf("LED report: rx_buff\n");


            // reset for next report
            rx_index = 0;
        } else {
            // have not received end of report yet
            if(rx_index < BUFF_SIZE - 1) {
                rx_index++;
            }
        }
    }


    if(LED_currentCommand != LED_prevCommand){
        LED_prevCommand = LED_currentCommand;
        switch (LED_currentCommand)
        {
        case RIO_msgs_enum::WHITE:
            SendWhiteMSG();
            break;


        case RIO_msgs_enum::MSG_IDLE:
            SendIdleMSG();
            break;




        case RIO_msgs_enum::NO_COMMS:
            SendNoCommsMSG();
            break;


         case RIO_msgs_enum::ELEVATOR_L1:
            SendElevatorL1MSG();
            break;  


         case RIO_msgs_enum::ALGAE_HELD:
            SendAlgaeHeldMSG();
            break;  


         case RIO_msgs_enum::ELEVATOR_L2:
            SendElevatorL2MSG();
            break;  


         case RIO_msgs_enum::ELEVATOR_L3:
            SendElevatorL3MSG();
            break;  


         case RIO_msgs_enum::IDK:
            SendIDKMSG();
            break;  

         case RIO_msgs_enum::GOTOMCDONALDS:
            SendGoToMcDonaldsMSG();
            break;  

        default:
            break;
        }
    }
#endif
}


// instead of atoi(), UltrasonicSerial used strtol(&readBuffer[1], (char **)NULL, 10);




void LED::ProcessReport()
{
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


    LED_currentCommand = RIO_msgs_enum::WHITE;


}




void LED::SendWhiteMSG() {
    char cmd[10];
    strncpy(cmd, "(0,0)\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();
   
}




void LED::SetMSGIdle(){


    LED_currentCommand = RIO_msgs_enum::MSG_IDLE;
}




void LED::SendIdleMSG() {
    char cmd[10];
    strncpy(cmd, "1,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();


}


void LED::SetNoComms(){


LED_currentCommand = RIO_msgs_enum::NO_COMMS;
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




void LED::SetElevatorL1(){
LED_currentCommand = RIO_msgs_enum::ELEVATOR_L1;
}






void LED::SendElevatorL1MSG() {
    char cmd[10];
    strncpy(cmd, "3,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();




}




void LED::SetAlgaeHeld(){
LED_currentCommand = RIO_msgs_enum::ALGAE_HELD;
}






void LED::SendAlgaeHeldMSG() {
    char cmd[10];
    sprintf(cmd, "4,0,%1.2f\r\n");
    printf("%s\n", cmd);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();


}


void LED::SetElevatorL2(){


LED_currentCommand = RIO_msgs_enum::ELEVATOR_L2;
}


void LED::SendElevatorL2MSG() {
    char cmd[10];
    strncpy(cmd, "5,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();


}


void LED::SetElevatorL3(){


LED_currentCommand = RIO_msgs_enum::ELEVATOR_L3;
}


void LED::SendElevatorL3MSG() {
    char cmd[10];
    strncpy(cmd, "6,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();


}


void LED::SetIDK(){


LED_currentCommand = RIO_msgs_enum::IDK;
}


void LED::SendIDKMSG() {
    char cmd[10];
    strncpy(cmd, "7,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();


}

void LED::SetGoToMcDonalds(){
    LED_currentCommand = RIO_msgs_enum::GOTOMCDONALDS;
}
void LED::SendGoToMcDonaldsMSG(){
    char cmd[10];
    strncpy(cmd, "8,0\r\n", 8);
    m_pserial->Write(cmd, strlen(cmd));
    m_pserial->Flush();    
}