/* 
 * LED.cpp - Control LED lights representing desired game piece
 */

#ifndef H_LED
#define H_LED

#include "Protocol2025.h" //yay i can use CONE and CUBE
#include <frc/SerialPort.h>

#define BUFF_SIZE 256

#include <frc2/command/SubsystemBase.h>
#include <generated/TunerConstants.h>

#include <functional>
#include <array>

// enum LED_STAGE_enum {
//   WHITE = 0,
//   LED_IDLE,
//   NO_COMMS,
//   NOTE_COLLECTED  
// };

class LED : public frc2::SubsystemBase {
public:
    LED();
    virtual ~LED() = default;

    void Init();
    void Update();

    void SetWhite();
    void SetMSGIdle();
    void SetNoComms();
    void SetElevatorL1();
    void SetAlgaeHeld();
    void SetElevatorL2();
    void SetElevatorL3();
    void SetIDK();
    void ProcessReport();
    void SetGoToMcDonalds();
    //enum LED_STAGE_enum GetTargetRangeIndicator();
    //void SetTargetType(LED_STAGE_enum target_type_param);
    //LED_STAGE_enum GetTargetType();

    std::array<std::function<void()>, ARDUINO_CONSTANTS::NUMBER_OF_LED_STATES> m_LEDArray;

private:
    frc::SerialPort* m_pserial;
    char rx_buff[BUFF_SIZE];
    int rx_index = 0;
    float valAngle  = 0;
    //LED_STAGE_enum target_type = LED_STAGE_enum::WHITE;
    RIO_msgs_enum LED_prevCommand = RIO_msgs_enum::MSG_IDLE;
    RIO_msgs_enum LED_currentCommand = RIO_msgs_enum::MSG_IDLE;
    void SendWhiteMSG();
    void SendIdleMSG();
    void SendNoCommsMSG();
    void SendElevatorL1MSG();
    void SendAlgaeHeldMSG();
    void SendElevatorL2MSG();
    void SendElevatorL3MSG();
    void SendIDKMSG();
    void SendGoToMcDonaldsMSG();
};

#endif