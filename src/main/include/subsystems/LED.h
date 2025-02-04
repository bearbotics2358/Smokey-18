/* 
 * LED.cpp - Control LED lights representing desired game piece
 */

#ifndef H_LED
#define H_LED

#include "Constants.h"
#include <frc/SerialPort.h>

#define BUFF_SIZE 256

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>

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

    void SetLEDState(ArduinoConstants::RIO_MESSAGES ledState);

private:
    std::array<std::function<void()>, ArduinoConstants::NUMBER_OF_LED_STATES> m_LEDArray;

    frc::SerialPort* m_pserial;
    char m_rxBuff[BUFF_SIZE];
    int m_rxIndex = 0;
    float m_valAngle = 0;
    //LED_STAGE_enum target_type = LED_STAGE_enum::WHITE;
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
    void SendGoToMcDonaldsMSG();
};

#endif