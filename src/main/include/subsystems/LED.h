#pragma once

// LED.h - Control LED lights representing desired game piece

#include <frc/SerialPort.h>

#define BUFF_SIZE 256

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

namespace ArduinoConstants {
    static constexpr int BAUD_RATE_ARDUINO = 115200;
    static constexpr frc::SerialPort::Port USB_PORT_ARDUINO = frc::SerialPort::kUSB;
    static constexpr int DATA_BITS_ARDUINO = 8;
    static constexpr frc::SerialPort::Parity PARITY_ARDUINO = frc::SerialPort::kParity_None;
    static constexpr frc::SerialPort::StopBits STOP_BITS_ARDUINO = frc::SerialPort::kStopBits_Two;

    enum RIO_MESSAGES {
        MSG_IDLE = 1,
        NO_COMMS = 2,
        ELEVATOR_L1 = 3,
        ALGAE_HELD = 4,
        ELEVATOR_L2 = 5,
        ELEVATOR_L3 = 6,
        IDK = 7,
        CLIMBLEFTTRUE = 8,
        CLIMBLEFTFALSE = 9,
        CLIMBRIGHTTRUE = 10,
        CLIMBRIGHTFALSE = 11, 
    };
}

class LED : public frc2::SubsystemBase {
public:
    LED();
    ~LED() override;

    void Periodic() override;

    void SetLEDState(ArduinoConstants::RIO_MESSAGES ledState);

private:
    void SendMSG(const char* message);
    void ProcessReport();  // Kept this as it was in the original

    frc::SerialPort* m_pserial;
    char m_rxBuff[32];
    int m_rxIndex = 0;

    ArduinoConstants::RIO_MESSAGES m_LEDPrevCommand;
    ArduinoConstants::RIO_MESSAGES m_LEDCurrentCommand = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;
};
