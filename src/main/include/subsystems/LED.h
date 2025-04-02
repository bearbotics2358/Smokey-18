/**
 * @file LED.h
 */

#pragma once

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
        ELEVATOR_L3_ALGAE = 4,
        ELEVATOR_L2 = 5,
        ELEVATOR_L3 = 6,
        IDK = 7,
        CLIMB_LEFT_TRUE = 8,
        CLIMB_LEFT_FALSE = 9,
        CLIMB_RIGHT_TRUE = 10,
        CLIMB_RIGHT_FALSE = 11,
    };
}

/**
 * @brief This class controls LED lights for different situations. 
 * See @ref ArduinoConstants::RIO_MESSAGES for these situations.
 */
class LED : public frc2::SubsystemBase {
public:
    LED();
    ~LED() override;

    /**
     * @brief Reads and sends messages to the Arduino board (that's connected to the RoboRio)
     * to change the LED patterns for different situations.
     */
    void Periodic() override;
    
    /**
     * @brief Changes the current LED state.
     * @param ledState The desired LED state to change to.
     */
    void SetLEDState(ArduinoConstants::RIO_MESSAGES ledState);

    /**
     * @brief The `frc2::CommandPtr` version of @ref LED::SetLEDState.
     */
    frc2::CommandPtr SetLEDStateCommand(ArduinoConstants::RIO_MESSAGES ledState);

private:
    void SendMSG(const char* message);
    void ProcessReport();  // Kept this as it was in the original\

    bool wasDSAttached = false;

    frc::SerialPort* m_pserial;
    char m_rxBuff[32];
    int m_rxIndex = 0;

    ArduinoConstants::RIO_MESSAGES m_LEDPrevCommand;
    ArduinoConstants::RIO_MESSAGES m_LEDCurrentCommand = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;
};
