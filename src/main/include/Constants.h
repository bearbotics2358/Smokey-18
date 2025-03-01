#pragma once

#include <frc/SerialPort.h>

namespace ArduinoConstants {
    static constexpr int BAUD_RATE_ARDUINO = 115200;
    static constexpr frc::SerialPort::Port USB_PORT_ARDUINO = frc::SerialPort::kUSB;
    static constexpr int DATA_BITS_ARDUINO = 8;
    static constexpr frc::SerialPort::Parity PARITY_ARDUINO = frc::SerialPort::kParity_None;
    static constexpr frc::SerialPort::StopBits STOP_BITS_ARDUINO = frc::SerialPort::kStopBits_Two;

    static constexpr int ARDUINO_DIO_PIN = 1;
    static constexpr int NUMBER_OF_LED_STATES = 9;

    enum RIO_MESSAGES {
        MSG_IDLE = 1,
        NO_COMMS = 2,
        ELEVATOR_L1 = 3,
        ALGAE_HELD = 4,
        ELEVATOR_L2 = 5,
        ELEVATOR_L3 = 6,
        IDK = 7,
        TEST = 8,
    };
}

namespace OperatorConstants {
    static constexpr int kOperatorPort = 4;
}

namespace DriverConstants {
    static constexpr int kDriverPort = 5;
}