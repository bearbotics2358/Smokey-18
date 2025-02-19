#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <iostream>
#include <frc/DigitalInput.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        ElevatorSubsystem();
    private:
    // @todo Set these to the correct IDs when they are assigned on the real robot
        const int kElevatorMotor1Id = 20;
        const int kElevatorMotor2Id = 19;
        const int kLimitSwitchId = 18;

        ctre::phoenix6::hardware::TalonFX m_elevatorMotor1;
        ctre::phoenix6::hardware::TalonFX m_elevatorMotor2;
        frc::DigitalInput m_elevatorLimitSwitch;
};