#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <iostream>
#include <frc/DigitalInput.h>

// @todo Set these to the correct IDs when they are assigned on the real robot
constexpr int kElevatorMotor1Id = 20;
constexpr int kElevatorMotor2Id = 19;
constexpr int kLimitSwitchId = 18;

class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        ElevatorSubsystem();
        void Periodic() override;
        void PlotElevatorPosition();
    private:
        ctre::phoenix6::hardware::TalonFX m_elevatorMotor1;
        ctre::phoenix6::hardware::TalonFX m_elevatorMotor2;
        frc::DigitalInput m_elevatorLimitSwitch;
};