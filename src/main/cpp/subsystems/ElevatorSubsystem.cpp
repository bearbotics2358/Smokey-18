#include "subsystems/ElevatorSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc2/command/Commands.h>

ElevatorSubsystem::ElevatorSubsystem():
m_elevatorMotor1(kElevatorMotor1Id),
// m_elevatorMotor2(kElevatorMotor2Id),
m_elevatorLimitSwitch(kLimitSwitchId)
{
    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    // slot0Configs.kV = .5;
    slot0Configs.kP = 0.8;
    slot0Configs.kI = 0.2;
    slot0Configs.kD = 0.1;
    m_elevatorMotor1.GetConfigurator().Apply(slot0Configs);

    // m_elevatorMotor2.GetConfigurator().Apply(slot0Configs);
    m_elevatorMotor1.SetPosition(0_tr);
};


frc2::CommandPtr ElevatorSubsystem::TurnToPosition(units::turn_t desiredRotations) {
    return frc2::cmd::RunOnce([this, desiredRotations] { 
        ctre::phoenix::StatusCode code = m_elevatorMotor1.SetControl(m_positionVoltage.WithPosition(desiredRotations).WithVelocity(1_rad_per_s));
        frc::SmartDashboard::PutNumber("Kraken Status Code", code);
    });
}