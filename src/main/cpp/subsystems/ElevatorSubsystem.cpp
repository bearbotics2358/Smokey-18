#include "subsystems/ElevatorSubsystem.h"

#include <ctre/phoenix6/controls/Follower.hpp>

ElevatorSubsystem::ElevatorSubsystem():
m_elevatorMotor1(kElevatorMotor1Id),
m_elevatorMotor2(kElevatorMotor2Id),
m_elevatorLimitSwitch(kLimitSwitchId)
{
    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kV = .12;
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    m_elevatorMotor1.GetConfigurator().Apply(slot0Configs);
    m_elevatorMotor2.GetConfigurator().Apply(slot0Configs);

    ctre::phoenix6::controls::Follower follower(kElevatorMotor1Id, true);
    m_elevatorMotor2.SetControl(follower);
}