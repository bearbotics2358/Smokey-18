#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <iostream>
#include <frc/DigitalInput.h>
#include <units/length.h>

// @todo Set these to the correct IDs when they are assigned on the real robot
constexpr int kElevatorMotor1Id = 20;
constexpr int kElevatorMotor2Id = 19;
constexpr int kLimitSwitchId = 18;

// @todo Assign these to real values when we know the distances
constexpr units::inch_t kElevatorStowPosition = 0_in;
constexpr units::inch_t kElevatorL1Position = 0_in;
constexpr units::inch_t kElevatorL2Position = 0_in;
constexpr units::inch_t kElevatorL3Position = 0_in;
constexpr units::inch_t kElevatorL4Position = 0_in;

class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        ElevatorSubsystem();

        void Periodic() override;
        void PlotElevatorPosition();

        frc2::CommandPtr SetPositionCommand(units::inch_t position);

    private:
        ctre::phoenix6::hardware::TalonFX m_elevatorMotor1;
        ctre::phoenix6::hardware::TalonFX m_elevatorMotor2;
        frc::DigitalInput m_elevatorLimitSwitch;
};