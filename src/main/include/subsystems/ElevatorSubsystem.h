#pragma once


#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <iostream>
#include <frc/DigitalInput.h>


constexpr int kElevatorMotor1Id = 36;
constexpr int kElevatorMotor2Id = 37;
constexpr int kLimitSwitchId = 1;

// @todo Assign these to real values when we know the distances
constexpr units::inch_t kElevatorStowPosition = 0_in;
constexpr units::inch_t kElevatorL1Position = 0_in;
constexpr units::inch_t kElevatorL2Position = 0_in;
constexpr units::inch_t kElevatorL3Position = 0_in;
constexpr units::inch_t kElevatorL4Position = 0_in;

// variable for go button use(will be changed when buttons are pressed on the driverstation)
    units::inch_t m_SavedElevatorCommand = units::inch_t{0};  // Initialize to a default value


class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        ElevatorSubsystem();
        void PrepareElevatorCommand(units::inch_t newPosition);

        void Periodic() override;
        void PlotElevatorPosition();

        frc2::CommandPtr GoToSavedPosition();
        frc2::CommandPtr SetPositionCommand(units::turn_t position);

    private:
        ctre::phoenix6::hardware::TalonFX m_elevatorMotor1;
        // ctre::phoenix6::hardware::TalonFX m_elevatorMotor2;
        frc::DigitalInput m_elevatorLimitSwitch;
        ctre::phoenix6::controls::PositionVoltage m_positionVoltage = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);
};