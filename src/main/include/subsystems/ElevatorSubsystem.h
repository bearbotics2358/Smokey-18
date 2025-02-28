#pragma once


#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <iostream>
#include <frc/DigitalInput.h>
#include <units/length.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedForward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Encoder.h>

constexpr int kElevatorMotor1Id = 36;
constexpr int kElevatorMotor2Id = 37;
constexpr int kLimitSwitchId = 0;

// @todo Assign these to real values when we know the distances
constexpr units::inch_t kElevatorStowPosition = 0_in;
constexpr units::inch_t kElevatorL1Position = 5_in;
constexpr units::inch_t kElevatorL2Position = 0_in;
constexpr units::inch_t kElevatorL3Position = 0_in;
constexpr units::inch_t kElevatorL4Position = 0_in;

constexpr float kSlowElevator = 0.6;



class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        ElevatorSubsystem();

        void Periodic() override;
        void PlotElevatorPosition();

        frc2::CommandPtr SetPositionCommand(units::inch_t position);

        frc2::CommandPtr Stop();

        // Test function to slowly raise the elevator
        frc2::CommandPtr Raise();

        // Test function to slowly lower the elevator
        frc2::CommandPtr Lower();

        units::inch_t CurrentHeight();

        frc2::CommandPtr GoToHeight(units::inch_t height);

        const units::inch_t WHEEL_RADIUS = 1.5_in;
        // 9 to 1
        const double GEAR_RATIO = 9.0;

        void SetMotorVoltage();

    private:
        ctre::phoenix6::hardware::TalonFX m_elevatorMotor1;
        ctre::phoenix6::hardware::TalonFX m_elevatorMotor2;
        frc::DigitalInput m_elevatorLimitSwitch;

        bool IsMagneticLimitSwitchActive();

        static constexpr units::meters_per_second_t kMaxVelocity = 0.50_mps;
        static constexpr units::meters_per_second_squared_t kMaxAcceleration = 0.75_mps_sq;
        static constexpr double kP = 40.0;
        static constexpr double kI = 1.0;
        static constexpr double kD = 0.0;
        static constexpr units::volt_t kS = 1.1_V;
        static constexpr units::volt_t kG = 1.2_V;
        static constexpr auto kV = 1.3_V / 0.3_mps;

        frc::TrapezoidProfile<units::meters>::Constraints m_constraints {
            kMaxVelocity, kMaxAcceleration};

        frc::ProfiledPIDController<units::meters> m_elevatorController{
            kP, kI, kD, m_constraints
        };
        
        frc::ElevatorFeedforward m_feedforward{kS, kG,kV};
        units::inch_t m_setpointHeight = 0_in;
};