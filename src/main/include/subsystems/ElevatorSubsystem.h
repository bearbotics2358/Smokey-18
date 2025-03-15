#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

#include <units/length.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc2/command/button/Trigger.h>

constexpr int kElevatorMotor1Id = 36;
constexpr int kElevatorMotor2Id = 37;
constexpr int kLimitSwitchId = 0;

// @todo Assign these to real values when we know the distances
constexpr units::inch_t kElevatorCollectPosition = 3.0_in;
constexpr units::inch_t kElevatorStowPosition = 0_in;
constexpr units::inch_t kElevatorL1Position = 0_in;
constexpr units::inch_t kElevatorL2Position = 12_in;
constexpr units::inch_t kElevatorL3Position = 28_in;
constexpr units::inch_t kElevatorL4Position = 57_in;

constexpr float kSlowElevator = 0.6;

// TODO: Tune later
constexpr units::inch_t kHeightThreshold = 20_in;

class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        ElevatorSubsystem();

        void Periodic() override;
        void PlotElevatorPosition();

        units::inch_t CurrentHeight();

        void PrepareElevator(units::inch_t newPosition);
        frc2::CommandPtr GoToHeight(units::inch_t height);
        frc2::CommandPtr GoToSavedPosition();

        const units::inch_t WHEEL_RADIUS = 1.325_in;
        // 9 to 1
        const double GEAR_RATIO = 9.0;

        void SetMotorVoltage();

        frc2::Trigger IsHeightAboveThreshold = frc2::Trigger([this] {
            return GetElevatorHeightAboveThreshold();
        });
    private:
        bool GetElevatorHeightAboveThreshold();

        ctre::phoenix6::hardware::TalonFX m_elevatorMotor1;
        ctre::phoenix6::hardware::TalonFX m_elevatorMotor2;
        frc::DigitalInput m_elevatorLimitSwitch;

        bool IsMagneticLimitSwitchActive();

        static constexpr double TOLERANCE = 0.30;

        static constexpr units::meters_per_second_t kMaxVelocity = 3.0_mps;
        static constexpr units::meters_per_second_squared_t kMaxAcceleration = 3.0_mps_sq;
        static constexpr double kP = 19.0;
        static constexpr double kI = 1.5;
        static constexpr double kD = 0.0;
        static constexpr units::volt_t kS = 0.30_V;
        static constexpr units::volt_t kG = 0.25_V;
        static constexpr auto kV = 0.0_V / 1.0_mps;

        frc::TrapezoidProfile<units::meters>::Constraints m_constraints {
            kMaxVelocity, kMaxAcceleration};

        frc::ProfiledPIDController<units::meters> m_elevatorPID{
            kP, kI, kD, m_constraints
        };

        frc::ElevatorFeedforward m_feedforward{kS, kG, kV};

        // Changing m_elevatorSetpointHeight will send the elevator to that position immediately
        units::inch_t m_elevatorSetpointHeight = 0_in;

        // Use m_desiredElevatorPosition when preparing the elevator for a call to GoToSavedPosition()
        units::inch_t m_desiredElevatorPosition;
};