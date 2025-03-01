#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

// @todo All constants here are TEMPORARY - change them as needed
constexpr int kClimberMotor1Id = 50;

constexpr float kSlowClimber = 0.5;

constexpr units::turn_t kClimberStartTurns = 20.0_tr;
constexpr units::turn_t kClimberEndTurns = 10.0_tr;
constexpr units::turn_t kClimberStowTunrs = 0.0_tr;

const double kClimberGearRatio = 1.0;

class Climber : public frc2::SubsystemBase {
public:
    Climber();
    void Periodic() override;

    frc2::CommandPtr Climb();
    frc2::CommandPtr CancelClimb();
    frc2::CommandPtr Stow();
    void SetMotorVoltage();
    units::turn_t CurrentTurns();

    frc2::CommandPtr StopClimbMotor();

private:
    ctre::phoenix6::hardware::TalonFX m_climberMotor;

        static constexpr units::meters_per_second_t kMaxVelocity = 0.25_mps;
        static constexpr units::meters_per_second_squared_t kMaxAcceleration = 0.75_mps_sq;
        static constexpr double kP = 1.0;
        static constexpr double kI = 1.0;
        static constexpr double kD = 0.0;

        frc::TrapezoidProfile<units::meters>::Constraints m_constraints {
            kMaxVelocity, kMaxAcceleration};

        frc::ProfiledPIDController<units::meters> m_elevatorPID{
            kP, kI, kD, m_constraints
        };

        units::turn_t m_setpointTurns = 0_tr;
};
