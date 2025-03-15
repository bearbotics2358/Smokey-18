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

#include <subsystems/IClimberDataProvider.h>

constexpr int kClimberMotor1Id = 32;

constexpr units::degree_t kClimberStartAngle = 168.0_deg;
constexpr units::degree_t kClimberEndAngle = 90.0_deg;
constexpr units::degree_t kClimberStowAngle = 10.0_deg;

const double kClimberGearRatio = 1.0;

class Climber : public frc2::SubsystemBase {
public:
    Climber(IClimberDataProvider* dataProvider);
    void Periodic() override;

    frc2::CommandPtr Climb();
    frc2::CommandPtr Extend();
    frc2::CommandPtr Stow();

    units::degree_t CurrentAngle();

    frc2::CommandPtr StopClimber();

private:
    void SetMotorVoltage();

    IClimberDataProvider* m_climberDataProvider;

    ctre::phoenix6::hardware::TalonFX m_climberMotor;

    static constexpr units::turns_per_second_t kMaxVelocity = 10_tps;
    static constexpr units::turns_per_second_squared_t kMaxAcceleration = 100_tr_per_s_sq;
    static constexpr double kP = 60.0;
    static constexpr double kI = 0.0;
    static constexpr double kD = 0.0;

    frc::TrapezoidProfile<units::turns>::Constraints m_constraints {
        kMaxVelocity, kMaxAcceleration
    };

    frc::ProfiledPIDController<units::turns> m_climberPID {
        kP, kI, kD, m_constraints
    };

    units::degree_t m_setpointAngle = kClimberStowAngle;
};
