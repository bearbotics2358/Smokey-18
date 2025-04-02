/**
 * @file Climber.h
 */

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

#include <frc2/command/button/Trigger.h>

/**
 * @brief The CAN ID of the climber motor.
 */
constexpr int kClimberMotor1Id = 32;

/**
 * @brief The angle when the climber fully extended.
 */
constexpr units::degree_t kClimberStartAngle = 165.0_deg;

/**
 * @brief The angle to end at for deep climbing.
 */
constexpr units::degree_t kClimberEndAngle = 41.0_deg;

/**
 * @brief The angle at the start of the match.
 */
constexpr units::degree_t kClimberStowAngle = 15.0_deg;

const double kClimberGearRatio = 1.0;

/**
 * @brief This subsystem controls the climber mechanism.
 */
class Climber : public frc2::SubsystemBase {
public:
    Climber(IClimberDataProvider* dataProvider);
    void Periodic() override;

    /**
     * @brief Moves the climber to @ref kClimberEndAngle.
     */
    frc2::CommandPtr Climb();

    /**
     * @brief Moves the climber to @ref kClimberStartAngle.
     */
    frc2::CommandPtr Extend();

    /**
     * @brief Moves the climber to @ref kClimberStowAngle.
     */
    frc2::CommandPtr Stow();

    units::degree_t CurrentAngle();

    /**
     * @brief Stops the climber motor.
     */
    frc2::CommandPtr StopClimber();

    /**
     * @brief Triggers when the proximity sensor detects the left side of the climber.
     */
    frc2::Trigger IsLeftCageHooked = frc2::Trigger([this] {
        return m_climberDataProvider->IsLeftCageHooked();
    });

    /**
     * @brief Triggers when the proximity sensor detects the right side of the climber.
     */
    frc2::Trigger IsRightCageHooked = frc2::Trigger([this] {
        return m_climberDataProvider->IsRightCageHooked();
    });

private:
    void SetMotorVoltage();

    IClimberDataProvider* m_climberDataProvider;

    ctre::phoenix6::hardware::TalonFX m_climberMotor;

    static constexpr units::turns_per_second_t kMaxVelocity = 60_deg_per_s;
    static constexpr units::turns_per_second_squared_t kMaxAcceleration = 80_deg_per_s_sq;
    static constexpr double kP = 77.5;
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
