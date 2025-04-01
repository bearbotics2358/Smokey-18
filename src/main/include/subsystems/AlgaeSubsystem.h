/**
 * @file AlgaeSubsystem.h
 */

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>
#include "subsystems/IAlgaeDataProvider.h"

#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ArmFeedforward.h>

#include <units/velocity.h>
#include <units/acceleration.h>

/**
 * @brief The CAN ID for the intake motor.
 */
constexpr int kAlgaeMotor = 49;

/**
 * @brief The CAN ID for the pivot motor.
 */
constexpr int kAlgaePivot = 35;

constexpr units::degree_t kAlgaeStowAngle = 150.0_deg;
constexpr units::degree_t kAlgaeExtendedAngle = 90.0_deg;

class AlgaeSubsystem : public frc2::SubsystemBase {
 public:
  AlgaeSubsystem(IAlgaeDataProvider* dataProvider);
  void Periodic();

  /**
   * @brief Runs the algae intake motor **counter-clockwise.**
   */
  frc2::CommandPtr Intake();

  /**
   * @brief Runs the algae intake motor **clockwise.**
   */
  frc2::CommandPtr Dispense();

  bool IsAlgaeStored();

  /**
   * @brief Sets the goal angle the algae arm should reach.
   */
  frc2::CommandPtr SetGoalAngle(units::degree_t angle);
  units::degree_t CurrentAngle();

 private:
    void GoToAngle();

    ctre::phoenix6::hardware::TalonFX m_algaePivotMotor;

    rev::spark::SparkMax m_algaeMotor;

    static constexpr units::turns_per_second_t kMaxVelocity = 1.5_tps;
    static constexpr units::turns_per_second_squared_t kMaxAcceleration = 0.75_tr_per_s_sq;
    static constexpr double kP = 15.0;
    static constexpr double kI = 0.0;
    static constexpr double kD = 0.0;

    static constexpr units::volt_t kS = 0.25_V;
    static constexpr units::volt_t kG = 0.0_V;
    static constexpr auto kV = 0.0_V / 1.0_rad_per_s;

    frc::TrapezoidProfile<units::turns>::Constraints m_constraints {
        kMaxVelocity, kMaxAcceleration
    };

    frc::ProfiledPIDController<units::turns> m_algaePID {
        kP, kI, kD, m_constraints
    };

    frc::ArmFeedforward m_algaeFeedForward{kS, kG, kV};

    units::degree_t m_setpointAngle = kAlgaeStowAngle;

    static constexpr double TOLERANCE = 1.0;

    IAlgaeDataProvider* m_algaeDataProvider;
};