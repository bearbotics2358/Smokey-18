
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

constexpr int kAlgaeMotorLeft = 49;
constexpr int kAlgaeMotorRight = 50;
constexpr int kAlgaePivot = 35;

class AlgaeSubsystem : public frc2::SubsystemBase {
 public:
  AlgaeSubsystem(IAlgaeDataProvider* dataProvider);
  void Periodic();

  frc2::CommandPtr Intake();
  frc2::CommandPtr Dispense();
  
  frc2::CommandPtr SetGoalAngle(double angle);

  units::degree_t CurrentAngle();

 private:
    void GoToAngle();

    ctre::phoenix6::hardware::TalonFX m_algaePivotMotor;

    rev::spark::SparkMax m_algaeLeftMotor;
    rev::spark::SparkMax m_algaeRightMotor;

    // TODO: tune these values
    static constexpr units::turns_per_second_t kMaxVelocity = 1.5_tps;
    static constexpr units::turns_per_second_squared_t kMaxAcceleration = 0.75_tr_per_s_sq;
    static constexpr double kP = 12;
    static constexpr double kI = 0.5;
    static constexpr double kD = 0.3;

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

    units::degree_t m_setpointAngle = 100.0_deg;

    static constexpr double TOLERANCE = 1.0;

    IAlgaeDataProvider* m_algaeDataProvider;
};