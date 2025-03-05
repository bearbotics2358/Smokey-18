
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/velocity.h>
#include <units/acceleration.h>

class AlgaeSubsystem : public frc2::SubsystemBase {
 public:
  AlgaeSubsystem();

  frc2::CommandPtr SetSpeed(double speed);

  frc2::CommandPtr GoToAngle(double angle);

  void SetAlgaeVoltage();

  units::turn_t CurrentAngle();

 private:
    ctre::phoenix6::hardware::TalonFX m_algaePivotMotor;

    // TODO: change ids later
    const int kAlgaeMotorLeft = 0;
    const int kAlgaeMotorRight = 1;

    rev::spark::SparkMax m_algaeLeftMotor;
    rev::spark::SparkMax m_algaeRightMotor;

    const int kAlgaePivot = 2;

    // TODO: tune these values
    static constexpr units::turns_per_second_t kMaxVelocity = 0.25_tps;
    static constexpr units::turns_per_second_squared_t kMaxAcceleration = 0.75_tr_per_s_sq;
    static constexpr double kP = 1.0;
    static constexpr double kI = 1.0;
    static constexpr double kD = 0.0;

    frc::TrapezoidProfile<units::turns>::Constraints m_constraints {
        kMaxVelocity, kMaxAcceleration
    };

    frc::ProfiledPIDController<units::turns> m_algaePID {
        kP, kI, kD, m_constraints 
    };

    units::turn_t m_setpointAngle = 0.0_tr;

    static constexpr double TOLERANCE = 1.0;
};