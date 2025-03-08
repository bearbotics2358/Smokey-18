// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CameraSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>
#include <units/acceleration.h>

static const double kDistanceFromReef = 0.5;

class AlignWithReef
    : public frc2::CommandHelper<frc2::Command, AlignWithReef> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param camera The subsystem used by this command.
   * @param drivetrain
   */
  explicit AlignWithReef(CameraSubsystem* camera, subsystems::CommandSwerveDrivetrain* drivetrain);
  void Initialize() override;
  void Execute() override;

  swerve::requests::RobotCentric robotOriented = swerve::requests::RobotCentric{}
    .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

 private:

  CameraSubsystem* m_camera;
  subsystems::CommandSwerveDrivetrain* m_drivetrain;

    static constexpr units::meters_per_second_t kMaxVelocity = 0.1_mps;
    static constexpr units::meters_per_second_squared_t kMaxAcceleration = 0.1_mps_sq;
    static constexpr units::radians_per_second_t kMaxAngularVelocity = 0.1_rad_per_s;
    static constexpr units::turns_per_second_squared_t kMaxAngularAcceleration = 0.1_rad_per_s_sq;
    static constexpr double kP = 1.0;
    static constexpr double kI = 1.0;
    static constexpr double kD = 0.0;


    frc::TrapezoidProfile<units::meter_t>::Constraints m_constraints {
        kMaxVelocity, kMaxAcceleration
    };

    frc::TrapezoidProfile<units::radian_t>::Constraints m_angularConstraints {
        kMaxAngularVelocity, kMaxAngularAcceleration
    };

    frc::ProfiledPIDController<units::meter_t> m_linearAlignmentPID {
        kP, kI, kD, m_constraints 
    };

    frc::ProfiledPIDController<units::radian_t> m_angularAlignmentPID {
        kP, kI, kD, m_angularConstraints 
    };

    static constexpr double TOLERANCE = 1.0;
};
