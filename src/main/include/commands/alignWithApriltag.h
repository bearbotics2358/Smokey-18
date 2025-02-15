// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CameraSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"

class alignWithApriltag
    : public frc2::CommandHelper<frc2::Command, alignWithApriltag> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param camera The subsystem used by this command.
   * @param drivetrain
   */
  explicit alignWithApriltag(CameraSubsystem* camera, subsystems::CommandSwerveDrivetrain* drivetrain);
  void Initialize() override;

  swerve::requests::RobotCentric forwardStraight = swerve::requests::RobotCentric{}
    .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

 private:

  CameraSubsystem* m_camera;
  subsystems::CommandSwerveDrivetrain* m_drivetrain;

};
