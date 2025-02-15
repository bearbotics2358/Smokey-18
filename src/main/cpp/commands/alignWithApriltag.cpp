// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/alignWithApriltag.h"

//command to align with an apriltag - please DO NOT use yet it is not complete
alignWithApriltag::alignWithApriltag(CameraSubsystem* camera, subsystems::CommandSwerveDrivetrain* drivetrain)
    : m_camera{camera}, 
    m_drivetrain{drivetrain} {
  // Register that this command requires the subsystem.
  AddRequirements(m_drivetrain);
  AddRequirements(m_camera);
}

void alignWithApriltag::Initialize() {
    while (fabs(m_camera->getYDistance()) > 0.05) {
        m_drivetrain->ApplyRequest([this]() -> auto&& {
            return forwardStraight.WithVelocityX(1_mps).WithRotationalRate(0.5_rad_per_s);
        });
    }
}

