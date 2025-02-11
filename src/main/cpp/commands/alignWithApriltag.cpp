// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/alignWithApriltag.h"

alignWithApriltag::alignWithApriltag(CameraSubsystem* camera, TunerSwerveDrivetrain* drivetrain)
    : m_camera{camera} {
  // Register that this command requires the subsystem.
  AddRequirements(m_camera);
}

void alignWithApriltag::Initialize() {
    while (abs(m_camera->getYDistance()) > 0.05) {
        //drive
    }
}