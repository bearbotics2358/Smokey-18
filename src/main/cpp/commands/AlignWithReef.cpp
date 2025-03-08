// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignWithReef.h"

//command to align with an apriltag - please DO NOT use yet it is not complete
AlignWithReef::AlignWithReef(CameraSubsystem* camera, subsystems::CommandSwerveDrivetrain* drivetrain)
    : m_camera{camera}, 
    m_drivetrain{drivetrain} {
  // Register that this command requires the subsystem.
  AddRequirements(m_drivetrain);
  AddRequirements(m_camera);
}

void AlignWithReef::Initialize() {

}

void AlignWithReef::Execute() {
    double horizontalPID = m_linearAlignmentPID.Calculate(0_m, units::meter_t(m_camera->getHorizontalTransformation()));
    double forwardPID = m_linearAlignmentPID.Calculate(0_m, units::meter_t(m_camera->getForwardTransformation() - kDistanceFromReef));
    double rotationPID = m_angularAlignmentPID.Calculate(0_rad, units::radian_t(m_camera->getZRotation()));

    m_drivetrain->ApplyRequest([this, horizontalPID, forwardPID, rotationPID]() -> auto&& {
        return robotOriented.WithVelocityX(units::meters_per_second_t(horizontalPID)).WithVelocityY(units::meters_per_second_t(forwardPID)).WithRotationalRate(units::radians_per_second_t(rotationPID));
    });
}

