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
    double forwardPID = m_XAlignmentPID.Calculate(m_distanceFromReefSetpoint.value(), m_camera->getForwardTransformation().value());
    frc::SmartDashboard::PutNumber("Forward PID", forwardPID);

    double horizontalPID = m_YAlignmentPID.Calculate(m_lateralSetpoint.value(), m_camera->getHorizontalTransformation().value());
    frc::SmartDashboard::PutNumber("Horizontal PID", horizontalPID);
}

void AlignWithReef::Execute() {
    double horizontalPID = m_YAlignmentPID.Calculate(m_lateralSetpoint.value(), m_camera->getHorizontalTransformation().value());
    double forwardPID = m_XAlignmentPID.Calculate(m_distanceFromReefSetpoint.value(), m_camera->getForwardTransformation().value());
    double rotationPID = m_angularAlignmentPID.Calculate(m_rotationalSetpoint, units::radian_t(m_camera->getZRotation()));

    frc::SmartDashboard::PutNumber("Horizontal PID", horizontalPID);
    frc::SmartDashboard::PutNumber("Forward PID", forwardPID);
    frc::SmartDashboard::PutNumber("Rotation PID", rotationPID);
    m_drivetrain->SetControl(robotOriented.WithVelocityX(units::meters_per_second_t(forwardPID))
                                          .WithVelocityY(units::meters_per_second_t(horizontalPID))
                                          .WithRotationalRate(-units::radians_per_second_t(rotationPID)));
}

bool AlignWithReef::IsFinished() {
    double lateral_diff = fabs(units::inch_t(m_lateralSetpoint - m_camera->getHorizontalTransformation()).value());
    double forward_diff = fabs(units::inch_t(m_distanceFromReefSetpoint - m_camera->getForwardTransformation()).value());
    double rotational_diff = fabs(units::radian_t(m_rotationalSetpoint - m_camera->getZRotation()).value());

    bool isFinished = (lateral_diff < units::inch_t(kLateralTolerance).value() && forward_diff < units::inch_t(kLateralTolerance).value()) ||
        m_camera->visibleTargets() == false;

    frc::SmartDashboard::PutBoolean("Align Finished", isFinished);

    return isFinished;
}