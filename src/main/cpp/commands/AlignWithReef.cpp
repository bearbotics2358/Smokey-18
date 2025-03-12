// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignWithReef.h"

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
    double forward = m_XAlignmentPID.Calculate(m_distanceFromReefSetpoint.value(), m_camera->getForwardTransformation().value());
    double strafe = m_YAlignmentPID.Calculate(m_strafeSetpoint.value(), m_camera->getStrafeTransformation().value());
    double rotation = m_angularAlignmentPID.Calculate(m_rotationalSetpoint, units::radian_t(m_camera->getZRotation()));

    frc::SmartDashboard::PutNumber("Strafe PID", strafe);
    frc::SmartDashboard::PutNumber("Forward PID", forward);
    frc::SmartDashboard::PutNumber("Rotation PID", rotation);
    m_drivetrain->SetControl(robotOriented.WithVelocityX(units::meters_per_second_t(forward))
                                          .WithVelocityY(units::meters_per_second_t(strafe))
                                          .WithRotationalRate(-units::radians_per_second_t(rotation)));
}

bool AlignWithReef::IsFinished() {
    units::meter_t forward_diff = units::math::abs(m_distanceFromReefSetpoint - m_camera->getForwardTransformation());
    units::meter_t strafe_diff = units::math::abs(m_strafeSetpoint - m_camera->getStrafeTransformation());
    units::radian_t rotational_diff = units::math::abs(m_rotationalSetpoint - m_camera->getZRotation());

    // If the bot is within tolerance for X, Y and rotational position, then we consider the command finished.
    // If we lose the ability to see the tag, also end the command.
    return ((strafe_diff < kStrafeTolerance
                && forward_diff < kForwardTolerance
                && rotational_diff < units::radian_t(kRotationTolerance))
            || m_camera->visibleTargets() == false);
}