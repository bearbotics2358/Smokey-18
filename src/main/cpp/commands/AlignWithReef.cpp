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
    // m_XAlignmentPID.SetGoal(m_distanceFromReefSetpoint);

    double forwardPID = m_XAlignmentPID.Calculate(m_distanceFromReefSetpoint.value(), m_camera->getForwardTransformation().value());
    frc::SmartDashboard::PutNumber("Forward PID", forwardPID);

    double horizontalPID = m_YAlignmentPID.Calculate(m_lateralSetpoint.value(), m_camera->getHorizontalTransformation().value());
    frc::SmartDashboard::PutNumber("Horizontal PID", horizontalPID);
}

void AlignWithReef::Execute() {
    frc::SmartDashboard::PutBoolean("Align started", true);
    double horizontalPID = m_YAlignmentPID.Calculate(m_lateralSetpoint.value(), m_camera->getHorizontalTransformation().value());
    double forwardPID = m_XAlignmentPID.Calculate(m_distanceFromReefSetpoint.value(), m_camera->getForwardTransformation().value());
    double rotationPID = m_angularAlignmentPID.Calculate(m_rotationalSetpoint, units::radian_t(m_camera->getZRotation()));

    // if (fabs(units::inch_t(m_distanceFromReefSetpoint - m_camera->getForwardTransformation()).value()) < 4.0) {
    //     forwardPID = 0.0;
    // }

    frc::SmartDashboard::PutNumber("Horizontal PID", horizontalPID);
    frc::SmartDashboard::PutNumber("Forward PID", forwardPID);
    frc::SmartDashboard::PutNumber("Rotation PID", rotationPID);

    m_drivetrain->SetControl(robotOriented.WithVelocityX(units::meters_per_second_t(forwardPID))
                                          .WithVelocityY(units::meters_per_second_t(horizontalPID))
                                          .WithRotationalRate(0_rad_per_s));
}

bool AlignWithReef::IsFinished() {
    // double horizontalPID = m_linearAlignmentPID.Calculate(m_lateralSetpoint, units::meter_t(m_camera->getHorizontalTransformation()));
    // double forwardPID = m_linearAlignmentPID.Calculate(m_distanceFromReefSetpoint, units::meter_t(m_camera->getForwardTransformation()));
    // double rotationPID = m_angularAlignmentPID.Calculate(m_rotationalSetpoint, units::radian_t(m_camera->getZRotation()));

    // double lateral_diff = fabs(m_lateralSetpoint.value() - m_camera->getHorizontalTransformation());
    // double forward_diff = fabs(m_distanceFromReefSetpoint.value() - m_camera->getForwardTransformation());
    // double rotational_diff = fabs(m_rotationalSetpoint.value() - m_camera->getZRotation());

    // bool isFinished = (lateral_diff < kLateralTolerance.value() && forward_diff < kLateralTolerance.value() && rotational_diff < kRotationTolerance.value());

    // frc::SmartDashboard::PutBoolean("Align Finished", isFinished);

    // return isFinished;

    return false;
}