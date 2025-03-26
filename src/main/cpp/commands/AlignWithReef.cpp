// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignWithReef.h"

AlignWithReef::AlignWithReef(
    CameraSubsystem* camera, 
    subsystems::CommandSwerveDrivetrain* drivetrain, 
    ReefSide reefSide
): m_camera{camera}, m_drivetrain{drivetrain}, m_reefSide(reefSide) {
    // Register that this command requires the subsystem.
    AddRequirements(m_drivetrain);
    AddRequirements(m_camera);

    if (reefSide == ReefSide::Left) {
        m_strafeSetpoint = kStrafeLeftReefSetpoint;
    } else {
        m_strafeSetpoint = kStrafeRightReefSetpoint;
    }

    m_rotationalPID.EnableContinuousInput(-180.0, 180.0);
}

void AlignWithReef::Initialize() {
    m_targetTagId = m_camera->GetTargetTagId();

    // If a target tag exists and it is in the reef tag list, then we should attempt to align to it
    if (m_targetTagId && kTagAngleMap.contains(m_targetTagId.value())) {
        m_targetDegrees = kTagAngleMap.at(m_targetTagId.value());
    } else {
        m_targetTagId = std::nullopt;
    }
}

void AlignWithReef::Execute() {
    if (!m_targetTagId) {
        // No target exists, so don't attempt to align
        return;
    }

    double forward = m_XAlignmentPID.Calculate(m_camera->getForwardTransformation().value(), kDistanceFromReefSetpoint.value());
    forward = std::clamp(forward, -1.0, 1.0);

    units::meter_t forward_diff = units::math::abs(kDistanceFromReefSetpoint - m_camera->getForwardTransformation());
    if (forward_diff < kForwardTolerance) {
        forward = 0.0;
    }

    double strafe = m_YAlignmentPID.Calculate(m_camera->getStrafeTransformation().value(), m_strafeSetpoint.value());
    strafe = std::clamp(strafe, -1.0, 1.0);

    units::meter_t strafe_diff = units::math::abs(m_strafeSetpoint - m_camera->getStrafeTransformation());
    if (strafe_diff < kStrafeTolerance) {
        strafe = 0.0;
    }

    units::degree_t currentDegrees = m_drivetrain->GetPose().Rotation().Degrees();
    double rotation = m_rotationalPID.Calculate(currentDegrees.value(), m_targetDegrees.value());
    rotation = std::clamp(rotation, -1.0, 1.0);

    frc::SmartDashboard::PutNumber("Strafe PID", strafe);
    frc::SmartDashboard::PutNumber("Forward PID", forward);
    frc::SmartDashboard::PutNumber("Rotation PID", rotation);

    m_drivetrain->SetControl(robotOriented.WithVelocityX(-forward * kMaxVelocity)
                                          .WithVelocityY(-strafe * kMaxVelocity)
                                          .WithRotationalRate(rotation * kMaxAngularVelocity));
}

bool AlignWithReef::IsFinished() {
    if (!m_targetTagId) {
        // If there is no target tag, finish the command immediately
        return true;
    }

    units::meter_t forward_diff = units::math::abs(kDistanceFromReefSetpoint - m_camera->getForwardTransformation());
    units::meter_t strafe_diff = units::math::abs(m_strafeSetpoint - m_camera->getStrafeTransformation());
    units::degree_t currentDegrees = m_drivetrain->GetPose().Rotation().Degrees();
    units::degree_t rotational_diff = units::math::abs(currentDegrees - m_targetDegrees);

    // If the bot is within tolerance for X, Y and rotational position, then we consider the command finished.
    // If we lose the ability to see the tag, also end the command.
    return (strafe_diff < kStrafeTolerance
                && forward_diff < kForwardTolerance
                && rotational_diff < kRotationTolerance);
}