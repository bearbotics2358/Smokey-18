// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignWithReef.h"

AlignWithReef::AlignWithReef(CameraSubsystem* camera, subsystems::CommandSwerveDrivetrain* drivetrain, bool useOffsetAlignment)
    : m_camera{camera},
    m_drivetrain{drivetrain} {
    // Register that this command requires the subsystem.
    AddRequirements(m_drivetrain);
    AddRequirements(m_camera);

    if (useOffsetAlignment) {
        m_alignmentTransform = kOffsetAlignedTransform;
    } else {
        m_alignmentTransform = kCenterAlignedTransform;
    }
}

void AlignWithReef::Initialize() {
    // To get the final pose lined up with the AprilTag
    //  1. Start with the pose of the AprilTag (field centric)
    //  2. Apply the transformation to get the pose we want the robot to achieve
    std::optional<frc::Pose3d> targetPose = m_camera->GetBestTargetPose();
    if (targetPose) {
        m_targetPose = targetPose.value().ToPose2d();
        m_targetPose.value().TransformBy(m_alignmentTransform);
    } else {
        m_targetPose = std::nullopt;
    }
}

void AlignWithReef::Execute() {
    if (m_targetPose) {
        const frc::Pose2d currentPose = m_drivetrain->GetState().Pose;
        const frc::Rotation2d desiredHeading = frc::Rotation2d{m_camera->getZRotation()};
        const frc::ChassisSpeeds pidSpeeds = m_holonomicPID.Calculate(currentPose, m_targetPose.value(), kMaxLinearVelocity, desiredHeading);

        m_drivetrain->SetControl(m_fieldSpeedRequest.WithSpeeds(pidSpeeds));
    }
}

bool AlignWithReef::IsFinished() {
    // If no target pose was generated, end the command right away
    if (!m_targetPose) {
        return true;
    }

    const frc::Pose2d currentPose = m_drivetrain->GetState().Pose;
    units::meter_t forward_diff = units::math::abs(currentPose.X() - m_targetPose.value().X());
    units::meter_t strafe_diff = units::math::abs(currentPose.Y() - m_targetPose.value().Y());
    units::radian_t rotational_diff = units::math::abs(currentPose.Rotation().Radians() - m_targetPose.value().Rotation().Radians());

    // If the bot is within tolerance for X, Y and rotational position, then we consider the command finished.
    return (strafe_diff < kStrafeTolerance
            && forward_diff < kForwardTolerance
            && rotational_diff < units::radian_t(kRotationTolerance));
}