// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveForwardToScore.h"

#include <frc/smartdashboard/SmartDashboard.h>

DriveForwardToScore::DriveForwardToScore(subsystems::CommandSwerveDrivetrain* drivetrain, frc::Pose2d goalPose):
m_drivetrain{drivetrain}
{
    m_goalPose = goalPose;
    AddRequirements(m_drivetrain);
}

void DriveForwardToScore::Initialize() {
    m_initialPosition = m_drivetrain->GetPose();

    m_targetDistance = GetDistance(m_initialPosition, m_goalPose);
}

void DriveForwardToScore::Execute() {
    units::inch_t currentXDistance = GetDistance(m_initialPosition, m_drivetrain->GetPose());
    frc::SmartDashboard::PutNumber("Drive Forward Current X", currentXDistance.value());
    double forward = m_XAlignmentPID.Calculate(currentXDistance.value(), m_targetDistance.value());
    forward = std::clamp(forward, -1.0, 1.0);
    m_drivetrain->SetControl(robotOriented.WithVelocityX(forward * kMaxVelocity));
}

bool DriveForwardToScore::IsFinished() {
    units::inch_t currentXDistance = m_drivetrain->GetPose().X();
    return units::math::abs(currentXDistance - m_targetDistance) < kTolerance;
}

units::inch_t DriveForwardToScore::GetDistance(frc::Pose2d first, frc::Pose2d second) {
    return units::inch_t(first.Translation().Distance(second.Translation()));
}