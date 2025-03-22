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

    frc::SmartDashboard::PutNumber("April Tag Goal Pose X", m_goalPose.X().value());
    frc::SmartDashboard::PutNumber("April Tag Goal Pose Y", m_goalPose.Y().value());
    frc::SmartDashboard::PutNumber("April Tag Goal Pose Rot", m_goalPose.Rotation().Degrees().value());

    m_targetDistance = GetDistance(m_initialPosition, m_goalPose);
    frc::SmartDashboard::PutNumber("Drive Forward Target", m_targetDistance.value());
}

void DriveForwardToScore::Execute() {
    m_currentXDistance = GetDistance(m_initialPosition, m_drivetrain->GetPose());
    frc::SmartDashboard::PutNumber("Drive Forward Current", m_currentXDistance.value());

    double forward = m_XAlignmentPID.Calculate(m_currentXDistance.value(), m_targetDistance.value());
    forward = std::clamp(forward, -1.0, 1.0);
    frc::SmartDashboard::PutNumber("Drive Forward Value", forward);
    
    // m_drivetrain->SetControl(robotOriented.WithVelocityX(forward * kMaxVelocity));
}

bool DriveForwardToScore::IsFinished() {
    return units::math::abs(m_targetDistance - m_currentXDistance) < kTolerance;
}

units::inch_t DriveForwardToScore::GetDistance(frc::Pose2d first, frc::Pose2d second) {
    return units::inch_t(first.Translation().Distance(second.Translation()));
}