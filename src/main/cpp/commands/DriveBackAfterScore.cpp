// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveBackAfterScore.h"

DriveBackAfterScore::DriveBackAfterScore(subsystems::CommandSwerveDrivetrain* drivetrain)
    : m_drivetrain{drivetrain} {
    AddRequirements(m_drivetrain);
}

void DriveBackAfterScore::Initialize() {
    m_targetX = kBackwardDistance + m_drivetrain->GetState().Pose.X();
}

void DriveBackAfterScore::Execute() {
    units::inch_t currentXDistance = m_drivetrain->GetState().Pose.X();
    double forward = m_XAlignmentPID.Calculate(currentXDistance.value(), m_targetX.value());
    forward = std::clamp(forward, -1.0, 1.0);

    m_drivetrain->SetControl(robotOriented.WithVelocityX(-forward * kMaxVelocity));
}

bool DriveBackAfterScore::IsFinished() {
    units::inch_t currentXDistance = m_drivetrain->GetState().Pose.X();
    return units::math::abs(currentXDistance - m_targetX) < kTolerance;
}