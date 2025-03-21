// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveForwardToScore.h"

#include <frc/smartdashboard/SmartDashboard.h>

DriveForwardToScore::DriveForwardToScore(subsystems::CommandSwerveDrivetrain* drivetrain)
    : m_drivetrain{drivetrain} {
    AddRequirements(m_drivetrain);
}

void DriveForwardToScore::Initialize() {
    m_targetX = kForwardDistance + m_drivetrain->GetPose().X();
}

void DriveForwardToScore::Execute() {
    units::inch_t currentXDistance = m_drivetrain->GetPose().X();
    frc::SmartDashboard::PutNumber("Drive Forward Current X", currentXDistance.value());
    double forward = m_XAlignmentPID.Calculate(currentXDistance.value(), m_targetX.value());
    forward = std::clamp(forward, -1.0, 1.0);
    m_drivetrain->SetControl(robotOriented.WithVelocityX(forward * kMaxVelocity));
}

bool DriveForwardToScore::IsFinished() {
    units::inch_t currentXDistance = m_drivetrain->GetPose().X();
    return units::math::abs(currentXDistance - m_targetX) < kTolerance;
}