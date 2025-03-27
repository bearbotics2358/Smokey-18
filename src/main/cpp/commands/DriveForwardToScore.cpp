// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveForwardToScore.h"

#include <frc/smartdashboard/SmartDashboard.h>

DriveForwardToScore::DriveForwardToScore(subsystems::CommandSwerveDrivetrain* drivetrain, units::inch_t distance)
    : m_drivetrain{drivetrain}, 
    m_forwardDistance{distance} {
    AddRequirements(m_drivetrain);
}

void DriveForwardToScore::Initialize() {
    m_initialPose = m_drivetrain->GetPose();
}

void DriveForwardToScore::Execute() {
    units::inch_t currentDistanceTraveled = GetDistance(m_initialPose, m_drivetrain->GetPose());
    frc::SmartDashboard::PutNumber("Drive Forward Current X", currentDistanceTraveled.value());

    double forward = m_XAlignmentPID.Calculate(currentDistanceTraveled.value(), m_forwardDistance.value());
    forward = std::clamp(forward, -1.0, 1.0);

    m_drivetrain->SetControl(robotOriented.WithVelocityX(forward * kMaxVelocity));
}

bool DriveForwardToScore::IsFinished() {
    units::inch_t currentDistanceTraveled = units::inch_t(
        GetDistance(m_initialPose, m_drivetrain->GetPose())
    );
    return units::math::abs(currentDistanceTraveled - m_forwardDistance) <= kTolerance;
}

units::inch_t DriveForwardToScore::GetDistance(frc::Pose2d first, frc::Pose2d second) {
    return units::inch_t(first.Translation().Distance(second.Translation()));
}