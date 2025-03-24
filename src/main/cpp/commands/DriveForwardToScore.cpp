// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveForwardToScore.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <chrono>

using namespace std::chrono;

DriveForwardToScore::DriveForwardToScore(subsystems::CommandSwerveDrivetrain* drivetrain, units::inch_t distance)
    : m_drivetrain{drivetrain}, 
    m_forwardDistance{distance} {
    AddRequirements(m_drivetrain);
}

void DriveForwardToScore::Initialize() {
    m_startTime = steady_clock::now();

    m_initialPose = frc::Pose2d(m_drivetrain->GetPose());

    if (m_forwardDistance == 0_in) {
        m_forwardDistance = kDefaultDistance;
    }
}

void DriveForwardToScore::Execute() {
    units::inch_t currentDistanceTraveled = units::inch_t(m_initialPose.Translation().Distance(m_drivetrain->GetPose().Translation()));
    frc::SmartDashboard::PutNumber("Drive Forward Current X", currentDistanceTraveled.value());
    double forward = m_XAlignmentPID.Calculate(currentDistanceTraveled.value(), m_forwardDistance.value());
    forward = std::clamp(forward, -1.0, 1.0);
    m_drivetrain->SetControl(robotOriented.WithVelocityX(forward * kMaxVelocity));
}

bool DriveForwardToScore::IsFinished() {
    //auto endTime = steady_clock::now();
    //auto timeDiff = duration_cast<duration<double>>(endTime - m_startTime);

    // if (timeDiff.count() > 1.5) {
    //     return true;
    // }

    units::inch_t currentDistanceTraveled = units::inch_t(m_initialPose.Translation().Distance(m_drivetrain->GetPose().Translation()));
    return units::math::abs(currentDistanceTraveled - m_forwardDistance) < kTolerance;
}