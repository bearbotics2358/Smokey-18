// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveRobot.h"

#include <frc/smartdashboard/SmartDashboard.h>

DriveRobot::DriveRobot(
    subsystems::CommandSwerveDrivetrain* drivetrain, 
    DriveDirection direction,
    units::inch_t distance
): 
m_drivetrain{drivetrain}, 
m_direction(direction),
m_distance{distance}
{
    AddRequirements(m_drivetrain);
}

void DriveRobot::Initialize() {
    m_initialPose = m_drivetrain->GetPose();
}

void DriveRobot::Execute() {
    units::inch_t currentDistanceTraveled = GetTraveledDistance();
    frc::SmartDashboard::PutNumber("Drive Distance Current X", currentDistanceTraveled.value());

    double forward = m_XAlignmentPID.Calculate(currentDistanceTraveled.value(), m_distance.value());
    forward = std::clamp(forward, -1.0, 1.0);

    m_drivetrain->SetControl(robotOriented.WithVelocityX(forward * kMaxVelocity * static_cast<int>(m_direction)));
}

bool DriveRobot::IsFinished() {
    units::inch_t currentDistanceTraveled = GetTraveledDistance();
    return units::math::abs(currentDistanceTraveled - m_distance) <= kTolerance;
}

units::inch_t DriveRobot::GetTraveledDistance() {
    return units::inch_t(m_initialPose.Translation().Distance(m_drivetrain->GetPose().Translation()));
}