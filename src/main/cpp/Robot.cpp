// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"
#include <frc2/command/CommandScheduler.h>
#include "subsystems/LED.h"
#include "commands/ChangeLEDs.h" 
#include <frc2/command/button/CommandGenericHID.h>
#include <Constants.h>
#include <RobotContainer.h>
#include <frc/DriverStation.h>


Robot::Robot() {} 

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();

    if (kUseLimelight) {
        auto const driveState = m_container.drivetrain.GetState();
        auto const heading = driveState.Pose.Rotation().Degrees();
        auto const omega = driveState.Speeds.omega;

        LimelightHelpers::SetRobotOrientation("limelight", heading.value(), 0, 0, 0, 0, 0);
        auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (llMeasurement && llMeasurement->tagCount > 0 && units::math::abs(omega) < 2_tps) {
            m_container.drivetrain.AddVisionMeasurement(llMeasurement->pose, llMeasurement->timestampSeconds);
        }
    }
    
     if (!frc::DriverStation::IsDSAttached()) {
        ArduinoConstants::RIO_MESSAGES::NO_COMMS;
     }

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        m_autonomousCommand->Cancel();
    }
}

void Robot::TeleopPeriodic() {
    
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif