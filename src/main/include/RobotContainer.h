// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/CameraSubsystem.h"
#include "Telemetry.h"
#include <frc2/command/RunCommand.h>
#include "subsystems/CoralSubsystem.h"
#include <io/FeatherCanDecoder.h>

class RobotContainer {
private:
    units::meters_per_second_t m_maxSpeed = TunerConstants::kSpeedAt12Volts;
    double m_speedMultiplier = 1.0;
    units::radians_per_second_t m_maxAngularRate = 0.75_tps;

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(m_maxSpeed * 0.1).WithRotationalDeadband(m_maxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};
    swerve::requests::RobotCentric forwardStraight = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{m_maxSpeed};

    frc2::CommandXboxController m_joystick{0};

    CameraSubsystem m_cameraSubsystem;
    CoralSubsystem m_coralSubsystem;

    FeatherCanDecoder* m_featherCanDecoder;

public:
    subsystems::CommandSwerveDrivetrain m_drivetrain{TunerConstants::CreateDrivetrain()};

private:
    /* Path follower */
    frc::SendableChooser<frc2::Command *> m_autoChooser;

public:
    RobotContainer();

    frc2::Command *GetAutonomousCommand();

private:
    void ConfigureBindings();
};
