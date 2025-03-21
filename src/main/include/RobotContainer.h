// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandGenericHID.h>
#include "io/FeatherCanDecoder.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/CameraSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/CoralSubsystem.h"
#include "subsystems/Climber.h"
#include "subsystems/ScoringSuperstructure.h"
#include "subsystems/LED.h"
#include "Telemetry.h"
#include <frc2/command/RunCommand.h>
#include "subsystems/AlgaeSubsystem.h"

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
    swerve::requests::RobotCentric strafe = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
        .WithVelocityX(0_mps);

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{m_maxSpeed};

    frc2::CommandXboxController m_joystick{0};
    frc2::CommandGenericHID m_gamepad{4};

    // Robot.cpp owns the FeatherCanDecoder object
    FeatherCanDecoder* m_featherCanDecoder;
    LED m_LED;
    CameraSubsystem m_cameraSubsystem;
    ElevatorSubsystem m_elevatorSubsystem;
    CoralSubsystem m_coralSubsystem;
    AlgaeSubsystem m_algaeSubsystem;

    Climber m_climberSubsystem;
    subsystems::ScoringSuperstructure m_scoringSuperstructure;

public:
    subsystems::CommandSwerveDrivetrain m_drivetrain{TunerConstants::CreateDrivetrain()};

private:
    /* Path follower */
    frc::SendableChooser<frc2::Command *> m_autoChooser;

public:
    RobotContainer(FeatherCanDecoder* featherCanDecoder);

    frc2::Command *GetAutonomousCommand();

    frc2::Command *StartLEDS();

    frc2::CommandPtr AddControllerRumble(double rumble);
private:
    void ConfigureBindings();
    void AddPathPlannerCommands();
};
