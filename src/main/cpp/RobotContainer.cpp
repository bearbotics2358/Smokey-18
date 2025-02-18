// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer()
{
    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.SetDefaultCommand(m_drivetrain.ApplyRequest([this]() -> auto&& {
        // Drivetrain will execute this command periodically
        return drive.WithVelocityX(-m_joystick.GetLeftY() * m_maxSpeed * m_speedMultiplier) // Drive forward with negative Y (forward)
            .WithVelocityY(-m_joystick.GetLeftX() * m_maxSpeed * m_speedMultiplier) // Drive left with negative X (left)
            .WithRotationalRate(-m_joystick.GetRightX() * m_maxAngularRate * m_speedMultiplier); // Drive counterclockwise with negative X (left)
    }));

    m_joystick.LeftBumper()
        .OnTrue(
            frc2::cmd::RunOnce([this] {m_speedMultiplier = 0.2;})
        )
        .OnFalse(
            frc2::cmd::RunOnce([this] {m_speedMultiplier = 1.0;})
        );

    m_joystick.A().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    m_joystick.B().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-m_joystick.GetLeftY(), -m_joystick.GetLeftX()});
    }));

    (m_joystick.X() && m_joystick.Y()).WhileTrue(cameraSubsystem.RunOnce([this] {frc::SmartDashboard::PutNumber("YDistance", cameraSubsystem.getYDistance());} ));
    (m_joystick.B() && m_joystick.Y()).WhileTrue(cameraSubsystem.RunOnce([this] {frc::SmartDashboard::PutNumber("ZRotation", cameraSubsystem.getZRotation());} ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (m_joystick.Back() && m_joystick.Y()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (m_joystick.Back() && m_joystick.X()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (m_joystick.Start() && m_joystick.Y()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (m_joystick.Start() && m_joystick.X()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    m_joystick.LeftBumper().OnTrue(m_drivetrain.RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}
