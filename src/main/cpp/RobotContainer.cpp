// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer(FeatherCanDecoder* featherCanDecoder):
    m_featherCanDecoder(featherCanDecoder),
    m_coralSubsystem(m_featherCanDecoder),
    m_scoringSuperstructure(m_elevatorSubsystem, m_coralSubsystem)
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

    (m_joystick.X() && m_joystick.Y()).WhileTrue(m_cameraSubsystem.RunOnce([this] {frc::SmartDashboard::PutNumber("YDistance", m_cameraSubsystem.getYDistance());} ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (m_joystick.Back() && m_joystick.Y()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (m_joystick.Back() && m_joystick.X()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (m_joystick.Start() && m_joystick.Y()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (m_joystick.Start() && m_joystick.X()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    m_joystick.POVRight().OnTrue(frc2::cmd::RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));

    // m_joystick.B().OnTrue(
    //     frc2::cmd::RunOnce([this] {
    //         m_coralSubsystem.GoToAngle(55.0);
    //     })
    // );

    // m_joystick.X().OnTrue(
    //     m_coralSubsystem.collectCoral()
    // );
    // m_joystick.Y().OnTrue(
    //     m_coralSubsystem.dispenseCoral()
    // );
    m_joystick.POVDown().OnTrue(
        m_elevatorSubsystem.GoToHeight(0_in)
        // frc2::cmd::Parallel(
        //     m_elevatorSubsystem.GoToHeight(12_in),
        //     frc2::cmd::RunOnce([this] {
        //         m_coralSubsystem.GoToAngle(135.0);
        //     })
        // )
    );

    m_joystick.POVUp().OnTrue(
        m_elevatorSubsystem.GoToHeight(50_in)
        // frc2::cmd::Parallel(
        //     m_elevatorSubsystem.GoToHeight(40_in),
        //     frc2::cmd::RunOnce([this] {
        //         m_coralSubsystem.GoToAngle(55.0);
        //     })
        // )
    );

    m_joystick.A().OnTrue(
        m_elevatorSubsystem.GoToHeight(20_in)
    );
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}
