// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <commands/ChangeLEDs.h>

RobotContainer::RobotContainer()
{
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

     (gamepad.GetRawButton(12) || gamepad.GetRawButton(11) || gamepad.GetRawButton(10) || gamepad.GetRawButton(9) || gamepad.GetRawButton(8)).ToggleOnFalse(ChangeLEDs(&m_led, ArduinoConstants::RIO_MESSAGES::MSG_IDLE));
    gamepad.GetRawButton(12).ToggleOnTrue(ChangeLEDs(&m_led, ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1));
    gamepad.GetRawButton(11).ToggleOnTrue(ChangeLEDs(&m_led, ArduinoConstants::RIO_MESSAGES::ALGAE_HELD));
    gamepad.GetRawButton(10).ToggleOnTrue(ChangeLEDs(&m_led, ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2));
    gamepad.GetRawButton(9).ToggleOnTrue(ChangeLEDs(&m_led, ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3));
    gamepad.GetRawButton(8).ToggleOnTrue(ChangeLEDs(&m_led, ArduinoConstants::RIO_MESSAGES::IDK));
    (gamepad.GetRawButton(12) && gamepad.GetRawButton(11) && gamepad.GetRawButton(10) && gamepad.GetRawButton(9) && gamepad.GetRawButton(8)).ToggleOnTrue(ChangeLEDs(&m_led, ArduinoConstants::RIO_MESSAGES::TEST));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected();
    return autoChooser.GetSelected();
}

