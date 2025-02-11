#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer() {
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    drivetrain.SetDefaultCommand(drivetrain.ApplyRequest([this]() -> auto&& {
        return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed)
            .WithVelocityY(-joystick.GetLeftX() * MaxSpeed)
            .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate);
    }));

    joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));
    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    // Button Map
    m_gamepad.Button(12).OnTrue(m_led.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1));
    m_gamepad.Button(11).OnTrue(m_led.SetLEDState(ArduinoConstants::RIO_MESSAGES::ALGAE_HELD));
    m_gamepad.Button(10).OnTrue(m_led.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2));
    m_gamepad.Button(9).OnTrue(m_led.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3));
    m_gamepad.Button(8).OnTrue(m_led.SetLEDState(ArduinoConstants::RIO_MESSAGES::IDK));

    // Example for running parallel commands with a single button press
    // m_gamepad.Button(10).OnTrue(frc2::cmd::Parallel(
    //     m_led.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2),
    //     frc2::cmd::RunOnce([this] { frc::SmartDashboard::PutString("Elevator", "L2"); })));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return autoChooser.GetSelected();
}
