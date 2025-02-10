#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer() {
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

    // Initialize the button map
    m_buttonMap[12] = ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1;
    m_buttonMap[11] = ArduinoConstants::RIO_MESSAGES::ALGAE_HELD;
    m_buttonMap[10] = ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2;
    m_buttonMap[9] = ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3;
    m_buttonMap[8] = ArduinoConstants::RIO_MESSAGES::IDK;

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

    for(auto button = m_buttonMap.begin(); button != m_buttonMap.end(); button++) {
        // To get the key: button->first
        // To get the value: button->second
        m_gamepad.Button(button->first).OnTrue(ChangeLEDs(m_led, button->second).ToPtr());
    }
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return autoChooser.GetSelected();
}
