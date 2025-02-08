#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <commands/ChangeLEDs.h>

RobotContainer::RobotContainer() {
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

    m_currentLEDMessage = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;
    m_led.SetLEDState(m_currentLEDMessage);

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

    //Button bindings using commands.
    for (const auto& pair : m_buttonMap) {
        int buttonNumber = pair.first;
        m_gamepad.Button(buttonNumber).ToggleOnTrue(new LEDSetCommand(this, m_buttonMap[buttonNumber]));
    }
}

//This is a LED Setting Call
void RobotContainer::SetLED(ArduinoConstants::RIO_MESSAGES message) {
    if (m_currentLEDMessage != message) {
        m_led.SetLEDState(message);
        m_currentLEDMessage = message;
    } else {
        m_led.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);
        m_currentLEDMessage = ArduinoConstants::RIO_MESSAGES::MSG_IDLE;
    }
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return autoChooser.GetSelected();
}
