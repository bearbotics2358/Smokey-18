// RobotContainer.h
#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"
#include <Constants.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <commands/ChangeLEDs.h>
#include <map> // Required for std::map
#include <frc2/command/CommandHelper.h> //For command Based Requirements

class RobotContainer {
private:
    units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
    units::radians_per_second_t MaxAngularRate = 0.75_tps;

    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1)
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};
    swerve::requests::RobotCentric forwardStraight = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController joystick{DriverConstants::kDriverPort};

public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};

private:
    frc::SendableChooser<frc2::Command *> autoChooser;

    // Define LEDSetCommand as a nested class - Now captures 'this'
    class LEDSetCommand : public frc2::CommandHelper<frc2::Command, LEDSetCommand> {
    public:
        LEDSetCommand(RobotContainer* container, ArduinoConstants::RIO_MESSAGES message)
            : m_container(container), m_message(message) {
            AddRequirements(&container->m_led); // Require the LED subsystem
        }

        void Initialize() override {
            m_container->SetLED(m_message); // Call SetLED on the RobotContainer
        }

        bool IsFinished() override {
            return true; // This is an instant command
        }

    private:
        RobotContainer* m_container;
        ArduinoConstants::RIO_MESSAGES m_message;
    };

public:
    RobotContainer();
    frc2::Command *GetAutonomousCommand();

private:
    void ConfigureBindings();
    frc2::CommandGenericHID m_gamepad{4};
    LED m_led;
    ArduinoConstants::RIO_MESSAGES m_currentLEDMessage;
    std::map<int, ArduinoConstants::RIO_MESSAGES> m_buttonMap; // Map button numbers to LED messages
    void SetLED(ArduinoConstants::RIO_MESSAGES message); // SetLED Prototype
};
