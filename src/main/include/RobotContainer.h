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
#include <subsystems/LED.h>

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

public:
    RobotContainer();
    frc2::Command *GetAutonomousCommand();

private:
    void ConfigureBindings();
    // TODO: Change `4` into a port constant.
    frc2::CommandGenericHID m_gamepad{4};
    std::map<int, ArduinoConstants::RIO_MESSAGES> m_buttonMap;
    LED* m_led;
};
