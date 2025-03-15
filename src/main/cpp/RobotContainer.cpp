// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "commands/AlignWithReef.h"
#include <subsystems/LED.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer(FeatherCanDecoder* featherCanDecoder):
m_featherCanDecoder(featherCanDecoder),
m_coralSubsystem(m_featherCanDecoder),
m_algaeSubsystem(m_featherCanDecoder),
m_climberSubsystem(m_featherCanDecoder),
m_scoringSuperstructure(m_elevatorSubsystem, m_coralSubsystem)
{
    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);

    m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);

    ConfigureBindings();

    //m_drivetrain.SetSwervesNeutralValue(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void RobotContainer::ConfigureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.SetDefaultCommand(m_drivetrain.ApplyRequest([this]() -> auto&& {
        // Drivetrain will execute this command periodically
        return drive.WithVelocityX(
                -m_joystick.GetLeftY() * m_maxSpeed * m_speedMultiplier
            ) // Drive forward with negative Y (forward)
            .WithVelocityY(
                -m_joystick.GetLeftX() * m_maxSpeed * m_speedMultiplier
            ) // Drive left with negative X (left)
            .WithRotationalRate(
                -m_joystick.GetRightX() * m_maxAngularRate * m_speedMultiplier
            ); // Drive counterclockwise with negative X (left)
    }));

    m_gamepad.Button(7).OnTrue(frc2::cmd::Parallel(
        frc2::cmd::RunOnce([this] {
            m_drivetrain.SetSwervesNeutralValue(ctre::phoenix6::signals::NeutralModeValue::Coast);
        }),
        m_climberSubsystem.Climb()
    ));

    m_gamepad.Button(11).OnTrue(frc2::cmd::RunOnce([this] {
        m_elevatorSubsystem.PrepareElevator(kElevatorL4Position);
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::IDK);
    }));

    m_gamepad.Button(10).OnTrue(frc2::cmd::RunOnce([this] {
        m_elevatorSubsystem.PrepareElevator(kElevatorL3Position);
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3);
    }));

    m_gamepad.Button(6).OnTrue(frc2::cmd::RunOnce([this] {
        m_elevatorSubsystem.PrepareElevator(kElevatorL2Position);
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2);
    }));

    m_gamepad.Button(3).OnTrue(frc2::cmd::RunOnce([this] {
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ALGAE_HELD);
    }));

    m_gamepad.Button(1).OnTrue(frc2::cmd::RunOnce([this] {
        m_elevatorSubsystem.PrepareElevator(kElevatorL1Position);
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1);
    }));

    m_gamepad.Button(2).OnTrue(frc2::cmd::RunOnce([this] {
        m_elevatorSubsystem.PrepareElevator(kElevatorStowPosition);
    }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // (m_joystick.Back() && m_joystick.Y()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    // (m_joystick.Back() && m_joystick.X()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    // (m_joystick.Start() && m_joystick.Y()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    // (m_joystick.Start() && m_joystick.X()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    m_joystick.X().WhileTrue(AlignWithReef(&m_cameraSubsystem, &m_drivetrain).ToPtr());

    // Temporarily disabling algae to use X for alignment testing
    // m_joystick.X().OnTrue(m_algaeSubsystem.Intake());
    // m_joystick.Y().OnTrue(m_algaeSubsystem.Dispense());

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    m_joystick.POVDown().OnTrue(frc2::cmd::RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));

    m_joystick.RightBumper().OnTrue(
        frc2::cmd::Parallel(
            m_elevatorSubsystem.GoToHeight(kElevatorL2Position),
            frc2::cmd::RunOnce([this] {
                m_coralSubsystem.GoToAngle(55.0);
            })
        )
    );

    m_joystick.LeftBumper().OnTrue(
        frc2::cmd::Parallel(
            m_elevatorSubsystem.GoToHeight(kElevatorL3Position),
            frc2::cmd::RunOnce([this] {
                m_coralSubsystem.GoToAngle(55.0);
            })
        )
    );

    m_joystick.POVUp().OnTrue(
        frc2::cmd::Parallel(
            m_elevatorSubsystem.GoToHeight(kElevatorL4Position),
            frc2::cmd::RunOnce([this] {
                m_coralSubsystem.GoToAngle(50.0);
            })
        )
    );

    m_joystick.POVLeft().OnTrue(
        frc2::cmd::Parallel(
            m_elevatorSubsystem.GoToHeight(kElevatorL1Position),
            frc2::cmd::RunOnce([this] {
                m_coralSubsystem.GoToAngle(65.0);
            })
        )
    );

    m_joystick.RightTrigger().OnTrue(
        m_coralSubsystem.dispenseCoral()
    );

    m_joystick.LeftTrigger().OnTrue(
        m_coralSubsystem.collectCoral()
    );

    m_joystick.A().OnTrue(
        frc2::cmd::Parallel(
            m_elevatorSubsystem.GoToHeight(0_in),
            frc2::cmd::RunOnce([this] {
                m_coralSubsystem.GoToAngle(160.0);
            })
        )
    );

    m_joystick.B().OnTrue(
        frc2::cmd::Parallel(
            m_elevatorSubsystem.GoToHeight(kElevatorCollectPosition),
            frc2::cmd::RunOnce([this] {
                m_coralSubsystem.GoToAngle(125.0);
            })
        )
    );

    (m_elevatorSubsystem.IsHeightAboveThreshold || m_joystick.LeftBumper())
        .OnTrue(
            frc2::cmd::RunOnce([this] {m_speedMultiplier = 0.1;})
        )
        .OnFalse(
            frc2::cmd::RunOnce([this] {m_speedMultiplier = 1.0;})
        );
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}
