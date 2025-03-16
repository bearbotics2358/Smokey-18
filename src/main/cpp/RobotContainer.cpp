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
#include <pathplanner/lib/auto/NamedCommands.h>

RobotContainer::RobotContainer(FeatherCanDecoder* featherCanDecoder):
m_featherCanDecoder(featherCanDecoder),
m_cameraSubsystem(&m_drivetrain),
m_coralSubsystem(m_featherCanDecoder),
m_algaeSubsystem(m_featherCanDecoder),
m_climberSubsystem(m_featherCanDecoder),
m_scoringSuperstructure(m_elevatorSubsystem, m_coralSubsystem, m_algaeSubsystem)
{
    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);

    m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);

    if(m_elevatorSubsystem.CurrentHeight() == kElevatorStowPosition){
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);

    // Test this.
    m_drivetrain.ConfigNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  
    ConfigureBindings();

    AddPathPlannerCommands();
}

void RobotContainer::ConfigureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    // **** Xbox Drivetrain Buttons **** //
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

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    // **** Driver Station Buttons **** //

    m_joystick.POVRight().WhileTrue(
        m_drivetrain.ApplyRequest([this] -> auto&& {
            return strafe.WithVelocityY(-0.25_mps);
        })
    );

    m_joystick.POVLeft().WhileTrue(
        m_drivetrain.ApplyRequest([this] -> auto&& {
            return strafe.WithVelocityY(0.25_mps);
        })
    );

    m_gamepad.Button(7).OnTrue(frc2::cmd::Parallel(
        frc2::cmd::RunOnce([this] {
            m_drivetrain.ConfigNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
        }),
        m_climberSubsystem.Climb()
    ));
    
    m_gamepad.Button(11).OnTrue(frc2::cmd::RunOnce([this] {
        m_scoringSuperstructure.PrepareElevator(kElevatorL4Position, false);
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::IDK);
    }));

    m_gamepad.Button(10).OnTrue(frc2::cmd::RunOnce([this] {
        m_scoringSuperstructure.PrepareElevator(kElevatorL3Position, true);
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3);
    }));

    m_gamepad.Button(7).OnTrue(frc2::cmd::RunOnce([this] {
        m_drivetrain.SetSwervesNeutralValue(ctre::phoenix6::signals::NeutralModeValue::Coast);
        m_climberSubsystem.CancelClimb();
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBLEFTFALSE);
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBRIGHTFALSE);
        if (m_climberSubsystem.IsLeftOnCage()){
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBLEFTTRUE);
        }
        else{
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBLEFTFALSE);
        }
        if(m_climberSubsystem.IsRightOnCage()){
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBRIGHTTRUE);
        }
        else{
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBRIGHTFALSE);
        }}))
    .OnFalse(frc2::cmd::RunOnce([this] {
        m_climberSubsystem.CancelClimb();

    }));
    
    // TODO: change the name of ALGAE_HELD
    m_gamepad.Button(9).OnTrue(frc2::cmd::RunOnce([this] {
        m_scoringSuperstructure.PrepareElevator(kElevatorL3Position, true);
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ALGAE_HELD);
    }));

    m_gamepad.Button(6).OnTrue(frc2::cmd::RunOnce([this] {
        m_scoringSuperstructure.PrepareElevator(kElevatorL2Position, false);
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2);
    }));


    m_gamepad.Button(1).OnTrue(frc2::cmd::RunOnce([this] {
        m_scoringSuperstructure.PrepareElevator(kElevatorL1Position, false);
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1);
    }));

    m_gamepad.Button(2).OnTrue(frc2::cmd::RunOnce([this] {
        m_scoringSuperstructure.ToStowPosition();
    }));

    m_gamepad.Button(3).OnTrue(frc2::cmd::RunOnce([this] {
        m_scoringSuperstructure.ToCollectPosition();
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1);
    }));

    // **** Xbox A, B, X, & Y Button functions **** //
    m_joystick.B().WhileTrue(AlignWithReef(&m_cameraSubsystem, &m_drivetrain).ToPtr());

    m_joystick.X().WhileTrue(m_scoringSuperstructure.ScoreIntoProcessor());
  
    // **** Xbox Trigger & Bumper Buttons **** //
    m_joystick.RightTrigger()
        .OnTrue(
            m_scoringSuperstructure.ScoreIntoReef()
        )
        .OnFalse(
            m_elevatorSubsystem.GoToHeight(m_elevatorSubsystem.CurrentHeight())
        );

    m_joystick.LeftTrigger().OnTrue(
        m_coralSubsystem.collectCoral()
    );

    (m_elevatorSubsystem.IsHeightAboveThreshold || m_joystick.LeftBumper())
        .OnTrue(
            frc2::cmd::RunOnce([this] {m_speedMultiplier = 0.1;})
        )
        .OnFalse(
            frc2::cmd::RunOnce([this] {m_speedMultiplier = 1.0;})
        );

    (m_climberSubsystem.IsReadyToClimb && m_gamepad.Button(7))
        .OnTrue(
            frc2::cmd::RunOnce([this] {m_climberSubsystem.Climb();})
        )
        .OnFalse(
            frc2::cmd::RunOnce([this] {m_climberSubsystem.CancelClimb();})
        );

    // **** Xbox Dpad Buttons **** //
    m_joystick.POVUp().OnTrue(m_coralSubsystem.dispenseCoral());

    m_joystick.POVDown().OnTrue(frc2::cmd::RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}

void RobotContainer::AddPathPlannerCommands() {
    using namespace pathplanner;
    NamedCommands::registerCommand(
        "Stow", 
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(kElevatorStowPosition, false))
    );
    NamedCommands::registerCommand(
        "Collect", 
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(kElevatorCollectPosition, false))
    );
    NamedCommands::registerCommand(
        "ScoreL1", 
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(kElevatorL1Position, false))
    );
    NamedCommands::registerCommand(
        "ScoreL2", 
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(kElevatorL2Position, false))
    );
    NamedCommands::registerCommand(
        "ScoreL3", 
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(kElevatorL3Position, false))
    );
    NamedCommands::registerCommand(
        "ScoreL4", 
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(kElevatorL4Position, false))
    );
    NamedCommands::registerCommand(
        "ScoreL3AndRemoveAlgae", 
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(kElevatorL3Position, true))
    );
    NamedCommands::registerCommand(
        "ScoreAlgae", 
        std::move(m_scoringSuperstructure.ScoreIntoProcessor())
    );
}
