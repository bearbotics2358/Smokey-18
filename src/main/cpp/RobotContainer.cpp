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

using namespace subsystems;

RobotContainer::RobotContainer(FeatherCanDecoder* featherCanDecoder):
m_featherCanDecoder(featherCanDecoder),
m_cameraSubsystem(&m_drivetrain),
m_coralSubsystem(m_featherCanDecoder),
m_algaeSubsystem(m_featherCanDecoder),
m_climberSubsystem(m_featherCanDecoder),
m_scoringSuperstructure(m_elevatorSubsystem, m_coralSubsystem, m_algaeSubsystem, m_drivetrain)
{
    m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);

    if (m_elevatorSubsystem.CurrentHeight() == kElevatorStowPosition) {
        m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);
    }

    m_drivetrain.ConfigNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  
    AddPathPlannerCommands();

    m_drivetrain.ConfigureAutoBuilder();

    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);

    ConfigureBindings();
}

frc2::CommandPtr RobotContainer::AddControllerRumble(double rumble) {
    return frc2::cmd::RunOnce([this, rumble] {
        m_driverJoystick.SetRumble(frc::GenericHID::RumbleType::kBothRumble, rumble);
    });
}

void RobotContainer::ConfigureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    // **** Xbox Drivetrain Buttons **** //
    m_drivetrain.SetDefaultCommand(m_drivetrain.ApplyRequest([this]() -> auto&& {
        // Drivetrain will execute this command periodically
        return drive.WithVelocityX(
                -m_driverJoystick.GetLeftY() * m_maxSpeed * m_speedMultiplier
            ) // Drive forward with negative Y (forward)
            .WithVelocityY(
                -m_driverJoystick.GetLeftX() * m_maxSpeed * m_speedMultiplier
            ) // Drive left with negative X (left)
            .WithRotationalRate(
                -m_driverJoystick.GetRightX() * m_maxAngularRate * m_speedMultiplier
            ); // Drive counterclockwise with negative X (left)
    }));

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    // **** Driver Station Buttons **** //
    m_operatorJoystick.POVUp().OnTrue(frc2::cmd::Parallel(
        frc2::cmd::RunOnce([this] {
            m_drivetrain.ConfigNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
        }),
        m_climberSubsystem.Extend()
    ));

    m_gamepad.Button(7)
        .OnTrue(frc2::cmd::RunOnce([this] {
            m_drivetrain.ConfigNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
            m_climberSubsystem.CancelClimb();
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBLEFTFALSE);
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBRIGHTFALSE);

            if (m_climberSubsystem.IsLeftOnCage()){
                m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBLEFTTRUE);
            } else {
                m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBLEFTFALSE);
            }

            if (m_climberSubsystem.IsRightOnCage()) {
                m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::CLIMBRIGHTTRUE);
            } else {
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

    m_operatorJoystick.POVDown().OnTrue(frc2::cmd::Parallel(
        frc2::cmd::RunOnce([this] {
            m_drivetrain.ConfigNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        }),
        m_climberSubsystem.Stow()
    ));

    m_operatorJoystick.Y().OnTrue(
        frc2::cmd::Parallel(
            m_scoringSuperstructure.PrepareScoring(ScoringSuperstructure::ScoringSelector::L4),
            frc2::cmd::RunOnce([this] {
                m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::IDK);
            })
        ));

    m_operatorJoystick.X().OnTrue(
        frc2::cmd::Parallel(
            m_scoringSuperstructure.PrepareScoring(ScoringSuperstructure::ScoringSelector::L3AlgaeAndCoral),
            frc2::cmd::RunOnce([this] {
                m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3);
            })
        ));

    // m_gamepad.Button(9).OnTrue(frc2::cmd::Parallel(
    //     m_scoringSuperstructure.PrepareScoring(ScoringSuperstructure::ScoringSelector::L3AlgaeOnly),
    //     frc2::cmd::RunOnce([this] {
    //         m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3_ALGAE);
    //     })
    // ));

    m_operatorJoystick.B().OnTrue(frc2::cmd::Parallel(
        m_scoringSuperstructure.PrepareScoring(ScoringSuperstructure::ScoringSelector::L2),
        frc2::cmd::RunOnce([this] {
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2);
        })
    ));

    // m_gamepad.Button(1).OnTrue(frc2::cmd::Parallel(
    //     m_scoringSuperstructure.PrepareScoring(ScoringSuperstructure::ScoringSelector::L1),
    //     frc2::cmd::RunOnce([this] {
    //         m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1);
    //     })
    // ));

    // #TODO: use logic to determine if the robot should go to stow or to processor depending if there is a algae held currently
    m_operatorJoystick.LeftBumper().OnTrue(frc2::cmd::Parallel(
        m_scoringSuperstructure.ToStowPosition(),
        frc2::cmd::RunOnce([this] {
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);
        })
    ));

    m_operatorJoystick.A().OnTrue(frc2::cmd::Parallel(
        m_scoringSuperstructure.ToCollectPosition(),
        frc2::cmd::RunOnce([this] {
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1);
        })
    ));

    // **** Xbox A, B, X, & Y Button functions **** //
    m_driverJoystick.B().WhileTrue(
        frc2::cmd::Either(
            // This checks the state of the L/R reef switch to determine which reef pole to align with
            // Sending false to AlignWithReef aligns right
            AlignWithReef(&m_cameraSubsystem, &m_drivetrain, false).ToPtr().AndThen(AddControllerRumble(1.0)),
            // Sending true to AlignWithReef aligns left
            AlignWithReef(&m_cameraSubsystem, &m_drivetrain, true).ToPtr().AndThen(AddControllerRumble(1.0)),

            [this] { return m_operatorJoystick.POVRight().Get(); }
        )
    ).ToggleOnFalse(
        AddControllerRumble(0.0)
    );

    m_driverJoystick.X().WhileTrue(m_scoringSuperstructure.ScoreIntoProcessor());

    // **** Xbox Trigger & Bumper Buttons **** //
    m_driverJoystick.RightTrigger()
        .OnTrue(
            m_scoringSuperstructure.ScoreIntoReef().FinallyDo(
                [this] { m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE); }
            )
        )
        .OnFalse(
            m_scoringSuperstructure.CancelScore()
        );

    m_driverJoystick.LeftTrigger()
        .OnTrue(
            m_coralSubsystem.collectCoral()
        )
        .OnFalse(
            frc2::cmd::Parallel(
                m_coralSubsystem.StopIntake(),
                m_scoringSuperstructure.ToStowPosition()
            )
        );

    (m_elevatorSubsystem.IsHeightAboveThreshold || m_driverJoystick.LeftBumper())
        .OnTrue(
            frc2::cmd::RunOnce([this] {m_speedMultiplier = 0.1;})
        )
        .OnFalse(
            frc2::cmd::RunOnce([this] {m_speedMultiplier = 1.0;})
        );

    m_driverJoystick.RightBumper().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));

    // **** Xbox Dpad Buttons **** //
    m_driverJoystick.POVUp().OnTrue(m_climberSubsystem.Climb());

    m_driverJoystick.POVDown().OnTrue(frc2::cmd::RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));

    m_driverJoystick.POVRight().WhileTrue(
        m_drivetrain.ApplyRequest([this]() -> auto&& {
            return strafe.WithVelocityY(-0.25_mps);
        })
    );

    m_driverJoystick.POVLeft().WhileTrue(
        m_drivetrain.ApplyRequest([this]() -> auto&& {
            return strafe.WithVelocityY(0.25_mps);
        })
    );
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}


void RobotContainer::AddPathPlannerCommands() {
    using namespace pathplanner;
    NamedCommands::registerCommand(
        "Stow",
        std::move(m_scoringSuperstructure.ToStowPosition())
    );
    NamedCommands::registerCommand(
        "Collect",
        std::move(m_scoringSuperstructure.ToCollectPosition())
    );
    NamedCommands::registerCommand(
        "ScoreL1",
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L1))
    );
    NamedCommands::registerCommand(
        "ScoreL2",
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L2))
    );
    NamedCommands::registerCommand(
        "ScoreL3",
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L3AlgaeAndCoral))
    );
    NamedCommands::registerCommand(
        "ScoreL4",
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L4))
    );
    NamedCommands::registerCommand(
        "ScoreL3AndRemoveAlgae",
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L3AlgaeAndCoral))
    );
    NamedCommands::registerCommand(
        "ScoreAlgae",
        std::move(m_scoringSuperstructure.ScoreIntoProcessor())
    );
    NamedCommands::registerCommand(
        "AlignWithReefLeft",
        std::move(AlignWithReef(&m_cameraSubsystem, &m_drivetrain, true).ToPtr())
    );
}
